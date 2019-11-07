/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "BodyMotionUtil.h"
#include "BodyMotion.h"
#include "ZMPSeq.h"
#include "Body.h"
#include "ForceSensor.h"
#include "RateGyroSensor.h"
#include "AccelerationSensor.h"
#include <cnoid/EigenUtil>
#include <cnoid/Vector3Seq>
#include <cnoid/GaussianFilter>
#include <cnoid/RangeLimiter>
#include <cnoid/FileUtil>
#include <fmt/format.h>
#include <fstream>
#include <vector>
#include <limits>
#include <cmath>
#include <algorithm>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

static bool saveRootLinkAttAsRpyFormat(BodyMotion& motion, const std::string& filename, std::ostream& os)
{
    auto linkPosSeq = motion.linkPosSeq();
    const int nFrames = linkPosSeq->numFrames();
        
    if(nFrames > 0 && linkPosSeq->numParts() > 0){
            
        ofstream ofs(filename.c_str());
        if(!ofs){
            os << filename + " cannot be opened.";
            return false;
        }
            
        const double r = linkPosSeq->frameRate();
        MultiSE3Seq::Part root = linkPosSeq->part(0);
        for(int i=0; i < nFrames; ++i){
            Vector3 rpy(rpyFromRot(Matrix3(root[i].rotation())));
            for(int j=0; j < 3; ++j){
                if(fabs(rpy[j]) < 1.0e-14){
                    rpy[j] = 0.0;
                }
            }
            ofs << format("{0:.4f} {1:g} {2:g} {3:g}\n", (i / r), rpy[0], rpy[1], rpy[2]);
        }
            
        return true;
    }
        
    return false;
}


static bool saveRootLinkAccAsGsensFile(BodyMotion& motion, Body* body, const std::string& filename, std::ostream& os)
{
    if(!body){
        return false;
    }

    auto linkPosSeq = motion.linkPosSeq();

    AccelerationSensor* gsens = 0;
    DeviceList<AccelerationSensor> accelSensors(body->devices());
    if(!accelSensors.empty()){
        gsens = accelSensors[0];
    }
    if(!gsens || !gsens->link() || gsens->link()->index() >= linkPosSeq->numParts()){
        return false;
    }

    const int nFrames = linkPosSeq->numFrames();
        
    if(nFrames > 0){
            
        ofstream ofs(filename.c_str());
        ofs.setf(ios::fixed);
        if(!ofs){
            os << filename + " cannot be opened.";
            return false;
        }

        Vector3Seq accSeq;
        calcLinkAccSeq(*linkPosSeq, gsens, 0, nFrames, accSeq);

        const double r = linkPosSeq->frameRate();
        for(int i=0; i < nFrames; ++i){
            Vector3 a = accSeq[i];
            for(int j=0; j < 3; ++j){
                if(fabs(a[j]) < 1.0e-14){
                    a[j] = 0.0;
                }
            }
            ofs << format("{0:.4f} {1:g} {2:g} {3:g}\n", (i / r), a[0], a[1], a[2]);
        }
            
        return true;
    }
        
    return false;
}


/**
   \todo The localRotaion of gsens should be considered.
*/
void cnoid::calcLinkAccSeq
(MultiSE3Seq& linkPosSeq, AccelerationSensor* gsens, int frameBegin, int numFrames, Vector3Seq& out_accSeq)
{
    const double r = linkPosSeq.frameRate();
    const double dt = 1.0 / r;
    vector<Vector3> vseq(numFrames);
    MultiSE3Seq::Part pseq = linkPosSeq.part(gsens->link()->index());

    int last = numFrames - 1;
    for(int i=0; i < last; ++i){
        const SE3& P0 = pseq[frameBegin + i];
        const SE3& P1 = pseq[frameBegin + i + 1];

        // \todo verify the following code
        AngleAxis a(P0.rotation().inverse() * P1.rotation());
        Vector3 w = a.axis() * a.angle() / dt;
        // original code
        //Vector3 w = omegaFromRot(P0.linear().transpose() * P1.linear()) / dt;
        
        vseq[i].noalias() =
            (P1.translation() - P0.translation()) / dt + P0.rotation() * w.cross(gsens->localTranslation());
    }
    vseq[last].setZero();
                
    out_accSeq.setNumFrames(numFrames);
    out_accSeq[0].noalias() = (pseq[frameBegin].rotation().inverse() * vseq[0]) / dt;
    for(int i=1; i < numFrames; ++i){
        out_accSeq[i].noalias() =
            pseq[frameBegin + i].rotation().inverse() * (vseq[i] - vseq[i-1]) / dt;
    }
}


bool cnoid::loadHrpsysSeqFileSet(BodyMotion& motion, const std::string& filename, std::ostream& os)
{
    motion.setNumFrames(0);

    stdx::filesystem::path orgpath(filename);
    
    bool loaded = false;

    shared_ptr<MultiValueSeq> jointPosSeq;
    stdx::filesystem::path posFile(orgpath);
    posFile.replace_extension(".pos");
    if(stdx::filesystem::exists(posFile) && !stdx::filesystem::is_directory(posFile)){
        string posFileString = getNativePathString(posFile);
        jointPosSeq = motion.jointPosSeq();
        if(!jointPosSeq->loadPlainFormat(posFileString)){
            os << jointPosSeq->seqMessage();
            jointPosSeq.reset();
        } else {
            if(posFileString == filename){
                loaded = true;
            }
        }
    }

    shared_ptr<MultiSE3Seq> rootLinkAttSeq;
    stdx::filesystem::path hipFile(orgpath);
    hipFile.replace_extension(".hip");
    if(stdx::filesystem::exists(hipFile) && !stdx::filesystem::is_directory(hipFile)){
        string hipFileString = getNativePathString(hipFile);
        rootLinkAttSeq = motion.linkPosSeq();
        if(!rootLinkAttSeq->loadPlainRpyFormat(hipFileString)){
            os << rootLinkAttSeq->seqMessage();
            rootLinkAttSeq.reset();
        } else {
            if(hipFileString == filename){
                loaded = true;
            }
        }
    }

    shared_ptr<MultiSE3Seq> linkPosSeq;
    stdx::filesystem::path waistFile(orgpath);
    waistFile.replace_extension(".waist");
    if(stdx::filesystem::exists(waistFile) && !stdx::filesystem::is_directory(waistFile)){
        string waistFileString = getNativePathString(waistFile);
        linkPosSeq = motion.linkPosSeq();
        if(!linkPosSeq->loadPlainMatrixFormat(waistFileString)){
            os << linkPosSeq->seqMessage();
            linkPosSeq.reset();
        } else {
            if(waistFileString == filename){
                loaded = true;
            }
        }
    }

    shared_ptr<ZMPSeq> zmpseq;
    if(jointPosSeq || linkPosSeq){
        stdx::filesystem::path zmpFile(orgpath);
        zmpFile.replace_extension(".zmp");
        if(stdx::filesystem::exists(zmpFile) && !stdx::filesystem::is_directory(zmpFile)){
            string zmpFileString = getNativePathString(zmpFile);
            zmpseq = getOrCreateZMPSeq(motion);
            if(!zmpseq->loadPlainFormat(zmpFileString)){
                os << zmpseq->seqMessage();
                clearZMPSeq(motion);
                zmpseq.reset();
            } else {
                if(!linkPosSeq){
                    zmpseq->setRootRelative(true);
                } else {
                    // make the coordinate global
                    MultiSE3Seq::Part rootSeq = linkPosSeq->part(0);
                    for(int i=0; i < zmpseq->numFrames(); ++i){
                        const SE3& p = rootSeq[std::min(i, rootSeq.size() - 1)];
                        (*zmpseq)[i] = p.rotation() * (*zmpseq)[i] + p.translation();
                    }
                    zmpseq->setRootRelative(false);
                }
                if(zmpFileString == filename){
                    loaded = true;
                }
            }
        }
    }

    if(loaded){
        double frameRate = 0.0;
        if(jointPosSeq){
            frameRate = jointPosSeq->frameRate();
        } else if(linkPosSeq){
            frameRate = linkPosSeq->frameRate();
        } else if(zmpseq){
            frameRate = zmpseq->frameRate();
        }
        if((jointPosSeq && jointPosSeq->frameRate() != frameRate) ||
           (linkPosSeq && linkPosSeq->frameRate() != frameRate) ||
           (zmpseq && zmpseq->frameRate() != frameRate)){
            os << "Frame rate is not consistent.";
            loaded = false;
        } else {
            motion.setFrameRate(frameRate);
        }
    }
    if(!loaded){
        motion.setNumFrames(0);
    }

    return loaded;
}


bool cnoid::saveHrpsysSeqFileSet(BodyMotion& motion, Body* body, const std::string& filename, std::ostream& os)
{
    stdx::filesystem::path orgpath(filename);

    if(motion.jointPosSeq()->saveAsPlainFormat(
           getNativePathString(
               stdx::filesystem::path(orgpath).replace_extension(".pos")), os) &&
           
       motion.linkPosSeq()->saveTopPartAsPosAndRPYFormat(
           getNativePathString(
               stdx::filesystem::path(orgpath).replace_extension(".waist")), os) &&
           
       saveRootLinkAttAsRpyFormat(
           motion, getNativePathString(
               stdx::filesystem::path(orgpath).replace_extension(".hip")), os)) {

        saveRootLinkAccAsGsensFile(
            motion, body, getNativePathString(
                stdx::filesystem::path(orgpath).replace_extension(".gsens")), os);

        auto zmpseq = getZMPSeq(motion);
        if(zmpseq){
            // make the coordinate relative to the waist
            Vector3Seq relZMP(zmpseq->numFrames());
            relZMP.setFrameRate(zmpseq->frameRate());
            MultiSE3Seq::Part rootSeq = motion.linkPosSeq()->part(0);
            for(int i=0; i < zmpseq->numFrames(); ++i){
                const SE3& p = rootSeq[std::min(i, rootSeq.size() - 1)];
                relZMP[i].noalias() = p.rotation().inverse() * (zmpseq->at(i) - p.translation());
            }
            return relZMP.saveAsPlainFormat(
                getNativePathString(stdx::filesystem::path(orgpath).replace_extension(".zmp")));
        }
        return true;
    }
    return false;
}


static void applyPollardVelocityLimitFilterSub
(MultiValueSeq::Part seq, double deltaUVLimit, double deltaLVLimit, double deltaKs,
 vector<double>& forward, vector<double>& backward)
{
    const double sqrtDeltaKs = sqrt(deltaKs);
    const int n = seq.size();
    const int last = n - 1;
    
    double v0, vv0, vv1; 
    double aa1;

    // forward pass
    forward[0] = seq[0];
    v0 = 0.0;
    vv0 = 0.0;
    for(int i = 0; i < last; ++i){
        aa1 = 2.0 * sqrtDeltaKs * (v0 - vv0) + deltaKs * (seq[i] - forward[i]);
        vv1 = vv0 + aa1;
        if(vv1 > deltaUVLimit){
            vv1 = deltaUVLimit;
        } else if(vv1 < deltaLVLimit) {
            vv1 = deltaLVLimit;
        }
        forward[i+1] = forward[i] + vv1;
        v0 = seq[i+1] - seq[i];
        vv0 = vv1;
    }
    
    // backward pass
    v0 = 0.0;
    vv0 = 0.0;
    backward[last] = seq[last];
    for(int i = last; i > 0; --i) {
        aa1 = 2.0 * sqrtDeltaKs * (v0 - vv0) + deltaKs * (seq[i] - backward[i]);
        vv1 = vv0 + aa1;
        if(vv1 > -deltaLVLimit){
            vv1 = -deltaLVLimit;
        } else if(vv1 < -deltaUVLimit){
            vv1 = -deltaUVLimit;
        }
        backward[i-1] = backward[i] + vv1;
        v0 = seq[i-1] - seq[i];
        vv0 = vv1;
    }
    
    // average the forward and backward passes 
    for(int i=0; i < n; i++){
        seq[i] = 0.5 * (forward[i] + backward[i]);
    }
}


static void applyVelocityLimitFilterSub
(MultiValueSeq::Part seq, double deltaUVLimit, double deltaLVLimit,
 vector<double>& forward, vector<double>& backward)
{
    const int n = seq.size();
    const int last = n - 1;

    // forward pass
    forward[0] = seq[0];
    for(int i = 0; i < last; ++i){
        double v = (seq[i+1] - forward[i]);
        if(v > deltaUVLimit){
            v = deltaUVLimit;
        } else if(v < deltaLVLimit){
            v = deltaLVLimit;
        }
        forward[i+1] = forward[i] + v;
    }
    
    // backward pass
    backward[last] = seq[last];
    for(int i = last; i > 0; --i) {
        double v = (seq[i-1] - backward[i]);
        if(v > -deltaLVLimit){
            v = -deltaLVLimit;
        } else if(v < -deltaUVLimit){
            v = -deltaUVLimit;
        }
        backward[i-1] = backward[i] + v;
    }
    
    // average the forward and backward passes 
    for(int i=0; i < n; i++){
        seq[i] = 0.5 * (forward[i] + backward[i]);
    }
}


bool cnoid::applyVelocityLimitFilter2(MultiValueSeq& seq, int part, double absLimit)
{
    const int numFrames = seq.numFrames();
    const double frameRate = seq.frameRate();
    vector<double> forward(numFrames);
    vector<double> backward(numFrames);
    const double deltaUVLimit = absLimit / frameRate;
    const double deltaLVLimit = -absLimit / frameRate;
    applyVelocityLimitFilterSub(seq.part(part), deltaUVLimit, deltaLVLimit, forward, backward);

    return true;
}


bool cnoid::applyVelocityLimitFilterDummy()
{
    return false;
}


static bool applyVelocityLimitFilterMain
(MultiValueSeq& seq, Body* body, double ks, bool usePollardMethod, std::ostream& os)
{
    bool applied = false;
    
    os << "applying the velocity limit filter ..." << endl;
    
    const int numParts = seq.numParts();
    const int numFrames = seq.numFrames();
    const double frameRate = seq.frameRate();
    const double deltaKs = ks / frameRate;
    
    vector<double> forward(numFrames);
    vector<double> backward(numFrames);
    
    int n = std::min(numParts, body->numJoints());
    for(int i=0; i < n; ++i){
        Link* joint = body->joint(i);
        if(joint->dq_upper() != std::numeric_limits<double>::max() ||
           joint->dq_lower() != -std::numeric_limits<double>::max()){
            const double deltaUVLimit = joint->dq_upper() / frameRate;
            const double deltaLVLimit = joint->dq_lower() / frameRate;
            
            os << format(" seq {0}: lower limit = {1}, upper limit = {2}",
                    i, joint->dq_lower(), joint->dq_upper()) << endl;

            if(usePollardMethod){
                applyPollardVelocityLimitFilterSub(
                    seq.part(i), deltaUVLimit, deltaLVLimit, deltaKs, forward, backward);
            } else {
                applyVelocityLimitFilterSub(
                    seq.part(i), deltaUVLimit, deltaLVLimit, forward, backward);
            }
                    
            applied = true;
        }
    }
    return applied;
}


bool cnoid::applyPollardVelocityLimitFilter
(MultiValueSeq& seq, Body* body, double ks, std::ostream& os)
{
    return applyVelocityLimitFilterMain(seq, body, ks, true, os);
}


bool cnoid::applyVelocityLimitFilter
(MultiValueSeq& seq, Body* body, std::ostream& os)
{
    return applyVelocityLimitFilterMain(seq, body, 1.0, false, os);
}


void cnoid::applyGaussianFilter
(MultiValueSeq& seq, double sigma, int range, std::ostream& os)
{
    vector<double> gwin;
    setGaussWindow(sigma, range, gwin);
    
    vector<double> orgseq(seq.numFrames());
    
    for(int i=0; i < seq.numParts(); ++i){
        if(i==0){
            os << format(_("applying the gaussian filter (sigma = {0}, range = {1}) to seq"),
                    sigma, range) << endl;
        }
        os << " " << i;

        MultiValueSeq::Part part = seq.part(i);
        std::copy(part.begin(), part.end(), orgseq.begin());
        applyGaussianFilter(part, orgseq, gwin, 0.0);
    }
}


void cnoid::applyRangeLimitFilter
(MultiValueSeq& seq, Body* body, double limitGrad, double edgeGradRatio, double margin, std::ostream& os)
{
    RangeLimiter limiter;

    os << "applying the joint position range limit filter ..." << endl;
    
    const int numParts = seq.numParts();
    
    int n = std::min(numParts, body->numJoints());
    for(int i=0; i < n; ++i){
        Link* joint = body->joint(i);
        if(joint->q_upper() != std::numeric_limits<double>::max() ||
           joint->q_lower() != -std::numeric_limits<double>::max()){
            const double upper = joint->q_upper() - margin;
            const double lower = joint->q_lower() + margin;
            if(upper > lower){
                os << format(" seq {0}: lower limit = {1}, upper limit = {2}",
                        i, joint->q_lower(), joint->q_upper()) << endl;

                MultiValueSeq::Part part = seq.part(i);
                limiter.apply(part, upper, lower, limitGrad, edgeGradRatio);
            }
        }
    }
}
