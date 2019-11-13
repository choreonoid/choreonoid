/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "BodyMotionPoseProvider.h"
#include "JointPath.h"
#include "LeggedBodyHelper.h"

using namespace std;
using namespace cnoid;

#if defined(_MSC_VER) && _MSC_VER < 1800
namespace {
inline long lround(double x) {
    return static_cast<long>((x > 0.0) ? floor(x + 0.5) : ceil(x -0.5));
}
}
#endif


BodyMotionPoseProvider::BodyMotionPoseProvider()
{

}


BodyMotionPoseProvider::BodyMotionPoseProvider(Body* body_, std::shared_ptr<BodyMotion> motion)
{
    initialize(body_, motion);
}


void BodyMotionPoseProvider::initialize(Body* body__, std::shared_ptr<BodyMotion> motion)
{
    body_ = body__->clone();
    this->motion = motion;
    footLinkPositions.reset(new MultiSE3MatrixSeq());
    footLinks.clear();
    ikPaths.clear();

    LeggedBodyHelperPtr legged = getLeggedBodyHelper(body_);
    if(legged->isValid()){
        for(int i=0; i < legged->numFeet(); ++i){
            Link* link = legged->footLink(i);
            auto ikPath = JointPath::getCustomPath(body_, body_->rootLink(), link);
            if(ikPath){
                if(ikPath->hasAnalyticalIK() || ikPath->numJoints() == 6){
                    footLinks.push_back(link);
                    ikPaths.push_back(ikPath);
                }
            }
        }
        ZMP_ = legged->homeCopOfSoles();
    } else {
        ZMP_.setZero();
    }
    updateMotion();
}


bool BodyMotionPoseProvider::updateMotion()
{
    const int numFrames = motion->numFrames();
    footLinkPositions->setDimension(numFrames, footLinks.size());
    footLinkPositions->setFrameRate(motion->frameRate());

    auto qseq = motion->jointPosSeq();
    auto pseq = motion->linkPosSeq();

    zmpSeq = getZMPSeq(*motion);

    if(pseq->numParts() < 1){
        return false;
    }

    Link* rootLink = body_->rootLink();
    minNumJoints = std::min(body_->numJoints(), qseq->numParts());
    qTranslated.resize(minNumJoints);

    for(int frame=0; frame < numFrames; ++frame){

        const SE3& p = pseq->at(frame, 0);
        rootLink->p() = p.translation();
        rootLink->R() = p.rotation().toRotationMatrix();

        MultiValueSeq::Frame q = qseq->frame(frame);

        for(int i=0; i < minNumJoints; ++i){
            body_->joint(i)->q() = q[i];
        }

        body_->calcForwardKinematics();
        
        for(size_t i=0; i < footLinks.size(); ++i){
            Link* footLink = footLinks[i];
            Affine3& p = footLinkPositions->at(frame, i);
            p.translation() = footLink->p();
            p.linear() = footLink->R();
        }
    }

    return true;
}


Body* BodyMotionPoseProvider::body() const
{
    return body_.get();
}


double BodyMotionPoseProvider::beginningTime() const
{
    return 0.0;
}


double BodyMotionPoseProvider::endingTime() const
{
    return (motion->numFrames() - 1) / motion->frameRate();
}


bool BodyMotionPoseProvider::seek
(double time, int waistLinkIndex, const Vector3& waistTranslation, bool applyWaistTranslation)
{
    int frame = lround(time * motion->frameRate());
    if(frame >= motion->numFrames()){
        frame = motion->numFrames() - 1;
    }
    const MultiValueSeq::Frame q = motion->jointPosSeq()->frame(frame);
    for(int i=0; i < minNumJoints; ++i){
        qTranslated[i] = q[i];
    }

    if(waistLinkIndex != 0){
        return false;
    }
    
    const SE3& waist = motion->linkPosSeq()->at(frame, 0);
    p_waist = waist.translation();
    R_waist = waist.rotation();
    if(applyWaistTranslation){
        p_waist += waistTranslation;
        for(size_t i=0; i < footLinks.size(); ++i){
            const Affine3& foot = footLinkPositions->at(frame, i);
            auto ikPath = ikPaths[i];
            ikPath->calcInverseKinematics(p_waist, R_waist, foot.translation(), foot.linear());
            for(int j=0; j < ikPath->numJoints(); ++j){
                Link* joint = ikPath->joint(j);
                qTranslated[joint->jointId()] = joint->q();
            }
        }
    }

    if(zmpSeq){
        if(zmpSeq->isRootRelative()){
            ZMP_.noalias() = R_waist * zmpSeq->at(frame) + p_waist;
        } else {
            ZMP_.noalias() = zmpSeq->at(frame);
        }
    }
    
    return true;
}


bool BodyMotionPoseProvider::seek(double time)
{
    return seek(time, 0, Vector3::Zero(), false);
}


bool BodyMotionPoseProvider::seek(double time, int waistLinkIndex, const Vector3& waistTranslation)
{
    return seek(time, waistLinkIndex, waistTranslation, true);
}


int BodyMotionPoseProvider::baseLinkIndex() const
{
    return 0;
}


bool BodyMotionPoseProvider::getBaseLinkPosition(Position& out_T) const
{
    out_T.linear() = R_waist;
    out_T.translation() = p_waist;
    return true;
}


void BodyMotionPoseProvider::getJointPositions(std::vector<stdx::optional<double>>& out_q) const
{
    int n = body_->numJoints();
    out_q.resize(n);
    for(int i=0; i < n; ++i){
        out_q[i] = qTranslated[i];
    }
}


stdx::optional<Vector3> BodyMotionPoseProvider::ZMP() const
{
    return ZMP_;
}
