/**
   @author Shin'ichiro Nakaoka
*/

#include "WaistBalancer.h"
#include <cnoid/PoseProvider>
#include <cnoid/LeggedBodyHelper>
#include <cnoid/ZMPSeq>
#include <cnoid/EigenUtil>
#include <cnoid/NullOut>
#include <cnoid/GaussianFilter>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

    const bool DoVerticalAccCompensation = true;
    
#if defined(_MSC_VER) && _MSC_VER < 1800
    inline long lround(double x) {
        return static_cast<long>((x > 0.0) ? floor(x + 0.5) : ceil(x -0.5));
    }
#endif

    struct Zelements {
        vector<Vector3>& array;
        int offset;
        Zelements(vector<Vector3>& array, int offset) : array(array), offset(offset) { }
        double& operator[](int index) { return array[index + offset].z(); }
    };
}


WaistBalancer::WaistBalancer()
    : os_(&nullout())
{
    g = 9.8;
    waistLink = 0;
    numIterations_ = 2;
    boundaryConditionType = KEEP_POSITIONS;
    initialWaistTrajectoryMode = ORG_TRAJECTORY;
    timeToStartBalancer = 0.0;
    preInitialDuration = 0.0;
    postFinalDuration = 0.0;
    dynamicsTimeRatio = 1.0;
    isBoundaryCmAdjustmentEnabled = false;
    isWaistHeightRelaxationEnabled = false;

    setBoundarySmoother(QUINTIC_SMOOTHER, 0.5);
    setFullTimeRange();
}


void WaistBalancer::setBody(const BodyPtr& body)
{
    body_ = body;
    waistLink = 0;
}


const BodyPtr& WaistBalancer::body() const
{
    return body_;
}


void WaistBalancer::setWaistLink(Link* waistLink)
{
    this->waistLink = waistLink;
}


void WaistBalancer::setNumIterations(int n)
{
    numIterations_ = n;
}


int WaistBalancer::numIterations() const
{
    return numIterations_;
}


void WaistBalancer::setTimeRange(double lower, double upper)
{
    timeRangeLower = lower;
    timeRangeUpper = std::max(lower, upper);
}


void WaistBalancer::setFullTimeRange()
{
    timeRangeLower = 0.0;
    timeRangeUpper = std::numeric_limits<double>::max();
}


void WaistBalancer::setTimeMargins(double timeToStartBalancer, double preInitialDuration, double postFinalDuration)
{
    this->timeToStartBalancer = timeToStartBalancer;
    this->preInitialDuration = preInitialDuration;
    this->postFinalDuration = postFinalDuration;
}


void WaistBalancer::setGravity(double g)
{
    this->g = g;
}


void WaistBalancer::setDynamicsTimeRatio(double r)
{
    dynamicsTimeRatio = r;
}


const char* WaistBalancer::boundaryConditionTypeNameOf(int type)
{
    if(type == ZERO_VELOCITY){
        return N_("zero-velocity");
    }
    return N_("position");
}

    
int WaistBalancer::boundaryConditionTypeOf(const std::string& name)
{
    if(name == N_("zero-velocity")){
        return ZERO_VELOCITY;
    }
    return KEEP_POSITIONS;
}


void WaistBalancer::setBoundaryConditionType(int type)
{
    boundaryConditionType = type;
}


const char* WaistBalancer::boundarySmootherTypeNameOf(int type)
{
    switch(type){
    case CUBIC_SMOOTHER:
        return N_("cubic");
    case QUINTIC_SMOOTHER:
        return N_("quintic");
    }
    return N_("off");
}


int WaistBalancer::boundarySmootherTypeOf(const std::string& name)
{
    if(name == N_("cubic")){
        return CUBIC_SMOOTHER;
    } else if(name == N_("quintic")){
        return QUINTIC_SMOOTHER;
    }
    return NO_SMOOTHER;
}


void WaistBalancer::setBoundarySmoother(int type, double smoothingTime)
{
    boundarySmootherType_ = type;
    if(type == CUBIC_SMOOTHER){
        boundarySmootherFunction =
            [&](int begin, int direction){ applyCubicBoundarySmoother(begin, direction); };
    } else if(type == QUINTIC_SMOOTHER){
        boundarySmootherFunction =
            [&](int begin, int direction){ applyQuinticBoundarySmoother(begin, direction); };
    } else {
        boundarySmootherType_ = NO_SMOOTHER;
    }
        
    boundarySmoothingTime = smoothingTime;
}


void WaistBalancer::enableBoundaryCmAdjustment(bool on, double transitionTime)
{
    isBoundaryCmAdjustmentEnabled = on;
    boundaryCmAdjustmentTransitionTime = transitionTime;
}


void WaistBalancer::setInitialWaistTrajectoryMode(int mode)
{
    initialWaistTrajectoryMode = mode;
}


void WaistBalancer::enableWaistHeightRelaxation(bool on)
{
    isWaistHeightRelaxationEnabled = on;
}


bool WaistBalancer::apply(PoseProvider* provider_, BodyMotion& motion, bool putAllLinkPositions)
{
    if(!body_){
        return false;
    }

    if(!waistLink){
        waistLink = body_->rootLink();
    }
    waistLinkIndex = waistLink->index();
    
    provider = provider_;

    frameRate = motion.frameRate();
    timeStep = 1.0 / frameRate;
    const double dynamicsFrameRate = dynamicsTimeRatio * frameRate;
    dt = 1.0 / dynamicsFrameRate;
    dt2 = dt / dynamicsFrameRate;
    m = body_->mass();
    mg = m * g;
    isCalculatingInitialWaistTrajectory = false;

    // preserve the original body state
    Link* rootLink = body_->rootLink();
    const int numJoints = body_->numJoints();
    q0.resize(numJoints);
    for(int i=0; i < numJoints; ++i){
        q0[i] = body_->joint(i)->q();
    }
    p0 = rootLink->p();
    R0 = rootLink->R();

    bool result = apply2(motion, putAllLinkPositions);

    // restore the original body state
    for(int i=0; i < numJoints; ++i){
        body_->joint(i)->q() = q0[i];
    }
    rootLink->p() = p0;
    rootLink->R() = R0;
    body_->calcForwardKinematics();

    return result;
}


bool WaistBalancer::apply2(BodyMotion& motion, bool putAllLinkPositions)
{
    targetBeginningTime = std::max(provider->beginningTime(), timeRangeLower);
    targetEndingTime = std::min(provider->endingTime(), timeRangeUpper);
    
    beginningFrame = std::max(0L, lround((targetBeginningTime - preInitialDuration) * frameRate));
    endingFrame = lround((targetEndingTime + postFinalDuration) * frameRate);
    frameToStartBalancer = beginningFrame + lround(timeToStartBalancer * frameRate);
    numFilteredFrames = endingFrame - frameToStartBalancer + 1;
    
    if(numFilteredFrames < 3){
        return false;
    }

    // prepare for the waist height relaxation
    doWaistHeightRelaxation = false;
    doStoreOriginalWaistFeetPositionsForWaistHeightRelaxation = false;
    if(isWaistHeightRelaxationEnabled){
        initWaistHeightRelaxation();
    }
    
    initBodyKinematics(endingFrame, Vector3::Zero());

    if(provider->baseLinkIndex() < 0){
        return false;
    }
    if(!provider->ZMP()){
        return false;
    }

    Vector3 cmProjection(cm[0], cm[1], desiredZmp[2]);
    doProcessFinalBoundary = ((cmProjection - desiredZmp).norm() < 2.0e-3);

    doBoundarySmoother = false;
    if(boundarySmootherType_ && boundaryConditionType == KEEP_POSITIONS){
        numBoundarySmoothingFrames = lround(boundarySmoothingTime * frameRate);
        doBoundarySmoother = (numBoundarySmoothingFrames < numFilteredFrames / 3);
    }

    if(numIterations_ == 0 || initialWaistTrajectoryMode == ORG_TRAJECTORY){
        totalCmTranslations.clear();
        totalCmTranslations.resize(endingFrame + 1, Vector3::Zero());

        if(isBoundaryCmAdjustmentEnabled){
            calcBoundaryCmAdjustmentTrajectory();
        }
    } else {
        totalCmTranslations.resize(endingFrame + 1);
        isCalculatingInitialWaistTrajectory = true;
        doBoundaryCmAdjustment = false;
    }

    coeffSeq.resize(numFilteredFrames);

    bool result = true;
    
    for(int i=0; i < numIterations_; ++i){
        result = calcCmTranslations();
        if(!result){
            break;
        }
    }

    if(result){
        result = applyCmTranslations(motion, putAllLinkPositions);
    }

    return result;
}


void WaistBalancer::calcBoundaryCmAdjustmentTrajectory()
{
    boundaryCmAdjustmentTranslations.resize(totalCmTranslations.size());

    // clear only the area where the smoother is applied
    if(doBoundarySmoother){
        for(int i=0; i < numBoundarySmoothingFrames; ++i){
            boundaryCmAdjustmentTranslations[i + frameToStartBalancer].setZero();
            boundaryCmAdjustmentTranslations[endingFrame - i].setZero();
        }
    }

    boundaryCmAdjustmentTransitionHalfLength = lround((boundaryCmAdjustmentTransitionTime / 2.0) * frameRate);

    doBoundaryCmAdjustment = calcBoundaryCmAdjustmentTrajectorySub(0, 1);
    if(calcBoundaryCmAdjustmentTrajectorySub(endingFrame, -1)){
        doProcessFinalBoundary = true;
        doBoundaryCmAdjustment = true;
    }
}


bool WaistBalancer::calcBoundaryCmAdjustmentTrajectorySub(int begin, int direction)
{
    bool adjusted = true;
    int i;
    Vector3 translation;

    provider->seek(timeOfFrame(begin));
    Vector3 zmp0 = *provider->ZMP();

    // In the first half period, CM stays above the initial ZMP position
    for(i=0; i < boundaryCmAdjustmentTransitionHalfLength; ++i){
        int frame = begin + i * direction;
        if(!calcWaistTranslationWithCmAboveZmp(frame, zmp0, translation)){
            adjusted = false;
            break;
        }
        boundaryCmAdjustmentTranslations[frame] = translation;
        totalCmTranslations[frame] = translation;
    }
    if(adjusted){
        // In the latter half period, CM is gradually corresponding to the original positions
        const double time = boundaryCmAdjustmentTransitionHalfLength / frameRate;
        double time2 = time * time;
        double a2 = -3.0 / time2;
        double a3 = 2.0 / (time * time2);
        
        i = static_cast<int>(boundaryCmAdjustmentTransitionHalfLength);
        const int n = static_cast<int>(boundaryCmAdjustmentTransitionHalfLength * 2.0);
        while(i < n){
            int frame = begin + i * direction;
            if(!calcWaistTranslationWithCmAboveZmp(frame, zmp0, translation)){
                adjusted = false;
                break;
            }
            double t = (i - boundaryCmAdjustmentTransitionHalfLength) * timeStep;
            double r = 1.0 + (a2 + a3 * t) * (t * t);
            Vector3 p = r * translation;
            boundaryCmAdjustmentTranslations[frame] = p;
            totalCmTranslations[frame] = p;
			++i;
        }
    }

    if(!adjusted){
        // reset translations
        for(int j=0; j < i; ++j){
            int frame = begin + j * direction;
            boundaryCmAdjustmentTranslations[frame].setZero();
            totalCmTranslations[frame].setZero();
        }
    }

    return adjusted;
}


bool WaistBalancer::calcWaistTranslationWithCmAboveZmp
(int frame, const Vector3& zmp, Vector3& out_translation)
{
    out_translation.setZero();

    double time = timeOfFrame(frame);
    provider->seek(time, waistLinkIndex, out_translation);

    int baseLinkIndex = provider->baseLinkIndex();
    if(baseLinkIndex < 0){
        return false;
    }
    baseLink = body_->link(baseLinkIndex);
    fkTraverse.find(baseLink);

    bool converged = false;
    const int n = body_->numJoints();

    for(int i=0; i < 50; ++i){

        provider->getBaseLinkPosition(baseLink->T());
        
        provider->getJointPositions(jointPositions);
        for(int j=0; j < n; ++j){
            Link* joint = body_->joint(j);
            const auto& q = jointPositions[j];
            joint->q() = q ? *q : 0.0;
        }
        fkTraverse.calcForwardKinematics(true);
        cm = body_->calcCenterOfMass();

        Vector3 diff(zmp[0] - cm[0], zmp[1] - cm[1], 0.0);

        if(diff.norm() < 1.0e-6){
            converged = true;
            break;
        }
        out_translation += diff;
        provider->seek(time, waistLinkIndex, out_translation);
    }

    return converged;
}


void WaistBalancer::initBodyKinematics(int frame, const Vector3& cmTranslation)
{
    provider->seek(timeOfFrame(frame), waistLinkIndex, cmTranslation);

    int baseLinkIndex = provider->baseLinkIndex();
    if(baseLinkIndex >= 0){
        baseLink = body_->link(baseLinkIndex);
        provider->getBaseLinkPosition(baseLink->T());
    } else {
        baseLink = body_->rootLink();
        baseLink->p().setZero();
        baseLink->R().setIdentity();
    }
    baseLink->v().setZero();
    baseLink->w().setZero();
    
    fkTraverse.find(baseLink);

    const int n = body_->numJoints();
    provider->getJointPositions(jointPositions);
    for(int i=0; i < n; ++i){
        Link* joint = body_->joint(i);
        const auto& q = jointPositions[i];
        joint->q() = q ? *q : 0.0;
        joint->dq() = 0.0;
    }

    updateCmAndZmp(frame);
}


void WaistBalancer::updateCmAndZmp(int frame)
{
    fkTraverse.calcForwardKinematics(true);

    cm = body_->calcCenterOfMass();

    if(isCalculatingInitialWaistTrajectory){
        Vector3& p = totalCmTranslations[frame];
        p.x() = -waistLink->p().x();
        p.y() = -waistLink->p().y();
        p.z() = 0.0;
        desiredZmp = *provider->ZMP();
        zmpDiff = desiredZmp;

    } else {
        Vector3 P, L;
        body_->calcTotalMomentum(P, L);

        dP = (P - P0) / dt;
        dL = (L - L0) / dt;

        P0 = P;
        L0 = L;

        const double inertial_g_thresh = 1.0;
        double ddz = dP.z() / m;
        inertial_g = g + ddz;
        
        if(inertial_g < inertial_g_thresh){
            os() << fmt::format(
                _("Warning: The body is floating at {0} (Vertical CM acceleration is {1})."),
                (frame * timeStep), (ddz))
                 << endl;

            if(DoVerticalAccCompensation){
                dP.z() = m * (inertial_g_thresh - g);
                inertial_g = inertial_g_thresh;
            }
        }

        zmp.x() = (dP.x() * desiredZmp.z() - dL.y() + mg * cm.x()) / (dP.z() + mg);
        zmp.y() = (dP.y() * desiredZmp.z() + dL.x() + mg * cm.y()) / (dP.z() + mg);
        zmp.z() = desiredZmp.z();
        zmpDiff = desiredZmp - zmp;

        desiredZmp = *provider->ZMP();
    }
}


bool WaistBalancer::updateBodyKinematics1(int frame)
{
    bool result = true;
    
    const int n = body_->numJoints();
    const int nextFrame = frame + 1;

    if(nextFrame <= endingFrame){

        // update velocities
        result = provider->seek(timeOfFrame(nextFrame), waistLinkIndex, totalCmTranslations[nextFrame]);

        if(!isCalculatingInitialWaistTrajectory){
        
            const int baseLinkIndex = provider->baseLinkIndex();
            if(baseLinkIndex != baseLink->index() && baseLinkIndex >= 0){
                baseLink = body_->link(baseLinkIndex);
                fkTraverse.find(baseLink);
            }
        
            Position T_next;
            if(!provider->getBaseLinkPosition(T_next)){
                T_next = baseLink->T();
            }
            baseLink->v() = (T_next.translation() - baseLink->p()) / dt;
            baseLink->w() = omegaFromRot(baseLink->R().transpose() * T_next.linear()) / dt;

            provider->getJointPositions(jointPositions);
            for(int i=0; i < n; ++i){
                Link* joint = body_->joint(i);
                const auto& q = jointPositions[i];
                if(q){
                    joint->dq() = (*q - joint->q()) / dt;
                } else {
                    joint->dq() = 0.0;
                }
            }
        }
    }

    updateCmAndZmp(frame);

    return result;
}


void WaistBalancer::updateBodyKinematics2()
{
    const int n = body_->numJoints();
    provider->getJointPositions(jointPositions);
    for(int i=0; i < n; ++i){
        Link* joint = body_->joint(i);
        const auto& q = jointPositions[i];
        if(q){
            joint->q() = *q;
        }
    }
    provider->getBaseLinkPosition(baseLink->T());
}


bool WaistBalancer::calcCmTranslations()
{
    initBodyKinematics(frameToStartBalancer, totalCmTranslations[frameToStartBalancer]);

    //const double gdt2 = g * dt2;
    
    for(int i = 0; i < numFilteredFrames; ++i){

        updateBodyKinematics1(i + frameToStartBalancer);

        if(doStoreOriginalWaistFeetPositionsForWaistHeightRelaxation){
            // store waist and feet positions
            WaistFeetPos& p = waistFeetPosSeq[i];
            p.p_Waist = waistLink->p();
            p.R_Waist = waistLink->R();
            for(int j=0; j < 2; ++j){
                Link* footLink = waistFeetIK.baseLink(j);
                p.p_Foot[j] = footLink->p();
                p.R_Foot[j] = footLink->R();
            }
        }

        updateBodyKinematics2();

        Coeff& c = coeffSeq[i];
        /*
        c.a = -cm.z();
        c.b = 2.0 * cm.z() + gdt2;
        c.d = gdt2 * zmpDiff;
        */

        if(DoVerticalAccCompensation){
            const double gdt2 = inertial_g * dt2;
            c.a = -cm.z() / gdt2;
            c.b = 2.0 * cm.z() / gdt2 + 1.0;
        } else {
            const double gdt2 = g * dt2;        
            c.a = -cm.z() / gdt2;
            c.b = 2.0 * cm.z() / gdt2 + 1.0;
        }
        c.d = zmpDiff;
    }
    
    double bet;
    vector<double> gam(numFilteredFrames);
    vector<Vector3> u(numFilteredFrames, Vector3::Zero());
    int last = numFilteredFrames - 1;

    Coeff& c0 = coeffSeq[0];
    Coeff& clast = coeffSeq[last];

    if(boundaryConditionType == ZERO_VELOCITY){
        c0.b += c0.a;
        clast.b += clast.a;
    }

    bet = c0.b;
    
    u[0].x() = c0.d.x() / bet;
    u[0].y() = c0.d.y() / bet;

    for(int i=1; i < numFilteredFrames - 2; ++i){
        Coeff& cprev = coeffSeq[i-1];
        Coeff& c = coeffSeq[i];
        gam[i] = cprev.a / bet;
        bet = c.b - c.a * gam[i];
        u[i].x() = (c.d.x() - c.a * u[i-1].x()) / bet;
        u[i].y() = (c.d.y() - c.a * u[i-1].y()) / bet;
    }

    gam[last] = coeffSeq[last-1].a / bet;

    bet = clast.b - clast.a * gam[last];
    
    u[last].x() = (clast.d.x() - clast.a * u[last-1].x()) / bet;
    u[last].y() = (clast.d.y() - clast.a * u[last-1].y()) / bet;
    totalCmTranslations[last + frameToStartBalancer] += u[last];

    for(int i = numFilteredFrames - 2; i >= 0; --i){
        u[i].x() -= gam[i+1] * u[i+1].x();
        u[i].y() -= gam[i+1] * u[i+1].y();
        Vector3& translation = totalCmTranslations[i + frameToStartBalancer];
        translation.x() += u[i].x();
        translation.y() += u[i].y();
        double sqrlen = translation.squaredNorm();
        static const double thresh = 1.0;
        if(sqrlen > thresh * thresh){ // divergence error
            translation *= thresh / sqrt(sqrlen);
            // Error should be notified ?
        }
    }

    if(doWaistHeightRelaxation){
        doStoreOriginalWaistFeetPositionsForWaistHeightRelaxation = false;
        relaxWaistHeightTrajectory();
    }

    isCalculatingInitialWaistTrajectory = false;

    return true;
}


void WaistBalancer::initWaistHeightRelaxation()
{
    LeggedBodyHelperPtr legged = getLeggedBodyHelper(body_);

    if(!legged->isValid() || legged->numFeet() != 2){
        os() << _("Waist height relaxation cannot be applied because the robot is not a biped robot.") << endl;

    } else {
        rightKneePitchJoint = legged->kneePitchJoint(0);
        leftKneePitchJoint = legged->kneePitchJoint(1);

        if(!rightKneePitchJoint || !leftKneePitchJoint){
            os() << _("Waist height relaxation cannot be applied because the knee joints are not specified.") << endl;

        } else {
            
            doWaistHeightRelaxation = true;
            doStoreOriginalWaistFeetPositionsForWaistHeightRelaxation = true;
            waistFeetPosSeq.resize(numFilteredFrames);
            waistDeltaZseq.resize(numFilteredFrames);

            if(waistFeetIK.body() != body_){
                waistFeetIK.reset(body_, waistLink);
                for(int i=0; i < 2; ++i){
                    waistFeetIK.addBaseLink(legged->footLink(i));
                }
            }
        }
    }
}    
    

void WaistBalancer::relaxWaistHeightTrajectory()
{
    for(int i = 0; i < numFilteredFrames; ++i){

        WaistFeetPos& p = waistFeetPosSeq[i];
        for(int j=0; j < 2; ++j){
            Link* footLink = waistFeetIK.baseLink(j);
            footLink->p() = p.p_Foot[j];
            footLink->R() = p.R_Foot[j];
        }

        Vector3 dp = totalCmTranslations[i + frameToStartBalancer];
        double& dz = dp.z();
        double hi = std::numeric_limits<double>::max();
        double low = -hi;
        const double step = 0.01;
        for(int j=0; j < 50; ++j){
            if((hi - low) < 1.0e-4){
                dz = low;
                break;
            }
            bool solved =
                waistFeetIK.calcInverseKinematics(p.p_Waist + dp, p.R_Waist) &&
                (rightKneePitchJoint->q() > 0.3 && leftKneePitchJoint->q() > 0.3);

            if(solved){
                if(dz == 0.0){
                    break;
                }
                low = dz;
                if(hi == std::numeric_limits<double>::max()){
                    dz = std::min(dz + step, 0.0);
                } else {
                    dz = (low + hi) / 2.0;
                }
            } else {
                hi = dz;
                if(low == -std::numeric_limits<double>::max()){
                    dz -= step;
                } else {
                    dz = (low + hi) / 2.0;
                }
            }
        }
        
        waistDeltaZseq[i] = dz;
    }

    Zelements zelems(totalCmTranslations, frameToStartBalancer);

    //applyGaussianFilter(zelems, waistDeltaZseq, 8.0, 40, 0.0);
    applyGaussianFilter(zelems, waistDeltaZseq, 5.0, 10, 0.0);
}


void WaistBalancer::applyCubicBoundarySmoother(int begin, int direction)
{
    double tf = numBoundarySmoothingFrames / frameRate;
    double tf2 = tf * tf;
    double a2 = 3.0 / tf2;
    double a3 = -2.0 / (tf * tf2);

    int frame = begin;
    if(isBoundaryCmAdjustmentEnabled){
        for(int i=0; i < numBoundarySmoothingFrames; ++i){
            double t = i * timeStep;
            double r = (a2 + (a3 * t)) * (t * t);
            totalCmTranslations[frame] =
                r * totalCmTranslations[frame] + (1.0 - r) * boundaryCmAdjustmentTranslations[frame];
            frame += direction;
        }
    } else {
        for(int i=0; i < numBoundarySmoothingFrames; ++i){
            double t = i * timeStep;
            double r = (a2 + (a3 * t)) * (t * t);
            totalCmTranslations[frame] = r * totalCmTranslations[frame];
            frame += direction;
        }
    }
}


void WaistBalancer::applyQuinticBoundarySmoother(int begin, int direction)
{
    double tf = numBoundarySmoothingFrames / frameRate;
    double tf2 = tf * tf;
    double tf3 = tf2 * tf;
    double tf4 = tf3 * tf;
    double tf5 = tf4 * tf;
    double a3 = 20.0 / (2.0 * tf3);
    double a4 = -30.0 / (2.0 * tf4);
    double a5 = 12.0 / (2.0 * tf5);

    int frame = begin;
    if(isBoundaryCmAdjustmentEnabled){
        for(int i=0; i < numBoundarySmoothingFrames; ++i){
            double t = i * timeStep;
            double r = (a3 + (a4 + a5 * t) * t) * (t * t * t);
            totalCmTranslations[frame] =
                r * totalCmTranslations[frame] + (1.0 - r) * boundaryCmAdjustmentTranslations[frame];
            frame += direction;
        }
    } else {
        for(int i=0; i < numBoundarySmoothingFrames; ++i){
            double t = i * timeStep;
            double r = (a3 + (a4 + a5 * t) * t) * (t * t * t);
            totalCmTranslations[frame] = r * totalCmTranslations[frame];
            frame += direction;
        }
    }        
}


bool WaistBalancer::applyCmTranslations(BodyMotion& motion, bool putAllLinkPositions)
{
    if(doBoundarySmoother){
        boundarySmootherFunction(frameToStartBalancer, 1);
        if(doProcessFinalBoundary){
            boundarySmootherFunction(endingFrame, -1);
        }
    }
    
    bool completed = true;

    const int numJoints = body_->numJoints();
    const int numLinksToPut = (putAllLinkPositions ? body_->numLinks() : 1);
    
    motion.setDimension(endingFrame + 1, numJoints, numLinksToPut, true);

    MultiValueSeq& qseq = *motion.jointPosSeq();
    MultiSE3Seq& pseq = *motion.linkPosSeq();
    auto zmpseq = getOrCreateZMPSeq(motion);
    zmpseq->setRootRelative(false);

    initBodyKinematics(beginningFrame, totalCmTranslations[beginningFrame]);

    for(int frame = beginningFrame; frame <= endingFrame; ++frame){

        completed &= updateBodyKinematics1(frame); 

        MultiValueSeq::Frame qs = qseq.frame(frame);
        for(int i=0; i < numJoints; ++i){
            qs[i] = body_->joint(i)->q();
        }

        zmpseq->at(frame) = zmp;
        
        for(int i=0; i < numLinksToPut; ++i){
            Link* link = body_->link(i);
            pseq.at(frame, i).set(link->T());
        }

        updateBodyKinematics2();
    }

    return completed;
}
