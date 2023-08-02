#include "YawMomentCompensationFilter.h"
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/LeggedBodyHelper>
#include <cnoid/BodyMotion>
#include <cnoid/ZMPSeq>
#include <cnoid/MessageOut>
#include <cnoid/stdx/clamp>
#include <fmt/format.h>
#include <vector>
#include "gettext.h"

#include <iostream>

using namespace std;
using namespace cnoid;
using fmt::format;

extern "C" int ql0001_(
    int* m, int* me, int* mmax, int* n, int* nmax, int* mnn,
    double* c, double* d, double* a, double* b, double* xl,
    double* xu, double* x, double* u, int* iout, int* ifail,
    int* iprint, double* war, int* lwar, int* iwar, int* liwar,
    double* eps1);

namespace {

constexpr bool DebugMode = false;

}

namespace cnoid {

class YawMomentCompensationFilter::Impl
{
public:
    BodyPtr body;
    LeggedBodyHelperPtr leggedBody;
    BodyMotion* bodyMotion;
    shared_ptr<BodyPositionSeq> positionSeq;
    shared_ptr<ZMPSeq> zmpSeq;
    Vector3 zmp;
    Vector3 cop;
    int currentFrameIndex;

    int numJoints;
    double ankleHeight;
    Link* leftFoot;
    Link* rightFoot;
    LinkTraverse leftFootTraverse;
    LinkTraverse rightFootTraverse;
    Link* supportFoot;
    LinkTraverse* supportFootTraverse;
    bool isBothLegsSupporting;

    double frictionCoefficient;
    double soleRadius;
    bool isDynamicNormalForceMode;
    double constantNormalForceFactor;
    bool isMomentOfBothFeetSupportingEnabled;

    // Coefficients of spring-mass-dumper-system
    double kp_recovery;
    double kd_recovery;
    double mass_recovery;

    // element: 0 - max yaw friction, 1 - compensated yaw moment, 2 - negative max yaw friction
    shared_ptr<MultiValueSeq> yawMomentSeqOutput;

    shared_ptr<Vector3Seq> copSeqOutput;

    struct JointWeight
    {
        int jointId;
        bool isEnabled;
        double weight;
        double vLimitRatio;

        JointWeight() : jointId(-1), isEnabled(false), weight(1.0), vLimitRatio(1.0) { }
    };

    vector<JointWeight> allJointWeights;
    vector<JointWeight> jointWeights;
    JointDisplacementRangeMode jointDisplacementRangeMode;
    bool isJointDisplacementRangeConstraintEnabled;
    int opposingAdjustmentJointId;
    int opposingAdjustmentReferenceJointId;
    vector<double> opposingAdjustmentReferenceJointDiffSeq;
    
    // coefficients of dynamic equation
    vector<double> Hz;
    double bz;

    // For all joints
    vector<double> dq_prev;

    // For target joints
    vector<double> theta_in;
    vector<double> prevTheta_in;
    vector<double> dTheta;
    vector<double> prev_dTheta;
    vector<double> theta_dif;
    vector<double> dTheta_recovery;
    vector<double> ddTheta_recovery;
    vector<double> q_next;

    int M;     // TOTAL NUMBER OF CONSTRAINTS
    int ME;    // NUMBER OF EQUALITY CONSTRAINTS
    int MMAX;  // ROW DIMENSION OF A. MMAX MUST BE AT LEAST ONE AND GREATER THAN M
    int N;     // NUMBER OF VARIABLES
    int NMAX;  // ROW DIMENSION OF C. NMAX MUST BE GREATER OR EQUAL TO N
    int MNN;   // MUST BE EQUAL TO M + N + N
    int LWAR;  // GRATER THAN 3*NMAX*NMAX/2 + 10*NMAX + 2*MMAX.
    int LIWAR; // AT LEAST N

    vector<double> A;    // (MMAX,NMAX) THE DATA MATRIX OF THE LINEAR CONSTRAINTS (Column Major)
    vector<double> B;    // (MMAX) THE CONSTANT DATA OF THE LINEAR CONSTRAINTS
    vector<double> C;    // (NMAX,NMAX) OBJECTIVE FUNCTION MATRIX (Column Major)
    vector<double> D;    // (NMAX) THE CONSTANT VECTOR OF THE OBJECTIVE FUNCTION
    vector<double> XL;   // (N) LOWER BOUNDS FOR THE VARIABLES
    vector<double> XU;   // (N) UPPER BOUNDS FOR THE VARIABLES
    vector<double> X;    // (N) THE OPTIMAL SOLUTION VECTOR (ON RETURN)
    vector<double> U;    // (MNN) THE LAGRANGE MULTIPLIERS (ON RETURN)
    vector<double> WAR;  // (LWAR) REAL WORKING ARRAY, LENGTH IS LWAR
    vector<int>    IWAR; // (LIWAR) INTEGER WORKING ARRAY, LENGTH IS LIWAR

    double dt;
    double dt2;
    double totalNormalForce;
    double minNormalForce;
    double maxYawFrictionMoment;

    int qpErrorId;

    MessageOut* mout;

    Impl();
    bool apply(Body* body, BodyMotion* bodyMotion);
    bool setBody(Body* orgBody);
    bool setBodyMotion(BodyMotion* bodyMotion);
    void compensateMoment();
    void detectSupportFootChange();
    bool checkIfFootTouchingToFloor(Link* foot);
    void setSupportFoot(Link* foot);
    void calcTotalNormalForceOfFloor();
    void updateCop();
    void calcCoefficientsOfDynamicEquation();
    double calcYawMomentAroundCop();
    double calcResultingYawMomentAroundCop();
    void calcMaxYawFrictionMoment();
    bool calcCompensationByQuadraticProgramming();
    std::pair<double, double> getInputMotionJointDisplacementRange(Link* joint, double q_input);
    bool checkJointDisplacementRangeOver();
    const char* lastQpErrorMessage();
    void adjustOpposingAdjustmentJointTrajectory();
    void putVector(const char* name, vector<double>& a);
    void putMatrix(const char* name, vector<double>& a, int row, int col, int rowMax);
};

}


double YawMomentCompensationFilter::defaultFrictionCoefficient()
{
    return 0.5;
}

double YawMomentCompensationFilter::defaultSoleRadius()
{
    return 0.05;
}

double YawMomentCompensationFilter::defaultConstantNormalForceFactor()
{
    return 0.8;
}

double YawMomentCompensationFilter::defaultRecoveryForcePGain()
{
    return 10.0;
}

double YawMomentCompensationFilter::defaultRecoveryForceDGain()
{
    return 6.0;
}


YawMomentCompensationFilter::YawMomentCompensationFilter()
{
    impl = new Impl;
}


YawMomentCompensationFilter::Impl::Impl()
{
    frictionCoefficient = defaultFrictionCoefficient();
    soleRadius = defaultSoleRadius();
    isDynamicNormalForceMode = true;
    constantNormalForceFactor = defaultConstantNormalForceFactor();
    isMomentOfBothFeetSupportingEnabled = true;
    kp_recovery = defaultRecoveryForcePGain();
    kd_recovery = defaultRecoveryForceDGain();
    mass_recovery = 1.0;
    jointDisplacementRangeMode = InputMotionJointDisplacementRange;
    isJointDisplacementRangeConstraintEnabled = true;
    opposingAdjustmentJointId = -1;
    opposingAdjustmentReferenceJointId = -1;

    mout = MessageOut::master();
}


YawMomentCompensationFilter::~YawMomentCompensationFilter()
{
    delete impl;
}


void YawMomentCompensationFilter::setMessageSink(MessageOut* mout)
{
    impl->mout = mout;
}


void YawMomentCompensationFilter::clearJointWeights()
{
    impl->allJointWeights.clear();
}


void YawMomentCompensationFilter::setJointWeight(int jointId, bool isEnabled, double weight, double vLimitRatio)
{
    if(DebugMode){
        cout << format("setJointWeight({0}, {1}, {2}, {3})", jointId, isEnabled, weight, vLimitRatio) << endl;
    }
        
    if(jointId >= impl->allJointWeights.size()){
        impl->allJointWeights.resize(jointId + 1);
    }
    auto& w = impl->allJointWeights[jointId];
    w.jointId = jointId;
    w.isEnabled = isEnabled;
    w.weight = weight;
    w.vLimitRatio = vLimitRatio;
}


void YawMomentCompensationFilter::setRecoveryForceGains(double kp, double kd)
{
    impl->kp_recovery = kp;
    impl->kd_recovery = kd;
}


void YawMomentCompensationFilter::setRecoveryCoefficients(double omega, double zita)
{
    impl->kp_recovery = omega * omega;
    impl->kd_recovery = 2.0 * zita * omega;
}


void YawMomentCompensationFilter::setJointDisplacementRangeMode(JointDisplacementRangeMode mode)
{
    impl->jointDisplacementRangeMode = mode;
}


void YawMomentCompensationFilter::setJointDisplacementRangeConstraintEnabled(bool on)
{
    impl->isJointDisplacementRangeConstraintEnabled = on;
}


void YawMomentCompensationFilter::setOpposingAdjustmentJoint(int adjustmentJointId, int referenceJointId)
{
    impl->opposingAdjustmentJointId = adjustmentJointId;
    if(impl->opposingAdjustmentJointId >= 0){
        impl->opposingAdjustmentReferenceJointId = referenceJointId;
    } else {
        impl->opposingAdjustmentReferenceJointId = -1;
    }
}


void YawMomentCompensationFilter::resetOpposingAdjustmentJoint()
{
    impl->opposingAdjustmentJointId = -1;
    impl->opposingAdjustmentReferenceJointId = -1;
}


void YawMomentCompensationFilter::setFrictionCoefficient(double c)
{
    impl->frictionCoefficient = c;
}


void YawMomentCompensationFilter::setSoleRadius(double r)
{
    impl->soleRadius = r;
}


void YawMomentCompensationFilter::setDynamicNormalForceMode(bool on)
{
    impl->isDynamicNormalForceMode = on;
}


void YawMomentCompensationFilter::setConstantNormalForceFactor(double factor)
{
    impl->constantNormalForceFactor = factor;
}


void YawMomentCompensationFilter::enableMomentOfBothFeetSupporting(bool on)
{
    impl->isMomentOfBothFeetSupportingEnabled = on;
}


void YawMomentCompensationFilter::setYawMomentSeqOutput(shared_ptr<MultiValueSeq> yawMomentSeq)
{
    impl->yawMomentSeqOutput = yawMomentSeq;
}


void YawMomentCompensationFilter::setCopSeqOutput(shared_ptr<Vector3Seq> copSeq)
{
    impl->copSeqOutput = copSeq;
}


bool YawMomentCompensationFilter::apply(Body* body, BodyMotion& bodyMotion)
{
    return impl->apply(body, &bodyMotion);
}


bool YawMomentCompensationFilter::Impl::apply(Body* body, BodyMotion* bodyMotion)
{
    if(!setBody(body)){
        return false;
    }

    if(!setBodyMotion(bodyMotion)){
        return false;
    }

    int numFrames = bodyMotion->numFrames();
    for(currentFrameIndex = 0; currentFrameIndex < numFrames; ++currentFrameIndex){
        compensateMoment();
    }

    if(opposingAdjustmentJointId >= 0){
        adjustOpposingAdjustmentJointTrajectory();
    }

    return true;
}


bool YawMomentCompensationFilter::Impl::setBody(Body* orgBody)
{
    body = orgBody->clone();
    numJoints = body->numJoints();

    jointWeights.clear();
    for(size_t i=0; i < allJointWeights.size(); ++i){
        JointWeight& w = allJointWeights[i];
        if(w.isEnabled){
            jointWeights.push_back(w);
        }
    }
    allJointWeights.resize(numJoints);

    leggedBody = new LeggedBodyHelper(body);
    if(!leggedBody->isValid() || leggedBody->numFeet() != 2){
        mout->putErrorln(format(_("{0} is not a biped robot."), body->name()));
        return false;
    }

    ankleHeight = -leggedBody->homeCopOfSoleLocal(0).z();

    leftFoot = leggedBody->footLink(LeggedBodyHelper::Left);
    leftFootTraverse.find(leftFoot, true, true);
    rightFoot = leggedBody->footLink(LeggedBodyHelper::Right);
    rightFootTraverse.find(rightFoot, true, true);
    supportFoot = nullptr;
    supportFootTraverse = nullptr;

    dq_prev.resize(numJoints);

    M = 2; // Yaw moment inequality constraints (max/min yaw moments)
    ME = 0;
    MMAX = M + 1;
    N = jointWeights.size();
    NMAX = N;
    MNN = M + N + N;
    LWAR = 3 * NMAX * NMAX / 2 + 10 * NMAX + 2 * MMAX + 1;
    LIWAR = N;

    A.resize(MMAX * NMAX);
    B.resize(MMAX);
    C.resize(NMAX * NMAX);
    D.resize(NMAX);
    XL.resize(N);
    XU.resize(N);
    X.resize(N);
    U.resize(MNN);
    WAR.resize(LWAR);
    IWAR.resize(LIWAR);

    for(int i=0; i < MMAX; ++i){
        for(int j=0; j < NMAX; ++j){
            A[i  + j * MMAX] = 0.0;
            
        }
    }
    B[M] = 0.0;
    for(int i=0; i < NMAX; ++i){
        for(int j=N; j < NMAX; ++j){
            C[i + j * NMAX] = 0.0;
        }
        D[i] = 0.0;
    }
    
    // make a matrix and constant vector of the objective function
    for(int i=0; i < N; i++){
        for(int j=0; j < N; j++){
            if(i == j){
                C[i + j * NMAX] = jointWeights[i].weight;
            } else {
                C[i + j * NMAX] = 0.0;
            }
        }
    }

    Hz.resize(N);
    theta_in.resize(N);
    prevTheta_in.resize(N);
    dTheta.resize(N);
    prev_dTheta.resize(N);
    theta_dif.resize(N);
    dTheta_recovery.resize(N);
    ddTheta_recovery.resize(N);
    q_next.resize(N);

    double mass = body->mass();
    totalNormalForce = constantNormalForceFactor * mass * 9.8;
    minNormalForce = 0.4 * mass * 9.8;

    isBothLegsSupporting = false;

    return true;
}


bool YawMomentCompensationFilter::Impl::setBodyMotion(BodyMotion* bodyMotion)
{
    this->bodyMotion = bodyMotion;
    int numFrames = bodyMotion->numFrames();
    if(numFrames < 2){
        mout->putErrorln(_("The input motion data does not have a valid number of time frames."));
        return false;
    }

    positionSeq = bodyMotion->positionSeq();

    if(positionSeq->numLinkPositionsHint() != 1){
        mout->putErrorln(
            format(_("The input motion data must contain only the position sequence of the root link as "
                     "link position sequences, but it contain position sequenss for {0} links."),
                   positionSeq->numLinkPositionsHint()));
        return false;
    }
    if(positionSeq->numJointDisplacementsHint() != body->numJoints()){
        mout->putErrorln(
            _("The number of joint displacement sequences does not match "
              "the number of joints in the target body."));
        return false;
    }

    // ZMP seq should be desired one in the global coordinate system
    zmpSeq = getZMPSeq(*bodyMotion);
    if(!zmpSeq){
        mout->putErrorln(_("The input motion data does not contain the ZMP sequence."));
        return false;
    }

    currentFrameIndex = 0;
    *body << positionSeq->frame(currentFrameIndex);
    body->calcForwardKinematics();

    for(int i=0; i < N; i++){
        double q = body->joint(jointWeights[i].jointId)->q();
        theta_in[i] = q;
        prevTheta_in[i] = q;
        dTheta[i] = 0.0;
        prev_dTheta[i] = 0.0;
        theta_dif[i] = 0.0;
        dTheta_recovery[i] = 0.0;
        ddTheta_recovery[i] = 0.0;
    }

    for(int i=0; i < numJoints; ++i){
        auto joint = body->joint(i);
        joint->dq() = 0.0;
        joint->ddq() = 0.0;
        dq_prev[i] = 0.0;
    }

    setSupportFoot(leftFoot);
    detectSupportFootChange();
    updateCop();

    dt = bodyMotion->timeStep();
    dt2 = dt * dt;

    if(yawMomentSeqOutput){
        yawMomentSeqOutput->clear();
        yawMomentSeqOutput->setFrameRate(bodyMotion->frameRate());
        yawMomentSeqOutput->setNumParts(3);
    }
    if(copSeqOutput){
        copSeqOutput->clear();
        copSeqOutput->setFrameRate(bodyMotion->frameRate());
    }

    if(opposingAdjustmentJointId >= 0){
        opposingAdjustmentReferenceJointDiffSeq.resize(numFrames);
    }
    
    return true;
}


void YawMomentCompensationFilter::Impl::compensateMoment()
{
    if(isDynamicNormalForceMode){
        calcTotalNormalForceOfFloor();
    }

    calcCoefficientsOfDynamicEquation();
    calcMaxYawFrictionMoment();
    
    auto& currentFrame = positionSeq->frame(currentFrameIndex);

    for(int i=0; i < N; i++){
        theta_in[i] = currentFrame.jointDisplacement(jointWeights[i].jointId);
    }

    bool resolved = true;
    if(N > 0){
        resolved = calcCompensationByQuadraticProgramming();
    }

    double yawMoment;
    
    if(!resolved || yawMomentSeqOutput){
        if(N > 0){
            yawMoment = calcResultingYawMomentAroundCop();
        } else {
            yawMoment = bz;
        }
    }

    double maxYawFrictionMoment0 = maxYawFrictionMoment;
    bool doPutNotification = false;
    if(!resolved){
        double o = abs(yawMoment) - maxYawFrictionMoment0;
        for(int i=1; i <= 10; ++i){
            maxYawFrictionMoment = maxYawFrictionMoment0 + o * i / 10.0;
            resolved = calcCompensationByQuadraticProgramming();
            yawMoment = calcResultingYawMomentAroundCop();
            if(resolved){
                break;
            }
        }
        doPutNotification = true;
    }
    
    if(doPutNotification){
        mout->putWarningln(
            format(_("Yaw moment compensation failed at time {0:.3f}: Yaw moment is {1:.3f}, Max yaw friction moment is {2:.3f}"),
                   currentFrameIndex * dt, yawMoment, maxYawFrictionMoment0));
        if(!resolved){
            mout->putWarningln(format(_("QP cannot be solved: {0}."), lastQpErrorMessage()));
        }
    }

    if(yawMomentSeqOutput){
        auto frame = yawMomentSeqOutput->appendFrame();
        frame[0] = maxYawFrictionMoment0;
        frame[1] = yawMoment;
        frame[2] = -maxYawFrictionMoment0;
    }

    for(int i=0; i < N; ++i){
        int jointId = jointWeights[i].jointId;
        auto joint = body->joint(jointId);
        double dq = (q_next[i] - joint->q()) / dt;
        joint->q() = q_next[i];
        dq_prev[jointId] = dq;
        prev_dTheta[i] = dTheta[i];
        prevTheta_in[i] = theta_in[i];
        theta_dif[i] = joint->q() - theta_in[i];
        dTheta_recovery[i] += ddTheta_recovery[i] * dt;

        if(jointId == opposingAdjustmentReferenceJointId){
            opposingAdjustmentReferenceJointDiffSeq[currentFrameIndex] = theta_dif[i];
        }
    }

    checkJointDisplacementRangeOver();
    
    for(int i = 0; i < numJoints; ++i){
        if(!allJointWeights[i].isEnabled){
            body->joint(i)->q() = currentFrame.jointDisplacement(i);
        }
    }
    body->rootLink()->setPosition(currentFrame.linkPosition(0).T());
    body->calcForwardKinematics();

    detectSupportFootChange();
    updateCop();
    if(copSeqOutput){
        copSeqOutput->append(cop);
    }

    for(int i=0; i < N; ++i){
        int jointId = jointWeights[i].jointId;
        currentFrame.jointDisplacement(jointId) = body->joint(jointId)->q();
    }
}


void YawMomentCompensationFilter::Impl::detectSupportFootChange()
{
    zmp = zmpSeq->at(currentFrameIndex);
    
    bool isLeftTouching = checkIfFootTouchingToFloor(leftFoot);
    bool isRightTouching = checkIfFootTouchingToFloor(rightFoot);

    double r; // 0.0: left foot to 1.0: right foot
    
    if(isLeftTouching){
        if(isRightTouching){
            Vector3 leftProjection = leggedBody->homeCopOfSole(0);
            leftProjection.z() = 0.0;
            Vector3 rightProjection = leggedBody->homeCopOfSole(1);
            leftProjection.z() = 0.0;
            Vector3 ltor = rightProjection - leftProjection;
            if(!ltor.isZero()){
                Vector3 ltozmp = zmp - leftProjection;
                r = ltozmp.dot(ltor.normalized()) / ltor.norm();
            } else {
                r = 0.5;
            }
        } else {
            r = 0.0;
        }
    } else {
        if(isRightTouching){
            r = 1.0;
        } else {
            r = 0.5;
        }
    }

    if(r > 0.1 && r < 0.9){
        isBothLegsSupporting = true;
    } else {
        isBothLegsSupporting = false;
        if(r <= 0.1){ // left support
            if(supportFoot != leftFoot){
                setSupportFoot(leftFoot);
            }
        } else if(r >= 0.9){ // right support
            if(supportFoot != rightFoot){
                setSupportFoot(rightFoot);
            }
        }
    }

    supportFoot->v().setZero();
    supportFoot->w().setZero();
    supportFoot->dv().setZero();
    supportFoot->dw().setZero();
}


bool YawMomentCompensationFilter::Impl::checkIfFootTouchingToFloor(Link* foot)
{
    if(foot->p().z() <= (ankleHeight + 1.0e-7)){
        Vector3 rpy = rpyFromRot(foot->R());
        rpy[2] = 0.0;
        if(rpy.isZero(1.0e-3)){
            return true;
        }
    }
    return false;
}


void YawMomentCompensationFilter::Impl::setSupportFoot(Link* foot)
{
    supportFoot = foot;
    if(foot == leftFoot){
        supportFootTraverse = &leftFootTraverse;
    } else {
        supportFootTraverse = &rightFootTraverse;
    }
}


void YawMomentCompensationFilter::Impl::calcTotalNormalForceOfFloor()
{
    // Assume that the body has the previous (filtered) joint angles, velocities and accelerations

    supportFoot->dv() << 0.0, 0.0, 9.8; // gravity
    
    supportFootTraverse->calcForwardKinematics(true, true);
    Vector6 F = supportFootTraverse->calcInverseDynamics();
    
    supportFoot->dv().setZero();

    totalNormalForce = std::max(F.head<3>().z(), minNormalForce);
}


// This function update COP, which is ZMP within the supporting area
void YawMomentCompensationFilter::Impl::updateCop()
{
    Vector3 c;
    double r;

    if(isBothLegsSupporting){
        Vector3 p0 = leggedBody->homeCopOfSole(0);
        Vector3 p1 = leggedBody->homeCopOfSole(1);
        c = (p0 + p1) / 2.0;
        c.z() = 0.0;
        r = (p0 - c).norm() + soleRadius;
    } else {
        int footIndex = supportFoot == leftFoot ? LeggedBodyHelper::Left : LeggedBodyHelper::Right;
        c = leggedBody->homeCopOfSole(footIndex);
        c.z() = 0.0;
        r = soleRadius;
    }

    Vector3 czmp = zmp - c;
    double l = czmp.norm();
    if(l > r){
        cop = (r / l) * czmp + c;
    } else {
        cop = zmp;
    }
}


// Assume that the body has the previous (filtered) joint angles.
void YawMomentCompensationFilter::Impl::calcCoefficientsOfDynamicEquation()
{
    auto& currentFrame = positionSeq->frame(currentFrameIndex);
    auto& prevFrame = positionSeq->frame(currentFrameIndex == 0 ? 0 : currentFrameIndex - 1);

    for(int i=0; i < N; ++i){
        for(int j=0; j < numJoints; ++j){
            auto joint = body->joint(j);
            joint->dq() = 0.0;
            if(j == jointWeights[i].jointId){
                joint->ddq() = 1.0;
            } else {
                joint->ddq() = 0.0;
            }
        }
        Hz[i] = calcYawMomentAroundCop();
    }

    for(int j=0; j < numJoints; ++j){
        auto joint = body->joint(j);
        joint->dq() = dq_prev[j];
        if(allJointWeights[j].isEnabled){
            joint->ddq() = 0.0;
        } else {
            double dq = (currentFrame.jointDisplacement(j) - prevFrame.jointDisplacement(j)) / dt;
            joint->ddq() = (dq - dq_prev[j]) / dt;
            dq_prev[j] = dq;
        }
    }
    bz = calcYawMomentAroundCop();
}


double YawMomentCompensationFilter::Impl::calcYawMomentAroundCop()
{
    supportFootTraverse->calcForwardKinematics(true, true);
    Vector6 F = supportFootTraverse->calcInverseDynamics();
    auto f = F.head<3>();
    auto tau = F.tail<3>();
    Vector3 tau_cop = -cop.cross(f) + tau;
    return tau_cop.z();
}


// Assume that the body has the last states calculated by calcCoefficientsOfDynamicEquation.
double YawMomentCompensationFilter::Impl::calcResultingYawMomentAroundCop()
{
    for(int i=0; i < N; ++i){
        int jointId = jointWeights[i].jointId;
        auto joint = body->joint(jointId);
        double dq = (q_next[i] - joint->q()) / dt;
        joint->ddq() = (dq - dq_prev[jointId]) / dt;
    }
    return calcYawMomentAroundCop();
}


// Assume that the feet have the previous frame position
void YawMomentCompensationFilter::Impl::calcMaxYawFrictionMoment()
{
    bool useBothLegSupportFriction = isBothLegsSupporting && isMomentOfBothFeetSupportingEnabled;

    // max rotation friction moment of disc of radius r (axis is center of disc,
    // pressure is even) is (2 / 3) * r * u * F
    maxYawFrictionMoment = 0.6666 * soleRadius * frictionCoefficient * totalNormalForce;

    if(useBothLegSupportFriction){
        Vector3 lpos = leftFoot->p();
        lpos.z() = 0.0;
        Vector3 rpos = rightFoot->p();
        rpos.z() = 0.0;

        Vector3 lr = (rpos - lpos);
        Vector3 e = lr.normalized();

        double k = e.dot(cop - lpos);
        double l = lr.norm();

        double copRatio = k / l;
        if(copRatio < 0.0){
            copRatio = 0.0;
        } else if(copRatio > 1.0){
            copRatio = 1.0;
        }

        double rSoleArm = (1.0 - copRatio) * l;
        double rFrictionMoment = copRatio * rSoleArm * frictionCoefficient * totalNormalForce;
        double lSoleArm = copRatio * l;
        double lFrictionMoment = (1.0 - copRatio) * lSoleArm * frictionCoefficient * totalNormalForce;
        
        maxYawFrictionMoment += (rFrictionMoment + lFrictionMoment);
    }
}


bool YawMomentCompensationFilter::Impl::calcCompensationByQuadraticProgramming()
{
    double cc = bz; // constant term of constraint

    for(int i=0; i < N; ++i){
        double dTheta_cmd = theta_in[i] - prevTheta_in[i];
        double f_recovery = -(kp_recovery * theta_dif[i] + kd_recovery * dTheta_recovery[i]);
        ddTheta_recovery[i] = f_recovery / mass_recovery;
        double dTheta_ret = (dTheta_recovery[i] + ddTheta_recovery[i] * dt) * dt;
        double dTheta_cmd_ret = dTheta_cmd + dTheta_ret;

        double a = Hz[i];
        A[i * MMAX + 0] = -a;
        A[i * MMAX + 1] = +a;

        cc += a * (dTheta_cmd_ret - prev_dTheta[i]) / dt2;
        
        JointWeight& w = jointWeights[i];
        int jointId = w.jointId;
        auto joint = body->joint(jointId);

        if(!isJointDisplacementRangeConstraintEnabled){
            XL[i] = -1.0e10;
            XU[i] =  1.0e10;

        } else if(jointDisplacementRangeMode == InputMotionJointDisplacementRange){
            auto range = getInputMotionJointDisplacementRange(joint, theta_in[i]);
            XL[i] = (range.first - joint->q() - dTheta_cmd_ret) / dt2;
            XU[i] = (range.second - joint->q() - dTheta_cmd_ret) / dt2;

        } else if(jointDisplacementRangeMode == ModelJointDisplacementRange){
            XL[i] = (joint->q_lower() - joint->q() - dTheta_cmd_ret) / dt2;
            XU[i] = (joint->q_upper() - joint->q() - dTheta_cmd_ret) / dt2;
        }

        dTheta[i] = dTheta_cmd_ret; // solution value is added later
    }

    B[0] = -cc + maxYawFrictionMoment;
    B[1] = +cc + maxYawFrictionMoment;

    double eps = 1.0e-16;

    if(DebugMode){
        //cout << "frame: " << currentFrameIndex << "\n";
        cout << "time: " << (currentFrameIndex * dt)<< "\n";
        putMatrix("A", A, M, N, MMAX);
        putVector("B", B);
        putMatrix("C", C, N, N, NMAX);
        //putVector("D", D);
        //putVector("XL", XL);
        //putVector("XU", XU);
    }

    IWAR[0] = 1;
    ql0001_(&M, &ME, &MMAX, &N, &NMAX, &MNN, &C[0], &D[0], &A[0], &B[0], &XL[0], &XU[0],
            &X[0], &U[0], 0, &qpErrorId, 0, &WAR[0], &LWAR, &IWAR[0], &LIWAR, &eps);

    bool resolved = (qpErrorId == 0);

    if(resolved){
        if(DebugMode){
            putVector("X", X);
            cout << endl;
        }
    }

    for(int i=0; i < N; ++i){
        int jointId = jointWeights[i].jointId;
        auto joint = body->joint(jointId);
        if(resolved){
            dTheta[i] += X[i] * dt2;
        }
        q_next[i] = joint->q() + dTheta[i];
    }
    
    return resolved;
}


std::pair<double, double> YawMomentCompensationFilter::Impl::getInputMotionJointDisplacementRange(Link* joint, double q_input)
{
    double q_lower = joint->q_lower();
    double q_upper = joint->q_upper();
    if(q_input < q_lower){
        q_lower = q_input;
    } else if(q_input > q_upper){
        q_upper = q_input;
    }
    return std::pair<double, double>(q_lower, q_upper);
}


bool YawMomentCompensationFilter::Impl::checkJointDisplacementRangeOver()
{
    bool hasLimitOverJoint = false;
    
    for(int i=0; i < N; ++i){
        bool isOver = false;
        int jointId = jointWeights[i].jointId;
        auto joint = body->joint(jointId);

        if(jointDisplacementRangeMode == InputMotionJointDisplacementRange){
            auto range = getInputMotionJointDisplacementRange(joint, theta_in[i]);
            if(joint->q() < range.first || joint->q() > range.second){
                isOver = true;
            }
        } else if(jointDisplacementRangeMode == ModelJointDisplacementRange){
            if(joint->q() < joint->q_lower() || joint->q() > joint->q_upper()){
                isOver = true;
            }
        }
        if(isOver){
            hasLimitOverJoint = true;
            
            if(joint->isRevoluteJoint()){
                mout->putWarningln(
                    format(_("Joint angle {0:.1f} [deg] of {1} is over the limit at time {2:.3f}."),
                           degree(joint->q()), joint->jointName(), currentFrameIndex * dt));
            } else {
                mout->putWarningln(
                    format(_("Joint displacement {0:.3f} [m] of {1} is over the limit at time {2:.3f}."),
                           joint->q(), joint->jointName(), currentFrameIndex * dt));
            }
        }
    }

    return hasLimitOverJoint;
}


const char* YawMomentCompensationFilter::Impl::lastQpErrorMessage()
{
    const char* message = "unknown cause";
    switch(qpErrorId){
    case 1: message = "too many iterations"; break;
    case 2: message = "accuracy insufficient to satisfy convergence criterion"; break;
    case 5: message = "length of a working array is too short"; break;
    default:
        if(qpErrorId > 10){
            message = "the constraints are inconsistent";
        } 
    }
    return message;
}


void YawMomentCompensationFilter::Impl::adjustOpposingAdjustmentJointTrajectory()
{
    auto joint = body->joint(opposingAdjustmentJointId);
    int numFrames = positionSeq->numFrames();
    for(int i=0; i < numFrames; ++i){
        auto& currentFrame = positionSeq->frame(i);
        auto& q = currentFrame.jointDisplacement(opposingAdjustmentJointId);
        auto q_adjusted = q - opposingAdjustmentReferenceJointDiffSeq[i];
        if((q_adjusted <= joint->q_upper() || q_adjusted <= q) &&
           (q_adjusted >= joint->q_lower() || q_adjusted >= q)){
            q = q_adjusted;
        }
    }
}


void YawMomentCompensationFilter::Impl::putVector(const char* name, vector<double>& a)
{
    cout << name << ": ";
    int n = a.size() - 1;
    for(size_t i=0; i < n; ++i){
        cout << a[i] << ", ";
    }
    cout << a.back() << "\n";
}


void YawMomentCompensationFilter::Impl::putMatrix(const char* name, vector<double>& a, int row, int col, int rowMax)
{
    cout << name << ":\n";
    for(int i=0; i < row; ++i){
        for(int j=0; j < col - 1; ++j){
            cout << a[i + j * rowMax] << ", ";
        }
        cout << a[i + (col - 1) * rowMax] << "\n";
    }
}
