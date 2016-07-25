/**
   \file
   \author Shin'ichiro Nakaoka
*/
  
#include "JointPath.h"
#include "Jacobian.h"
#include "Body.h"
#include "BodyCustomizerInterface.h"
#include <cnoid/EigenUtil>
#include <cnoid/TruncatedSVD>

using namespace std;
using namespace std::placeholders;
using namespace cnoid;


double JointPath::numericalIKdefaultDeltaScale()
{
    return 0.9;
}

double JointPath::numericalIKdefaultTruncateRatio()
{
    return TruncatedSVD<MatrixXd>::defaultTruncateRatio();
}

int JointPath::numericalIKdefaultMaxIterations()
{
    return 50;
}

double JointPath::numericalIKdefaultMaxIKerror()
{
    return 1.0e-6;
}

double JointPath::numericalIKdefaultDampingConstant()
{
    return 1.0e-6;
}


namespace cnoid {

class JointPathIkImpl
{
public:
    bool isBestEffortIKmode;
    double deltaScale;
    int maxIterations;
    int iteration; 
    double maxIKerrorSqr;
    double dampingConstantSqr;
    MatrixXd J;
    VectorXd dTask;
    VectorXd dq;
    vector<double> q0;
    MatrixXd JJ;
    Eigen::ColPivHouseholderQR<MatrixXd> QR;
    TruncatedSVD<MatrixXd> svd;
    std::function<double(VectorXd& out_error)> errorFunc;
    std::function<void(MatrixXd& out_Jacobian)> jacobianFunc;

    JointPathIkImpl() {
        deltaScale = JointPath::numericalIKdefaultDeltaScale();
        maxIterations = JointPath::numericalIKdefaultMaxIterations();
        iteration = 0;
        dTask.resize(6);
        isBestEffortIKmode = false;
        double e = JointPath::numericalIKdefaultMaxIKerror();
        maxIKerrorSqr = e * e;
        double d = JointPath::numericalIKdefaultDampingConstant();
        dampingConstantSqr = d * d;
    }

    void resize(int numJoints){
        J.resize(dTask.size(), numJoints);
        dq.resize(numJoints);
    }
};
}


JointPath::JointPath()
{
    initialize();
}


JointPath::JointPath(Link* base, Link* end)
    : linkPath(base, end), 
      joints(linkPath.size())
{
    initialize();
    extractJoints();
}


JointPath::JointPath(Link* end)
    : linkPath(end), 
      joints(linkPath.size())
{
    initialize();
    extractJoints();
}


void JointPath::initialize()
{
    ik = 0;
    needForwardKinematicsBeforeIK = false;
}
	

JointPath::~JointPath()
{
    if(ik){
        delete ik;
    }
}


bool JointPath::setPath(Link* base, Link* end)
{
    if(linkPath.setPath(base, end)){
        extractJoints();
    }
    onJointPathUpdated();

    return (!joints.empty());
}


bool JointPath::setPath(Link* end)
{
    linkPath.setPath(end);
    extractJoints();
    onJointPathUpdated();
	
    return !joints.empty();
}


void JointPath::extractJoints()
{
    numUpwardJointConnections = 0;

    int n = linkPath.size();
    if(n <= 1){
        joints.clear();
    } else {
        int i = 0;
        if(linkPath.isDownward(i)){
            i++;
        }
        joints.resize(n); // reserve size n buffer
        joints.clear();
        int m = n - 1;
        while(i < m){
            Link* link = linkPath[i];
            if(link->jointId() >= 0){
                if(link->isRotationalJoint() || link->isSlideJoint()){
                    joints.push_back(link);
                    if(!linkPath.isDownward(i)){
                        numUpwardJointConnections++;
                    }
                }
            }
            ++i;
        }
        if(linkPath.isDownward(m-1)){
            Link* link = linkPath[m];
            if(link->jointId() >= 0){
                if(link->isRotationalJoint() || link->isSlideJoint()){
                    joints.push_back(link);
                }
            }
        }
    }
}


int JointPath::indexOf(const Link* link) const
{
    for(size_t i=0; i < joints.size(); ++i){
        if(joints[i] == link){
            return i;
        }
    }
    return -1;
}


void JointPath::onJointPathUpdated()
{
    if(ik){
        ik->errorFunc = nullptr;
        ik->jacobianFunc = nullptr;
    }
}


void JointPath::calcJacobian(Eigen::MatrixXd& out_J) const
{
    const int n = joints.size();
    out_J.resize(6, n);
	
    if(n > 0){

        //! \todo compare the efficiency for the following codes
        if(false){
            setJacobian<0x3f, 0, 0>(*this, linkPath.endLink(), out_J);

        } else {
        
            Link* targetLink = linkPath.endLink();
		
            for(int i=0; i < n; ++i){
			
                Link* link = joints[i];
			
                switch(link->jointType()){
				
                case Link::ROTATIONAL_JOINT:
                {
                    Vector3 omega = link->R() * link->a();
                    const Vector3 arm = targetLink->p() - link->p();
                    if(!isJointDownward(i)){
                        omega = -omega;
                    }
                    out_J.col(i) << omega.cross(arm), omega;
                }
                break;
				
                case Link::SLIDE_JOINT:
                {
                    Vector3 dp = link->R() * link->d();
                    if(!isJointDownward(i)){
                        dp = -dp;
                    }
                    out_J.col(i) << dp, Vector3::Zero();
                }
                break;
				
                default:
                    out_J.col(i).setZero();
                }
            }
        }
    }
}


inline JointPathIkImpl* JointPath::getOrCreateIK()
{
    if(!ik){
        ik = new JointPathIkImpl();
    }
    return ik;
}


bool JointPath::isBestEffortIKmode() const
{
    return ik ? ik->isBestEffortIKmode : false;
}


void JointPath::setBestEffortIKmode(bool on)
{
    getOrCreateIK()->isBestEffortIKmode = on;
}


void JointPath::setNumericalIKmaxIKerror(double e)
{
    getOrCreateIK()->maxIKerrorSqr = e * e;
}


void JointPath::setNumericalIKdeltaScale(double s)
{
    getOrCreateIK()->deltaScale = s;
}


void JointPath::setNumericalIKtruncateRatio(double r)
{
    getOrCreateIK()->svd.setTruncateRatio(r);
}

    
void JointPath::setNumericalIKmaxIterations(int n)
{
    getOrCreateIK()->maxIterations = n;
}


void JointPath::setNumericalIKdampingConstant(double lambda)
{
    getOrCreateIK()->dampingConstantSqr = lambda * lambda;
}


void JointPath::customizeTarget
(int numTargetElements,
 std::function<double(VectorXd& out_error)> errorFunc,
 std::function<void(MatrixXd& out_Jacobian)> jacobianFunc)
{
    if(!ik){
        ik = new JointPathIkImpl;
    }

    ik->dTask.resize(numTargetElements);
    ik->errorFunc = errorFunc;
    ik->jacobianFunc = jacobianFunc;
}


JointPath& JointPath::setGoal
(const Vector3& base_p, const Matrix3& base_R, const Vector3& end_p, const Matrix3& end_R)
{
    targetTranslationGoal = end_p;
    targetRotationGoal = end_R;
    
    Link* baseLink = linkPath.baseLink();
    baseLink->p() = base_p;
    baseLink->R() = base_R;

    needForwardKinematicsBeforeIK = true;

    return *this;
}


bool JointPath::calcInverseKinematics
(const Vector3& base_p, const Matrix3& base_R, const Vector3& end_p, const Matrix3& end_R)
{
    targetTranslationGoal = end_p;
    targetRotationGoal = end_R;
    
    Link* baseLink = linkPath.baseLink();
    baseLink->p() = base_p;
    baseLink->R() = base_R;

    if(hasAnalyticalIK()){
        return calcInverseKinematics(targetTranslationGoal, targetRotationGoal);
    } else {
        needForwardKinematicsBeforeIK = true;
        return calcInverseKinematics();
    }
}


bool JointPath::calcInverseKinematics(const Vector3& end_p, const Matrix3& end_R)
{
    targetTranslationGoal = end_p;
    targetRotationGoal = end_R;

    return calcInverseKinematics();
}


bool JointPath::calcInverseKinematics()
{
    const bool USE_USUAL_INVERSE_SOLUTION_FOR_6x6_NON_BEST_EFFORT_PROBLEM = false;
    const bool USE_SVD_FOR_BEST_EFFORT_IK = false;

    if(joints.empty()){
        if(linkPath.empty()){
            return false;
        }
        if(baseLink() == endLink()){
            baseLink()->p() = targetTranslationGoal;
            baseLink()->R() = targetRotationGoal;
            return true;
        } else {
            // \todo implement here
            return false;
        }
    }
    const int n = numJoints();
    
    if(!ik){
        ik = new JointPathIkImpl();
    }
    if(!ik->jacobianFunc){
        ik->jacobianFunc = std::bind(setJacobian<0x3f, 0, 0>, std::ref(*this), endLink(), _1);
    }
    ik->resize(n);
    
    Link* target = linkPath.endLink();

    if(needForwardKinematicsBeforeIK){
        calcForwardKinematics();
        needForwardKinematicsBeforeIK = false;
    }

    ik->q0.resize(n);
    if(!ik->isBestEffortIKmode){
        for(int i=0; i < n; ++i){
            ik->q0[i] = joints[i]->q();
        }
    }

    double prevErrsqr = std::numeric_limits<double>::max();
    bool completed = false;

    bool useUsualInverseSolution = false;
    if(USE_USUAL_INVERSE_SOLUTION_FOR_6x6_NON_BEST_EFFORT_PROBLEM){
        if(!ik->isBestEffortIKmode && (n == 6) && (ik->dTask.size() == 6)){
            useUsualInverseSolution = true;
        }
    }
        
    if(USE_SVD_FOR_BEST_EFFORT_IK && !useUsualInverseSolution && !ik->isBestEffortIKmode){
        // disable truncation
        ik->svd.setTruncateRatio(std::numeric_limits<double>::max());
    }

    for(ik->iteration = 0; ik->iteration < ik->maxIterations; ++ik->iteration){

        double errorSqr;
        if(ik->errorFunc){
            errorSqr = ik->errorFunc(ik->dTask);
        } else {
            ik->dTask.head<3>() = targetTranslationGoal - target->p();
            ik->dTask.segment<3>(3) = target->R() * omegaFromRot(target->R().transpose() * targetRotationGoal);
            errorSqr = ik->dTask.squaredNorm();
        }
        if(errorSqr < ik->maxIKerrorSqr){
            completed = true;
            break;
        }
        if(prevErrsqr - errorSqr < ik->maxIKerrorSqr){
            if(ik->isBestEffortIKmode && (errorSqr > prevErrsqr)){
                for(int j=0; j < n; ++j){
                    joints[j]->q() = ik->q0[j];
                }
                calcForwardKinematics();
            }
            break;
        }
        prevErrsqr = errorSqr;

        ik->jacobianFunc(ik->J);

        if(useUsualInverseSolution){
            ik->dq = ik->QR.compute(ik->J).solve(ik->dTask);
        } else {
            if(USE_SVD_FOR_BEST_EFFORT_IK){
                ik->svd.compute(ik->J).solve(ik->dTask, ik->dq);
            } else {
                // The damped least squares (singurality robust inverse) method
                ik->JJ = ik->J * ik->J.transpose() + ik->dampingConstantSqr * MatrixXd::Identity(ik->J.rows(), ik->J.rows());
                ik->dq = ik->J.transpose() * ik->QR.compute(ik->JJ).solve(ik->dTask);
            }
        }

        if(ik->isBestEffortIKmode){
            for(int j=0; j < n; ++j){
                double& q = joints[j]->q();
                ik->q0[j] = q;
                q += ik->deltaScale * ik->dq(j);
            }
        } else {
            for(int j=0; j < n; ++j){
                joints[j]->q() += ik->deltaScale * ik->dq(j);
            }
        }

        calcForwardKinematics();
    }

    if(!completed && !ik->isBestEffortIKmode){
        for(int i=0; i < n; ++i){
            joints[i]->q() = ik->q0[i];
        }
        calcForwardKinematics();
    }
    
    return completed;
}


int JointPath::numIterations() const
{
    return ik ? ik->iteration : 0;
}


bool JointPath::hasAnalyticalIK() const
{
    return false;
}


std::ostream& operator<<(std::ostream& os, JointPath& path)
{
    int n = path.numJoints();
    for(int i=0; i < n; ++i){
        Link* link = path.joint(i);
        os << link->name();
        if(i != n){
            os << (path.isJointDownward(i) ? " => " : " <= ");
        }
    }
    os << std::endl;
    return os;
}


namespace {

class CustomJointPath : public JointPath
{
    BodyPtr body;
    int ikTypeId;
    bool isCustomizedIkPathReversed;
    virtual void onJointPathUpdated();
public:
    CustomJointPath(const BodyPtr& body, Link* baseLink, Link* targetLink);
    virtual ~CustomJointPath();
    virtual bool calcInverseKinematics(const Vector3& end_p, const Matrix3& end_R);
    virtual bool hasAnalyticalIK() const;
};
}


CustomJointPath::CustomJointPath(const BodyPtr& body, Link* baseLink, Link* targetLink)
    : JointPath(baseLink, targetLink),
      body(body)
{
    onJointPathUpdated();
}


CustomJointPath::~CustomJointPath()
{

}


void CustomJointPath::onJointPathUpdated()
{
    ikTypeId = body->customizerInterface()->initializeAnalyticIk(
        body->customizerHandle(), baseLink()->index(), endLink()->index());
    if(ikTypeId){
        isCustomizedIkPathReversed = false;
    } else {
        // try reversed path
        ikTypeId = body->customizerInterface()->initializeAnalyticIk(
            body->customizerHandle(), endLink()->index(), baseLink()->index());
        if(ikTypeId){
            isCustomizedIkPathReversed = true;
        }
    }
}


bool CustomJointPath::calcInverseKinematics(const Vector3& end_p, const Matrix3& end_R)
{
    bool solved;
	
    if(ikTypeId == 0 || isBestEffortIKmode()){

        solved = JointPath::calcInverseKinematics(end_p, end_R);

    } else {

#if 0
        std::vector<double> qorg(numJoints());
        for(int i=0; i < numJoints(); ++i){
            qorg[i] = joint(i)->q();
        }
#endif

        const Link* targetLink = endLink();
        const Link* baseLink_ = baseLink();

        Vector3 p_relative;
        Matrix3 R_relative;
        if(!isCustomizedIkPathReversed){
            p_relative.noalias() = baseLink_->R().transpose() * (end_p - baseLink_->p());
            R_relative.noalias() = baseLink_->R().transpose() * end_R;
        } else {
            p_relative.noalias() = end_R.transpose() * (baseLink_->p() - end_p);
            R_relative.noalias() = end_R.transpose() * baseLink_->R();
        }
        solved = body->customizerInterface()->
            calcAnalyticIk(body->customizerHandle(), ikTypeId, p_relative, R_relative);

        if(solved){
            calcForwardKinematics();
#if 0
            double errsqr =
                (end_p - targetLink->p()).squaredNorm() +
                omegaFromRot(targetLink->R().transpose() * end_R).squaredNorm();

            if(errsqr < maxIKerrorSqr()){
                solved = true;
            } else {
                solved = false;
                for(int i=0; i < numJoints(); ++i){
                    joint(i)->q() = qorg[i];
                }
                calcForwardKinematics();
            }
#endif
        }
    }

    return solved;
}


bool CustomJointPath::hasAnalyticalIK() const
{
    return (ikTypeId != 0);
}


JointPathPtr cnoid::getCustomJointPath(Body* body, Link* baseLink, Link* targetLink)
{
    if(body->customizerInterface() && body->customizerInterface()->initializeAnalyticIk){
        return std::make_shared<CustomJointPath>(body, baseLink, targetLink);
    } else {
        return std::make_shared<JointPath>(baseLink, targetLink);
    }
}
