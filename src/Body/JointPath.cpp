/**
   \file
   \author Shin'ichiro Nakaoka
*/
  
#include "JointPath.h"
#include "Jacobian.h"
#include "Body.h"
#include "CustomJointPathHandler.h"
#include "BodyCustomizerInterface.h"
#include <cnoid/EigenUtil>
#include <cnoid/TruncatedSVD>

using namespace std;
using namespace cnoid;


double JointPath::numericalIkDefaultDeltaScale()
{
    return 0.9;
}

int JointPath::numericalIkDefaultMaxIterations()
{
    return 50;
}

double JointPath::numericalIkDefaultMaxIkError()
{
    return 1.0e-7;
}

double JointPath::numericalIkDefaultDampingConstant()
{
    return 1.0e-6;
}

//! \deprecated
double JointPath::numericalIkDefaultTruncateRatio()
{
    return TruncatedSVD<MatrixXd>::defaultTruncateRatio();
}

namespace cnoid {

class NumericalIK
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    bool isBestEffortIkMode;
    double deltaScale;
    int maxIterations;
    int iteration; 
    double maxIkErrorSqr;
    double dampingConstantSqr;
    vector<double> q0;
    Isometry3 T0;
    MatrixXd J;
    VectorXd dTask;
    VectorXd dq;
    MatrixXd JJ;
    Eigen::ColPivHouseholderQR<MatrixXd> QR;
    TruncatedSVD<MatrixXd> svd;
    std::function<double(VectorXd& out_error)> errorFunc;
    std::function<void(MatrixXd& out_Jacobian)> jacobianFunc;

    NumericalIK() {
        deltaScale = JointPath::numericalIkDefaultDeltaScale();
        maxIterations = JointPath::numericalIkDefaultMaxIterations();
        iteration = 0;
        dTask.resize(6);
        isBestEffortIkMode = false;
        double e = JointPath::numericalIkDefaultMaxIkError();
        maxIkErrorSqr = e * e;
        double d = JointPath::numericalIkDefaultDampingConstant();
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
    : linkPath_(base, end), 
      joints_(linkPath_.size())
{
    initialize();
    extractJoints();
}


JointPath::JointPath(Link* end)
    : linkPath_(end), 
      joints_(linkPath_.size())
{
    initialize();
    extractJoints();
}


void JointPath::initialize()
{
    needForwardKinematicsBeforeIK = false;
    numericalIK = nullptr;
    isCustomIkDisabled_ = false;
}    


void JointPath::extractJoints()
{
    numUpwardJointConnections = 0;

    int n = linkPath_.size();
    if(n <= 1){
        joints_.clear();
    } else {
        int i = 0;
        if(linkPath_.isDownward(i)){
            i++;
        }
        joints_.resize(n); // reserve size n buffer
        joints_.clear();
        int m = n - 1;
        while(i < m){
            Link* link = linkPath_[i];
            if(link->jointId() >= 0){
                if(link->isRotationalJoint() || link->isSlideJoint()){
                    joints_.push_back(link);
                    if(!linkPath_.isDownward(i)){
                        numUpwardJointConnections++;
                    }
                }
            }
            ++i;
        }
        if(linkPath_.isDownward(m-1)){
            Link* link = linkPath_[m];
            if(link->jointId() >= 0){
                if(link->isRotationalJoint() || link->isSlideJoint()){
                    joints_.push_back(link);
                }
            }
        }
    }
}


int JointPath::indexOf(const Link* link) const
{
    for(size_t i=0; i < joints_.size(); ++i){
        if(joints_[i] == link){
            return i;
        }
    }
    return -1;
}


void JointPath::calcJacobian(Eigen::MatrixXd& out_J) const
{
    const int n = joints_.size();
    out_J.resize(6, n);
	
    if(n > 0){

        //! \todo compare the efficiency for the following codes
        if(false){
            setJacobian<0x3f, 0, 0>(*this, linkPath_.endLink(), out_J);

        } else {
        
            Link* targetLink = linkPath_.endLink();
		
            for(int i=0; i < n; ++i){
			
                Link* link = joints_[i];
			
                switch(link->jointType()){
				
                case Link::REVOLUTE_JOINT:
                {
                    Vector3 omega = link->R() * link->a();
                    const Vector3 arm = targetLink->p() - link->p();
                    if(!isJointDownward(i)){
                        omega = -omega;
                    }
                    out_J.col(i) << omega.cross(arm), omega;
                }
                break;
				
                case Link::PRISMATIC_JOINT:
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


NumericalIK* JointPath::getOrCreateNumericalIK()
{
    if(!numericalIK){
        numericalIK = new NumericalIK;
    }
    return numericalIK;
}


bool JointPath::isBestEffortIkMode() const
{
    return numericalIK ? numericalIK->isBestEffortIkMode : false;
}


void JointPath::setBestEffortIkMode(bool on)
{
    getOrCreateNumericalIK()->isBestEffortIkMode = on;
}


void JointPath::setNumericalIkMaxIkError(double e)
{
    getOrCreateNumericalIK()->maxIkErrorSqr = e * e;
}


void JointPath::setNumericalIkDeltaScale(double s)
{
    getOrCreateNumericalIK()->deltaScale = s;
}


void JointPath::setNumericalIkMaxIterations(int n)
{
    getOrCreateNumericalIK()->maxIterations = n;
}


void JointPath::setNumericalIkDampingConstant(double lambda)
{
    getOrCreateNumericalIK()->dampingConstantSqr = lambda * lambda;
}


/**
   \deprecated
   This parameter is used when SVD is used to solve a numerical IK, but the current
   implementation uses the damped least square method instead of SVD by default.
   Therefore this parameter is meaningless in the current implementation.
*/
void JointPath::setNumericalIkTruncateRatio(double r)
{
    getOrCreateNumericalIK()->svd.setTruncateRatio(r);
}

    
void JointPath::customizeTarget
(int numTargetElements,
 std::function<double(VectorXd& out_error)> errorFunc,
 std::function<void(MatrixXd& out_Jacobian)> jacobianFunc)
{
    auto nuIK = getOrCreateNumericalIK();
    nuIK->dTask.resize(numTargetElements);
    nuIK->errorFunc = errorFunc;
    nuIK->jacobianFunc = jacobianFunc;
}


JointPath& JointPath::setBaseLinkGoal(const Isometry3& T)
{
    linkPath_.baseLink()->setPosition(T);
    needForwardKinematicsBeforeIK = true;
    return *this;
}


bool JointPath::calcInverseKinematics()
{
    if(numericalIK && numericalIK->errorFunc){
        Isometry3 T; // dummy
        return calcInverseKinematics(T);
    }
    return false;
}


bool JointPath::calcInverseKinematics(const Isometry3& T)
{
    const bool USE_USUAL_INVERSE_SOLUTION_FOR_6x6_NON_BEST_EFFORT_PROBLEM = false;
    const bool USE_SVD_FOR_BEST_EFFORT_IK = false;

    if(joints_.empty()){
        if(linkPath_.empty()){
            return false;
        }
        if(baseLink() == endLink()){
            endLink()->setPosition(T);
            if(endLink()->isFreeJoint() && !endLink()->isRoot()){
                endLink()->setOffsetPosition(endLink()->parent()->T().inverse() * T);
            }
            return true;
        } else {
            // \todo implement here
            return false;
        }
    }
    const int n = numJoints();

    auto nuIK = getOrCreateNumericalIK();
    
    if(!nuIK->jacobianFunc){
        nuIK->jacobianFunc = [&](MatrixXd& out_Jacobian){ setJacobian<0x3f, 0, 0>(*this, endLink(), out_Jacobian); };
    }
    nuIK->resize(n);
    
    Link* target = linkPath_.endLink();

    if(needForwardKinematicsBeforeIK){
        calcForwardKinematics();
        needForwardKinematicsBeforeIK = false;
    }

    nuIK->q0.resize(n);
    if(!nuIK->isBestEffortIkMode){
        for(int i=0; i < n; ++i){
            nuIK->q0[i] = joints_[i]->q();
        }
        nuIK->T0 = target->T();
    }

    double prevErrsqr = std::numeric_limits<double>::max();
    bool completed = false;

    bool useUsualInverseSolution = false;
    if(USE_USUAL_INVERSE_SOLUTION_FOR_6x6_NON_BEST_EFFORT_PROBLEM){
        if(!nuIK->isBestEffortIkMode && (n == 6) && (nuIK->dTask.size() == 6)){
            useUsualInverseSolution = true;
        }
    }
        
    if(USE_SVD_FOR_BEST_EFFORT_IK && !useUsualInverseSolution && !nuIK->isBestEffortIkMode){
        // disable truncation
        nuIK->svd.setTruncateRatio(std::numeric_limits<double>::max());
    }

    for(nuIK->iteration = 0; nuIK->iteration < nuIK->maxIterations; ++nuIK->iteration){

        double errorSqr;
        if(nuIK->errorFunc){
            errorSqr = nuIK->errorFunc(nuIK->dTask);
        } else {
            nuIK->dTask.head<3>() = T.translation() - target->p();
            nuIK->dTask.segment<3>(3) = target->R() * omegaFromRot(target->R().transpose() * T.linear());
            errorSqr = nuIK->dTask.squaredNorm();
        }
        if(errorSqr < nuIK->maxIkErrorSqr){
            completed = true;
            target->T() = T;
            break;
        }
        if(prevErrsqr - errorSqr < nuIK->maxIkErrorSqr){
            if(nuIK->isBestEffortIkMode && (errorSqr > prevErrsqr)){
                // Revert the joint displacements to the previous state in this iteration
                for(int j=0; j < n; ++j){
                    joints_[j]->q() = nuIK->q0[j];
                }
                calcForwardKinematics();
            }
            break;
        }
        prevErrsqr = errorSqr;

        nuIK->jacobianFunc(nuIK->J);

        if(useUsualInverseSolution){
            nuIK->dq = nuIK->QR.compute(nuIK->J).solve(nuIK->dTask);
        } else {
            if(USE_SVD_FOR_BEST_EFFORT_IK){
                nuIK->svd.compute(nuIK->J).solve(nuIK->dTask, nuIK->dq);
            } else {
                // The damped least squares (singurality robust inverse) method
                nuIK->JJ = nuIK->J * nuIK->J.transpose() + nuIK->dampingConstantSqr * MatrixXd::Identity(nuIK->J.rows(), nuIK->J.rows());
                nuIK->dq = nuIK->J.transpose() * nuIK->QR.compute(nuIK->JJ).solve(nuIK->dTask);
            }
        }

        if(nuIK->isBestEffortIkMode){
            for(int j=0; j < n; ++j){
                double& q = joints_[j]->q();
                nuIK->q0[j] = q;
                q += nuIK->deltaScale * nuIK->dq(j);
            }
        } else {
            for(int j=0; j < n; ++j){
                joints_[j]->q() += nuIK->deltaScale * nuIK->dq(j);
            }
        }

        calcForwardKinematics();
    }

    if(!completed && !nuIK->isBestEffortIkMode){
        for(int i=0; i < n; ++i){
            joints_[i]->q() = nuIK->q0[i];
        }
        calcForwardKinematics();
        target->T() = nuIK->T0;
    }

    return completed;
}


bool JointPath::calcRemainingPartForwardKinematicsForInverseKinematics()
{
    if(!remainingLinkTraverse){
        remainingLinkTraverse = make_shared<LinkTraverse>(baseLink(), true, true);
        for(auto& link : linkPath_){
            remainingLinkTraverse->remove(link);
        }
        remainingLinkTraverse->prependRootAdjacentLinkToward(baseLink());
    }
    remainingLinkTraverse->calcForwardKinematics();
    return true;
}
        
    
int JointPath::numIterations() const
{
    return numericalIK ? numericalIK->iteration : 0;
}


bool JointPath::hasCustomIK() const
{
    return false;
}


bool JointPath::hasAnalyticalIK() const
{
    return hasCustomIK();
}


bool JointPath::calcInverseKinematics
(const Vector3& base_p, const Matrix3& base_R, const Vector3& end_p, const Matrix3& end_R)
{
    Isometry3 T_base;
    T_base.linear() = base_R;
    T_base.translation() = base_p;

    Isometry3 T_end;
    T_end.linear() = end_R;
    T_end.translation() = end_p;
    
    return setBaseLinkGoal(T_base).calcInverseKinematics(T_end);
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

// deprecated
class JointPathWithCustomizerIk : public JointPath
{
    BodyPtr body;
    int ikTypeId;
    bool isCustomizedIkPathReversed;
    
public:
    JointPathWithCustomizerIk(const BodyPtr& body, Link* baseLink, Link* endLink)
        : JointPath(baseLink, endLink),
          body(body)
    {
        ikTypeId = body->customizerInterface()->initializeAnalyticIk(
            body->customizerHandle(), baseLink->index(), endLink->index());
        if(ikTypeId){
            isCustomizedIkPathReversed = false;
        } else {
            // try reversed path
            ikTypeId = body->customizerInterface()->initializeAnalyticIk(
                body->customizerHandle(), endLink->index(), baseLink->index());
            if(ikTypeId){
                isCustomizedIkPathReversed = true;
            }
        }
    }
        
    virtual bool hasCustomIK() const override
    {
        return (ikTypeId != 0);
    }
    
    virtual bool calcInverseKinematics(const Isometry3& T) override
    {
        if(isCustomIkDisabled() || ikTypeId == 0){
            return JointPath::calcInverseKinematics(T);
        }
        
        const Link* baseLink_ = baseLink();
        Vector3 p;
        Matrix3 R;
        
        if(!isCustomizedIkPathReversed){
            p = baseLink_->R().transpose() * (T.translation() - baseLink_->p());
            R.noalias() = baseLink_->R().transpose() * T.linear();
        } else {
            p = T.linear().transpose() * (baseLink_->p() - T.translation());
            R.noalias() = T.linear().transpose() * baseLink_->R();
        }
        
        bool solved = body->customizerInterface()->
            calcAnalyticIk(body->customizerHandle(), ikTypeId, p, R);

        if(solved){
            calcForwardKinematics();
        }
        
        return solved;
    }
};

}


std::shared_ptr<JointPath> JointPath::getCustomPath(Body* body, Link* baseLink, Link* endLink)
{
    auto customJointPathHandler = body->findHandler<CustomJointPathHandler>();
    if(customJointPathHandler){
        auto customPath = customJointPathHandler->getCustomJointPath(baseLink, endLink);
        if(customPath){
            return customPath;
        }
    }

    // deprecated
    if(body->customizerInterface() && body->customizerInterface()->initializeAnalyticIk){
        return make_shared<JointPathWithCustomizerIk>(body, baseLink, endLink);
    }

    return make_shared<JointPath>(baseLink, endLink);
}
