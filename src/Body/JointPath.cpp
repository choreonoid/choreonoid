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

class NumericalIK
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

    NumericalIK() {
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


bool JointPath::isBestEffortIKmode() const
{
    return numericalIK ? numericalIK->isBestEffortIKmode : false;
}


void JointPath::setBestEffortIKmode(bool on)
{
    getOrCreateNumericalIK()->isBestEffortIKmode = on;
}


void JointPath::setNumericalIKmaxIKerror(double e)
{
    getOrCreateNumericalIK()->maxIKerrorSqr = e * e;
}


void JointPath::setNumericalIKdeltaScale(double s)
{
    getOrCreateNumericalIK()->deltaScale = s;
}


void JointPath::setNumericalIKtruncateRatio(double r)
{
    getOrCreateNumericalIK()->svd.setTruncateRatio(r);
}

    
void JointPath::setNumericalIKmaxIterations(int n)
{
    getOrCreateNumericalIK()->maxIterations = n;
}


void JointPath::setNumericalIKdampingConstant(double lambda)
{
    getOrCreateNumericalIK()->dampingConstantSqr = lambda * lambda;
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


JointPath& JointPath::setBaseLinkGoal(const Position& T)
{
    linkPath_.baseLink()->setPosition(T);
    needForwardKinematicsBeforeIK = true;
    return *this;
}


bool JointPath::calcInverseKinematics()
{
    if(numericalIK->errorFunc){
        Position T; // dummy
        return calcInverseKinematics(T);
    }
    return false;
}


bool JointPath::calcInverseKinematics(const Position& T)
{
    const bool USE_USUAL_INVERSE_SOLUTION_FOR_6x6_NON_BEST_EFFORT_PROBLEM = false;
    const bool USE_SVD_FOR_BEST_EFFORT_IK = false;

    if(joints_.empty()){
        if(linkPath_.empty()){
            return false;
        }
        if(baseLink() == endLink()){
            baseLink()->setPosition(T);
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
    if(!nuIK->isBestEffortIKmode){
        for(int i=0; i < n; ++i){
            nuIK->q0[i] = joints_[i]->q();
        }
    }

    double prevErrsqr = std::numeric_limits<double>::max();
    bool completed = false;

    bool useUsualInverseSolution = false;
    if(USE_USUAL_INVERSE_SOLUTION_FOR_6x6_NON_BEST_EFFORT_PROBLEM){
        if(!nuIK->isBestEffortIKmode && (n == 6) && (nuIK->dTask.size() == 6)){
            useUsualInverseSolution = true;
        }
    }
        
    if(USE_SVD_FOR_BEST_EFFORT_IK && !useUsualInverseSolution && !nuIK->isBestEffortIKmode){
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
        if(errorSqr < nuIK->maxIKerrorSqr){
            completed = true;
            break;
        }
        if(prevErrsqr - errorSqr < nuIK->maxIKerrorSqr){
            if(nuIK->isBestEffortIKmode && (errorSqr > prevErrsqr)){
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

        if(nuIK->isBestEffortIKmode){
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

    if(!completed && !nuIK->isBestEffortIKmode){
        for(int i=0; i < n; ++i){
            joints_[i]->q() = nuIK->q0[i];
        }
        calcForwardKinematics();
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


void JointPath::setNumericalIKenabled(bool on)
{
    if(on){
        getOrCreateNumericalIK();
    } else {
        if(numericalIK){
            delete numericalIK;
            numericalIK = nullptr;
        }
    }
}


bool JointPath::calcInverseKinematics
(const Vector3& base_p, const Matrix3& base_R, const Vector3& end_p, const Matrix3& end_R)
{
    Position T_base;
    T_base.linear() = base_R;
    T_base.translation() = base_p;

    Position T_end;
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
    
    virtual bool calcInverseKinematics(const Position& T) override
    {
        if(isNumericalIkEnabled() || ikTypeId == 0){
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
