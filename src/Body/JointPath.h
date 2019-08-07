/**
   \file
   \brief The header file of the JointPath class
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_JOINT_PATH_H
#define CNOID_BODY_JOINT_PATH_H

#include "LinkPath.h"
#include "InverseKinematics.h"
#include <cnoid/EigenTypes>
#include <functional>
#include <memory>
#include "exportdecl.h"

namespace cnoid {

class JointPathIkImpl;

class CNOID_EXPORT JointPath : public InverseKinematics
{
public:
    JointPath();
    JointPath(Link* base, Link* end);
    JointPath(Link* end);
    virtual ~JointPath();

    bool empty() const {
        return joints_.empty();
    }
		
    int numJoints() const {
        return joints_.size();
    }
		
    Link* joint(int index) const {
        return joints_[index];
    }

    Link* baseLink() const {
        return linkPath.baseLink();
    }

    Link* endLink() const {
        return linkPath.endLink();
    }

    bool isJointDownward(int index) const {
        return (index >= numUpwardJointConnections);
    }

    LinkPath::accessor joints() { return LinkPath::accessor(joints_); }
    LinkPath::const_accessor joints() const { return LinkPath::const_accessor(joints_); }

    void calcForwardKinematics(bool calcVelocity = false, bool calcAcceleration = false) const {
        linkPath.calcForwardKinematics(calcVelocity, calcAcceleration);
    }

    int indexOf(const Link* link) const;

    bool isNumericalIkEnabled() const { return nuIK != 0; }
    void setNumericalIKenabled(bool on);

    bool isBestEffortIKmode() const;
    void setBestEffortIKmode(bool on);

    void setNumericalIKmaxIKerror(double e);
    void setNumericalIKdeltaScale(double s);
    void setNumericalIKmaxIterations(int n);
    void setNumericalIKdampingConstant(double lambda);
        
    static double numericalIKdefaultDeltaScale();
    static int numericalIKdefaultMaxIterations();
    static double numericalIKdefaultMaxIKerror();
    static double numericalIKdefaultDampingConstant();
        
    void customizeTarget(
        int numTargetElements,
        std::function<double(VectorXd& out_error)> errorFunc,
        std::function<void(MatrixXd& out_Jacobian)> jacobianFunc);

    // For the path customized by the customizeTarget function
    bool calcInverseKinematics();

    JointPath& storeCurrentPosition();

    JointPath& setBaseLinkGoal(const Position& T);

    virtual bool calcInverseKinematics(const Position& T) override;

    int numIterations() const;

    virtual bool hasCustomIK() const;

    //! deprecated
    bool calcInverseKinematics(const Vector3& p, const Matrix3& R) {
        return InverseKinematics::calcInverseKinematics(p, R);
    }
    //! deprecated
    bool calcInverseKinematics(
        const Vector3& base_p, const Matrix3& base_R, const Vector3& end_p, const Matrix3& end_R);
    //! deprecated
    void calcJacobian(Eigen::MatrixXd& out_J) const;
    //! deprecated
    void setNumericalIKtruncateRatio(double r);
    //! deprecated
    static double numericalIKdefaultTruncateRatio();

    //! deprecated. Use hasCustomIK() insted of this.
    bool hasAnalyticalIK() const;

private:

    JointPath(const JointPath& org);
		
    void initialize();
    void extractJoints();
    void doResetWhenJointPathUpdated();
    JointPathIkImpl* getOrCreateNumericalIK();

    LinkPath linkPath;
    std::vector<Link*> joints_;
    int numUpwardJointConnections;
    bool needForwardKinematicsBeforeIK;
    JointPathIkImpl* nuIK; // numerical IK
};

class Body;

/**
   This function returns a joint path which may do analytical inverse kinematics
   when the body has the analytical one for a given path.
   \todo move back this function to the Body class
*/
CNOID_EXPORT std::shared_ptr<JointPath> getCustomJointPath(Body* body, Link* baseLink, Link* endLink);

#ifdef CNOID_BACKWARD_COMPATIBILITY
typedef std::shared_ptr<JointPath> JointPathPtr;
#endif

}

#endif
