/**
   \file
   \brief The header file of the JointPath class
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_JOINT_PATH_H
#define CNOID_BODY_JOINT_PATH_H

#include "LinkPath.h"
#include "InverseKinematics.h"
#include <cnoid/Referenced>
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

    bool setPath(Link* base, Link* end);
    bool setPath(Link* end);

    //! Deprecated. Use "setPath()" instead of this.
    bool find(Link* base, Link* end) { return setPath(base, end); }
    //! Deprecated. Use "setPath()" instead of this.
    bool find(Link* end) { return setPath(end); }

    bool empty() const {
        return joints.empty();
    }
		
    int numJoints() const {
        return joints.size();
    }
		
    Link* joint(int index) const {
        return joints[index];
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

    void calcForwardKinematics(bool calcVelocity = false, bool calcAcceleration = false) const {
        linkPath.calcForwardKinematics(calcVelocity, calcAcceleration);
    }

    int indexOf(const Link* link) const;

    bool isNumericalIkEnabled() const { return nuIK != 0; }

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

    // InverseKinematics Interface
    virtual bool hasAnalyticalIK() const;

    JointPath& storeCurrentPosition();

    JointPath& setBaseLinkGoal(const Position& T);


    virtual bool calcInverseKinematics(const Position& T) override;

    /*
    bool calcNumericalIK() {
        return JointPath::calcInverseKinematics();
    }
    */

    int numIterations() const;

    /*
    JointPath& setGoal(const Vector3& end_p, const Matrix3& end_R) {
        targetTranslationGoal = end_p;
        targetRotationGoal = end_R;
        return *this;
    }
    */

    //! deprecated
    //JointPath& setGoal(const Vector3& base_p, const Matrix3& base_R, const Vector3& end_p, const Matrix3& end_R);

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

protected:
		
    virtual void onJointPathUpdated();

private:

    JointPath(const JointPath& org);
		
    void initialize();
    void extractJoints();
    JointPathIkImpl* getOrCreateNumericalIK();

    LinkPath linkPath;
    std::vector<Link*> joints;
    int numUpwardJointConnections;
    bool needForwardKinematicsBeforeIK;
    JointPathIkImpl* nuIK; // numerical IK
};

typedef std::shared_ptr<JointPath> JointPathPtr;

class Body;

/**
   This function returns a joint path which may do analytical inverse kinematics
   when the body has the analytical one for a given path.
   \todo move back this function to the Body class
*/
CNOID_EXPORT JointPathPtr getCustomJointPath(Body* body, Link* baseLink, Link* targetLink);

}

#endif
