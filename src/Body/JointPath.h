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

class NumericalIK;
class LinkTraverse;
class Body;

class CNOID_EXPORT JointPath : public InverseKinematics
{
public:
    /**
       This function returns a joint path which may do analytical inverse kinematics
       when the body has the analytical one for a given path.
    */
    static std::shared_ptr<JointPath> getCustomPath(Body* body, Link* baseLink, Link* endLink);
    
    JointPath();
    JointPath(Link* base, Link* end);
    JointPath(Link* end);
    JointPath(const JointPath& org) = delete;
    JointPath& operator=(const JointPath& rhs) = delete;

    bool empty() const {
        return joints_.empty();
    }
		
    int size() const {
        return static_cast<int>(joints_.size());
    }

    int numJoints() const {
        return size();
    }
		
    Link* joint(int index) const {
        return joints_[index];
    }

    Link* operator[] (int index) const {
        return joints_[index];
    }

    Link* baseLink() const {
        return linkPath_.baseLink();
    }

    Link* endLink() const {
        return linkPath_.endLink();
    }

    bool isJointDownward(int index) const {
        return (index >= numUpwardJointConnections);
    }

    const std::vector<LinkPtr>& joints() const { return joints_; }

    typedef LinkTraverse::iterator iterator;
    typedef LinkTraverse::const_iterator const_iterator;

    iterator begin() { return joints_.begin(); }
    iterator end() { return joints_.end(); }
    const_iterator begin() const { return joints_.begin(); }
    const_iterator end() const { return joints_.end(); }

    LinkPath& linkPath() { return linkPath_; }
    const LinkPath& linkPath() const { return linkPath_; }

    void calcForwardKinematics(bool calcVelocity = false, bool calcAcceleration = false) const {
        linkPath_.calcForwardKinematics(calcVelocity, calcAcceleration);
    }

    int indexOf(const Link* link) const;

    virtual bool hasCustomIK() const;
    bool isCustomIkDisabled() const { return isCustomIkDisabled_; }
    void setCustomIkDisabled(bool on) { isCustomIkDisabled_ = on; }
    
    bool isBestEffortIkMode() const;
    void setBestEffortIkMode(bool on);
    void setNumericalIkMaxIkError(double e);
    void setNumericalIkDeltaScale(double s);
    void setNumericalIkMaxIterations(int n);
    void setNumericalIkDampingConstant(double lambda);
    static double numericalIkDefaultDeltaScale();
    static int numericalIkDefaultMaxIterations();
    static double numericalIkDefaultMaxIkError();
    static double numericalIkDefaultDampingConstant();
    
    void customizeTarget(
        int numTargetElements,
        std::function<double(VectorXd& out_error)> errorFunc,
        std::function<void(MatrixXd& out_Jacobian)> jacobianFunc);

    // For the path customized by the customizeTarget function
    bool calcInverseKinematics();

    JointPath& storeCurrentPosition();

    JointPath& setBaseLinkGoal(const Isometry3& T);

    virtual bool calcInverseKinematics(const Isometry3& T) override;
    virtual bool calcRemainingPartForwardKinematicsForInverseKinematics() override;

    int numIterations() const;

    std::string name() const { return name_; }
    void setName(const std::string& name){ name_ = name; }

    [[deprecated("Use calcInverseKinematics(const Isometry3& T)")]]
    bool calcInverseKinematics(const Vector3& p, const Matrix3& R) {
        return InverseKinematics::calcInverseKinematics(p, R);
    }
    [[deprecated("Use calcInverseKinematics(const Isometry3& T)")]]
    bool calcInverseKinematics(
        const Vector3& base_p, const Matrix3& base_R, const Vector3& end_p, const Matrix3& end_R);
    [[deprecated]]
    void calcJacobian(Eigen::MatrixXd& out_J) const;
    [[deprecated("Use hasCustomIK")]]
    bool hasAnalyticalIK() const;
    [[deprecated("Use setCustomIkDisabled.")]]
    void setNumericalIKenabled(bool on) { setCustomIkDisabled(on); }
    [[deprecated("Use isCustomIkDisabled.")]]
    bool isNumericalIkEnabled() const { return isCustomIkDisabled(); }
    [[deprecated("Use isBestEffortIkMode.")]]
    bool isBestEffortIKmode() const { return isBestEffortIkMode(); }
    [[deprecated("Use setBestEffortIkMode.")]]
    void setBestEffortIKmode(bool on) { setBestEffortIkMode(on); }
    [[deprecated("Use setNumericalIkMaxIkError.")]]
    void setNumericalIKmaxIKerror(double e){ setNumericalIkMaxIkError(e); }
    [[deprecated("Use setNumericalIkDeltaScale.")]]
    void setNumericalIKdeltaScale(double s) { setNumericalIkDeltaScale(s); }
    [[deprecated("Use setNumericalIkMaxIterations.")]]
    void setNumericalIKmaxIterations(int n) { setNumericalIkMaxIterations(n); }
    [[deprecated("Use setNumericalIkDampingConstant.")]]
    void setNumericalIKdampingConstant(double lambda) { setNumericalIkDampingConstant(lambda); }
    [[deprecated("Use numericalIkDefaultDeltaScale.")]]
    static double numericalIKdefaultDeltaScale(){ return numericalIkDefaultDeltaScale(); }
    [[deprecated("Use numericalIkDefaultMaxIterations.")]]
    static int numericalIKdefaultMaxIterations(){ return numericalIkDefaultMaxIterations(); }
    [[deprecated("Use numericalIkDefaultMaxIkError.")]]
    static double numericalIKdefaultMaxIKerror(){ return numericalIkDefaultMaxIkError(); }
    [[deprecated("Use numericalIkDefaultDampingConstant.")]]
    static double numericalIKdefaultDampingConstant(){ return numericalIkDefaultDampingConstant(); }
    [[deprecated]]
    void setNumericalIkTruncateRatio(double r);
    [[deprecated]]
    static double numericalIkDefaultTruncateRatio();

private:
    void initialize();
    void extractJoints();
    void doResetWhenJointPathUpdated();
    NumericalIK* getOrCreateNumericalIK();

    LinkPath linkPath_;
    std::vector<LinkPtr> joints_;
    std::shared_ptr<LinkTraverse> remainingLinkTraverse;
    NumericalIK* numericalIK;
    int numUpwardJointConnections;
    bool needForwardKinematicsBeforeIK;
    bool isCustomIkDisabled_;
    std::string name_;
};

//! \deprecated Use JointPath::getCustomPath instead of this.
inline std::shared_ptr<JointPath> getCustomJointPath(Body* body, Link* baseLink, Link* endLink){
    return JointPath::getCustomPath(body, baseLink, endLink);
}

#ifdef CNOID_BACKWARD_COMPATIBILITY
typedef std::shared_ptr<JointPath> JointPathPtr;
#endif

}

#endif
