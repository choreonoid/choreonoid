/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_LINK_H
#define CNOID_BODY_LINK_H

#include <cnoid/SceneGraph>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT Link
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Link();

    /**
       This does shallow copy.
       You have to use the copy constructor of the Body class to copy the link tree
    */
    Link(const Link& link);
        
    virtual ~Link();

    int index() const { return index_; }
    bool isValid() const { return (index_ >= 0); }

    Link* parent() const { return parent_; }
    Link* sibling() const { return sibling_; }
    Link* child() const { return child_; }
        
    bool isRoot() const { return !parent_; }

    Position& T() { return T_; }
    const Position& T() const { return T_; }

    Position& position() { return T_; }
    const Position& position() const { return T_; }

    template<class Scalar, int Mode, int Options>
        void setPosition(const Eigen::Transform<Scalar, 3, Mode, Options>& T) {
        T_ = T.template cast<Position::Scalar>();
    }

    template<typename Derived1, typename Derived2>
        void setPosition(const Eigen::MatrixBase<Derived1>& rotation, const Eigen::MatrixBase<Derived2>& translation) {
        T_.linear() = rotation;
        T_.translation() = translation;
    }

    Position::TranslationPart p() { return T_.translation(); }
    Position::ConstTranslationPart p() const { return T_.translation(); }
    Position::TranslationPart translation() { return T_.translation(); }
    Position::ConstTranslationPart translation() const { return T_.translation(); }

    template<typename Derived>
    void setTranslation(const Eigen::MatrixBase<Derived>& p) {
        T_.translation() = p.template cast<Affine3::Scalar>();
    }

    Position::LinearPart R() { return T_.linear(); }
    Position::ConstLinearPart R() const { return T_.linear(); }
    Position::LinearPart rotation() { return T_.linear(); }
    Position::ConstLinearPart rotation() const { return T_.linear(); }

    template<typename Derived>
    void setRotation(const Eigen::MatrixBase<Derived>& R) {
        T_.linear() = R.template cast<Affine3::Scalar>();
    }

    template<typename T>
    void setRotation(const Eigen::AngleAxis<T>& a) {
        T_.linear() = a.template cast<Affine3::Scalar>().toRotationMatrix();
    }
    
    // To, Ro?
    Position& Tb() { return Tb_; }
    const Position& Tb() const { return Tb_; }
        
    Position::ConstTranslationPart b() const { return Tb_.translation(); }
    Position::ConstTranslationPart offsetTranslation() const { return Tb_.translation(); }

    Position::ConstLinearPart Rb() const { return Tb_.linear(); }
    Position::ConstLinearPart offsetRotation() const { return Tb_.linear(); }

    Matrix3& Rs() { return Rs_; }
    const Matrix3& Rs() const { return Rs_; }

    enum JointType {
        /// rotational joint (1 dof)
        ROTATIONAL_JOINT,
        /// translational joint (1 dof)
        SLIDE_JOINT,
        /// 6-DOF root link
        FREE_JOINT,
        /*
          Joint types below here are treated as a fixed joint
          when a code for processing a joint type is not given
        */
        /// fixed joint(0 dof)
        FIXED_JOINT,
        /// special joint for pseudo crawler simulation
        CRAWLER_JOINT
    };

    int jointId() const { return jointId_; }
        
    JointType jointType() const { return jointType_; }
    bool isFixedJoint() const { return (jointType_ >= FIXED_JOINT); }
    bool isFreeJoint() const { return jointType_ == FREE_JOINT; }
    bool isRotationalJoint() const { return jointType_ == ROTATIONAL_JOINT; }
    bool isSlideJoint() const { return jointType_ == SLIDE_JOINT; }
        
    const Vector3& a() const { return a_; }    
    const Vector3& jointAxis() const { return a_; }
    const Vector3& d() const { return a_; } // joint axis alias for a slide joint
        
    double q() const { return q_; }
    double& q() { return q_; }
    double dq() const { return dq_; }
    double& dq() { return dq_; }
    double ddq() const { return ddq_; }
    double& ddq() { return ddq_; }
    double u() const { return u_; }
    double& u() { return u_; }

    double q_upper() const { return q_upper_; }  ///< the upper limit of joint values
    double q_lower() const { return q_lower_; }  ///< the lower limit of joint values
    double dq_upper() const { return dq_upper_; } ///< the upper limit of joint velocities
    double dq_lower() const { return dq_lower_; } ///< the upper limit of joint velocities

    const Vector3& v() const { return v_; }
    Vector3& v() { return v_; }
    const Vector3& w() const { return w_; }
    Vector3& w() { return w_; }
    const Vector3& dv() const { return dv_; }
    Vector3& dv() { return dv_; }
    const Vector3& dw() const { return dw_; }
    Vector3& dw() { return dw_; }

    /// center of mass (self local)
    const Vector3& c() const { return c_; }
    const Vector3& centerOfMass() const { return c_; }
        
    /// center of mass (world coordinate)
    const Vector3& wc() const { return wc_; }
    const Vector3& centerOfMassGlobal() const { return wc_; }
    Vector3& wc() { return wc_; }
        
    /// mass
    double m() const { return m_; }
    double mass() const { return m_; }

    ///< inertia tensor (self local, around c)
    const Matrix3& I() const { return I_; }    

    /// Equivalent rotor inertia: n^2*Jm [kg.m^2]
    double Jm2() const { return Jm2_; }

    const Vector6& F_ext() const { return F_ext_; }
    Vector6& F_ext() { return F_ext_; }
    Vector6::ConstFixedSegmentReturnType<3>::Type f_ext() const { return F_ext_.head<3>(); }
    Vector6::FixedSegmentReturnType<3>::Type f_ext() { return F_ext_.head<3>(); }
    Vector6::ConstFixedSegmentReturnType<3>::Type tau_ext() const { return F_ext_.tail<3>(); }
    Vector6::FixedSegmentReturnType<3>::Type tau_ext() { return F_ext_.tail<3>(); }

    const std::string& name() const { return name_; }

    SgNode* shape() const { return visualShape_; }
    SgNode* visualShape() const { return visualShape_; }
    SgNode* collisionShape() const { return collisionShape_; }

    // functions for constructing a link
    void setIndex(int index) { index_ = index; }

    virtual void prependChild(Link* link);
    virtual void appendChild(Link* link);
    bool removeChild(Link* link);

    void setOffsetPosition(const Position& T){
        Tb_ = T;
    }
        
    template<typename Derived>
        void setOffsetTranslation(const Eigen::MatrixBase<Derived>& offset) {
        Tb_.translation() = offset;
    }
    template<typename Derived>
        void setOffsetRotation(const Eigen::MatrixBase<Derived>& offset) {
        Tb_.linear() = offset;
    }
    template<typename Derived>
        void setAccumlatedSegmentRotation(const Eigen::MatrixBase<Derived>& Rs) {
        Rs_ = Rs;
    }
        
    void setJointType(JointType type) { jointType_ = type; }
    void setJointId(int id) { jointId_ = id; }
    void setJointAxis(const Vector3& axis) { a_ = axis; }
        
    void setJointRange(double lower, double upper) { q_lower_ = lower; q_upper_ = upper; }
    void setJointVelocityRange(double lower, double upper) { dq_lower_ = lower; dq_upper_ = upper; }

    void setMass(double m) { m_ = m; }
    void setInertia(const Matrix3& I) { I_ = I; }
    void setCenterOfMass(const Vector3& c) { c_ = c; }
    void setEquivalentRotorInertia(double Jm2) { Jm2_ = Jm2; }
        
    void setName(const std::string& name);

    void setShape(SgNodePtr shape);
    void setVisualShape(SgNodePtr shape);
    void setCollisionShape(SgNodePtr shape);

    // The following two methods should be deprecated after introducing Tb
    Matrix3 attitude() const { return R() * Rs_; }
    void setAttitude(const Matrix3& Ra) { R() = Ra * Rs_.transpose(); }
    Matrix3 calcRfromAttitude(const Matrix3& Ra) { return Ra * Rs_.transpose(); }


#ifdef CNOID_BACKWARD_COMPATIBILITY
    // fext, tauext
    const double& ulimit() const { return q_upper_; }  ///< the upper limit of joint values
    const double& llimit() const { return q_lower_; }  ///< the lower limit of joint values
    const double& uvlimit() const { return dq_upper_; } ///< the upper limit of joint velocities
    const double& lvlimit() const { return dq_lower_; } ///< the upper limit of joint velocities

    Matrix3 segmentAttitude() const { return R() * Rs_; }
    void setSegmentAttitude(const Matrix3& Ra) { R() = Ra * Rs_.transpose(); }
#endif

private:
    int index_; 
    int jointId_;
    Link* parent_;
    Link* sibling_;
    Link* child_;
    Position T_;
    Position Tb_;
    Matrix3 Rs_; // temporary variable for porting. This should be removed later.
    Vector3 a_;
    JointType jointType_;
    double q_;
    double dq_;
    double ddq_;
    double u_;
    Vector3	v_;
    Vector3	w_;
    Vector3	dv_;
    Vector3	dw_;
    Vector3 c_;
    Vector3 wc_;
    double m_;
    Matrix3 I_;
    double Jm2_;
    Vector6 F_ext_; // should be Vector3 x 2?
    double q_upper_;
    double q_lower_;
    double dq_upper_;
    double dq_lower_;
    std::string name_;
    SgNodePtr visualShape_;
    SgNodePtr collisionShape_;
};

}
	
#endif
