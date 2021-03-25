/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_LINK_H
#define CNOID_BODY_LINK_H

#include <cnoid/ClonableReferenced>
#include <cnoid/EigenTypes>
#include <cnoid/SceneUpdate>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class Body;
class SgNode;
class SgGroup;
class SgPosTransform;
class Mapping;

class Link;
typedef ref_ptr<Link> LinkPtr;

class CNOID_EXPORT Link : public ClonableReferenced
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Link();

    /**
       This does shallow copy.
       You have to use the copy constructor of the Body class to copy the link tree
    */
    Link(const Link& link);

    Link* clone() const {
        return static_cast<Link*>(doClone(nullptr));
    }
        
    virtual ~Link();

    virtual void initializeState();

    const std::string& name() const { return name_; }
    int index() const { return index_; }
    bool isValid() const { return (index_ >= 0); }
    bool isRoot() const { return !parent_; }
    bool isStatic() const;
    bool isFixedToRoot() const;
    bool isOwnerOf(const Link* link) const;
    bool isEndLink() const { return !child_; }

    Body* body() { return body_; }
    const Body* body() const { return body_; }
    Link* parent() const { return parent_; }
    Link* sibling() const { return sibling_; }
    Link* child() const { return child_; }

    Isometry3& T() { return T_; }
    const Isometry3& T() const { return T_; }

    Isometry3& position() { return T_; }
    const Isometry3& position() const { return T_; }

    template<class Scalar, int Mode, int Options>
    void setPosition(const Eigen::Transform<Scalar, 3, Mode, Options>& T){
        T_ = T.template cast<Isometry3::Scalar>();
    }
    template<class Derived>
    void setPosition(const Eigen::MatrixBase<Derived>& T){
        T_ = T.template cast<Isometry3::Scalar>();
    }
    template<typename Derived1, typename Derived2>
    void setPosition(const Eigen::MatrixBase<Derived1>& rotation, const Eigen::MatrixBase<Derived2>& translation){
        T_.linear() = rotation;
        T_.translation() = translation;
    }

    Isometry3::TranslationPart p() { return T_.translation(); }
    Isometry3::ConstTranslationPart p() const { return T_.translation(); }
    Isometry3::TranslationPart translation() { return T_.translation(); }
    Isometry3::ConstTranslationPart translation() const { return T_.translation(); }

    template<typename Derived>
    void setTranslation(const Eigen::MatrixBase<Derived>& p) {
        T_.translation() = p.template cast<Isometry3::Scalar>();
    }

    Isometry3::LinearPart R() { return T_.linear(); }
    Isometry3::ConstLinearPart R() const { return T_.linear(); }
    Isometry3::LinearPart rotation() { return T_.linear(); }
    Isometry3::ConstLinearPart rotation() const { return T_.linear(); }

    template<typename Derived>
    void setRotation(const Eigen::MatrixBase<Derived>& R) {
        T_.linear() = R.template cast<Isometry3::Scalar>();
    }
    template<typename T>
    void setRotation(const Eigen::AngleAxis<T>& a) {
        T_.linear() = a.template cast<Isometry3::Scalar>().toRotationMatrix();
    }
    template<typename Derived>
    void setRotation(const Eigen::QuaternionBase<Derived>& q) {
        T_.linear() = q.template cast<Isometry3::Scalar>().toRotationMatrix();
    }
    
    // To, Ro?
    const Isometry3& Tb() const { return Tb_; }
    const Isometry3& offsetPosition() const { return Tb_; }
        
    Isometry3::ConstTranslationPart b() const { return Tb_.translation(); }
    Isometry3::ConstTranslationPart offsetTranslation() const { return Tb_.translation(); }

    Isometry3::ConstLinearPart Rb() const { return Tb_.linear(); }
    Isometry3::ConstLinearPart offsetRotation() const { return Tb_.linear(); }

    [[deprecated("This func. always returns the identity matrix")]]
    Matrix3 Rs() const { return Matrix3::Identity(); }

    enum JointType {
        RevoluteJoint = 0,
        PrismaticJoint = 1,
        /// 6-DOF root link
        FreeJoint = 2,
        /*
          Joint types below here are treated as a fixed joint
          when a code for processing a joint type is not given
        */
        FixedJoint = 3,

        /**
           Special joint for simplified simulation of a continuous track
        */
        PseudoContinuousTrackJoint = 4,

        // Deprecated
        REVOLUTE_JOINT = RevoluteJoint,
        ROTATIONAL_JOINT = RevoluteJoint,
        PRISMATIC_JOINT = PrismaticJoint,
        SLIDE_JOINT = PrismaticJoint,
        FREE_JOINT = FreeJoint,
        FIXED_JOINT = FixedJoint,
        PSEUDO_CONTINUOUS_TRACK = PseudoContinuousTrackJoint
    };

#if !defined(__GNUC__) || __GNUC__ > 5
    [[deprecated("Use Link::PseudoContinuousTrack.")]]
#endif
    static constexpr int PseudoContinousTrack = PseudoContinuousTrackJoint;

    int jointId() const { return jointId_; }
    const std::string& jointName() const;
    const std::string& jointSpecificName() const { return jointSpecificName_; }
        
    JointType jointType() const { return static_cast<JointType>(jointType_); }
    const char* jointTypeLabel() const;
    const char* jointTypeSymbol() const;
    [[deprecated("Use jointTypeLabel or jointTypeSymbol")]]
    const char* jointTypeString(bool useUnderscore = false) const;
    bool isFixedJoint() const { return (jointType_ >= FixedJoint); }
    bool isFreeJoint() const { return jointType_ == FreeJoint; }
    bool isRevoluteJoint() const { return jointType_ == RevoluteJoint; }
    bool isPrismaticJoint() const { return jointType_ == PrismaticJoint; }
    bool hasJoint() const { return jointType_ <= 1; }

    /// deprecated
    bool isRotationalJoint() const { return jointType_ == RevoluteJoint; }
    /// deprecated
    bool isSlideJoint() const { return jointType_ == PrismaticJoint; }
        
    const Vector3& a() const { return a_; }    
    const Vector3& jointAxis() const { return a_; }
    const Vector3& d() const { return a_; } // joint axis alias for a slide joint

    /// Equivalent rotor inertia: n^2*Jm [kg.m^2]
    double Jm2() const { return Jm2_; }

    enum StateFlag {

        // States
        StateNone = 0,
        JointDisplacement = 1 << 0,
        JointAngle = JointDisplacement,
        JointVelocity = 1 << 1,
        JointAcceleration = 1 << 2,
        JointEffort = 1 << 3,
        JointForce = JointEffort,
        JointTorque = JointEffort,
        LinkPosition = 1 << 4,
        LinkTwist = 1 << 5,
        LinkExtWrench = 1 << 6,
        LinkContactState = 1 << 7,

        // Options
        HighGainActuation = 1 << 8,

        // Don't use this
        DeprecatedJointSurfaceVelocity = 1 << 9,

        MaxStateTypeBit = 9,
        NumStateTypes = 10,

        // Deprecated
        NO_ACTUATION = StateNone,
        JOINT_TORQUE = JointTorque,
        JOINT_FORCE = JointForce,
        JOINT_EFFORT = JointEffort,
        JOINT_ANGLE = JointAngle,
        JOINT_DISPLACEMENT = JointDisplacement,
        JOINT_VELOCITY = JointVelocity,
        LINK_POSITION = LinkPosition
    };

#if !defined(__GNUC__) || __GNUC__ > 5
    [[deprecated("Use Link::JointVelocity as the actuation mode and Link::PseudoContinuousTrackJoint as the joint type.")]]
#endif
    static constexpr int JOINT_SURFACE_VELOCITY = DeprecatedJointSurfaceVelocity;

    // \ret Logical sum of the correpsonding StateFlag bits
    short actuationMode() const { return actuationMode_; }
    // \param mode Logical sum of the correpsonding StateFlag bits
    void setActuationMode(short mode) { actuationMode_ = mode; }

    [[deprecated("Just use the link name as a prefix or use the int type as a variable.")]]
    typedef StateFlag ActuationMode;
    
    /**
       The special mode which can be used to calculate contact forces only.
       In order for this mode to work correctly, the mode should be specified for all the movable links.
       The mode is currently supported by AISTSimulator.
    */
    static constexpr short AllStateHighGainActuationMode =
        LinkPosition | LinkTwist | LinkExtWrench | JointDisplacement | JointVelocity | JointEffort | HighGainActuation;

    // \ret Logical sum of the correpsonding StateFlag bits
    short sensingMode() const { return sensingMode_; }
    // \param mode Logical sum of the correpsonding StateFlag bits
    void setSensingMode(short mode) { sensingMode_ = mode; }
    void mergeSensingMode(short mode) { sensingMode_ |= mode; }

    static std::string getStateModeString(short mode);
    
    double q() const { return q_; }
    double& q() { return q_; }
    double dq() const { return dq_; }
    double& dq() { return dq_; }
    double ddq() const { return ddq_; }
    double& ddq() { return ddq_; }
    double u() const { return u_; }
    double& u() { return u_; }

    double q_target() const { return q_target_; }  ///< the target position of the joint displacement
    double& q_target() { return q_target_; }       ///< the target position of the joint displacement
    double dq_target() const { return dq_target_; } ///< the target velocity of the joint displacement
    double& dq_target() { return dq_target_; }      ///< the target velocity of the joint displacement

    double q_initial() const { return q_initial_; }
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

    const Vector6& externalWrench() const { return F_ext_; }
    Vector6& externalWrench() { return F_ext_; }
    Vector6::ConstFixedSegmentReturnType<3>::Type externalForce() const { return F_ext_.head<3>(); }
    Vector6::FixedSegmentReturnType<3>::Type externalForce() { return F_ext_.head<3>(); }
    Vector6::ConstFixedSegmentReturnType<3>::Type externalTorque() const { return F_ext_.tail<3>(); }
    Vector6::FixedSegmentReturnType<3>::Type externalTorque() { return F_ext_.tail<3>(); }

    const Vector6& F_ext() const { return F_ext_; }
    Vector6& F_ext() { return F_ext_; }
    Vector6::ConstFixedSegmentReturnType<3>::Type f_ext() const { return F_ext_.head<3>(); }
    Vector6::FixedSegmentReturnType<3>::Type f_ext() { return F_ext_.head<3>(); }
    Vector6::ConstFixedSegmentReturnType<3>::Type tau_ext() const { return F_ext_.tail<3>(); }
    Vector6::FixedSegmentReturnType<3>::Type tau_ext() { return F_ext_.tail<3>(); }

    void addExternalForceAtLocalPosition(const Vector3& f_global, const Vector3& p_local){
        f_ext() += f_global;
        tau_ext() += (T_ * p_local).cross(f_global);
    }

    void addExternalForceAtGlobalPosition(const Vector3& f_global, const Vector3& p_global){
        f_ext() += f_global;
        tau_ext() += p_global.cross(f_global);
    }

    [[deprecated("Use addExternalForceAtLocalPosition.")]]
    void addExternalForce(const Vector3& f_global, const Vector3& p_local){
        addExternalForceAtLocalPosition(f_global, p_local);
    }
    
    int materialId() const { return materialId_; }
    std::string materialName() const;

    //! All the members are described in the global coordinate system
    class ContactPoint
    {
    public:
        ContactPoint(const Vector3& position, const Vector3& normal, const Vector3& force, const Vector3& velocity, double depth)
            : position_(position), normal_(normal), force_(force), velocity_(velocity), depth_(depth) { }

        const Vector3& position() const { return position_; }
        //! The contact normal vector. The direction is from another object to this link.
        const Vector3& normal() const { return normal_; }
        //! The contact force vector. The direction is from another object to this link.
        const Vector3& force() const { return force_; }
        //! The relative velocity of the contact point on this link based on the other link.
        const Vector3& velocity() const { return velocity_; }
        double depth() { return depth_; }

    private:
        Vector3 position_;
        Vector3 normal_;
        Vector3 force_;
        Vector3 velocity_;
        double depth_;

        /**
           The following value is not yet supported, but is better to include in this data structue
           in the future. The id is used to identify the counterpart object (link). It is not
           reasonable to directly store the object pointer because the information could be used in
           another context such as controller implementations and remote communication codes.
           It is easier to handle just an integer value rather than handle the pointer.
           To identy the actual object (link), a simulator item should provide the API to obtain
           the information on the object corresponding to a given ID number.
        */
        //int objectId_;
    };

    /**
       The following contact date is avaiable if LinkContactState is included in the sensingMode.
       \note A dynamics engine should update the data when the above flag is specified in the sensingMode.
    */
    std::vector<ContactPoint>& contactPoints() { return contactPoints_; }
    const std::vector<ContactPoint>& contactPoints() const { return contactPoints_; }
    
    SgGroup* shape() const { return visualShape_; }
    SgGroup* visualShape() const { return visualShape_; }
    SgGroup* collisionShape() const { return collisionShape_; }
    bool hasDedicatedCollisionShape() const;

    // functions for constructing a link
    void setBodyToSubTree(Body* newBody);
    void setParent(Link* parent){ parent_ = parent; }
    void setIndex(int index) { index_ = index; }
    void setName(const std::string& name);
    virtual void prependChild(Link* link);
    virtual void appendChild(Link* link);
    bool removeChild(Link* link);

    void setOffsetPosition(const Isometry3& T){
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
    template<typename T>
    void setOffsetRotation(const Eigen::AngleAxis<T>& a) {
        Tb_.linear() = a.template cast<Isometry3::Scalar>().toRotationMatrix();
    }
    
    template<typename Derived>
    [[deprecated("No need to use this function.")]]
    void setAccumulatedSegmentRotation(const Eigen::MatrixBase<Derived>& /* Rs */) {
    }
        
    void setJointType(JointType type) { jointType_ = type; }
    void setJointId(int id) { jointId_ = id; }
    void setJointName(const std::string& name);
    void resetJointSpecificName();
    void setJointAxis(const Vector3& axis) { a_ = axis; }

    void setInitialJointDisplacement(double q) { q_initial_ = q; }
    void setInitialJointAngle(double q) { q_initial_ = q; }
    void setJointRange(double lower, double upper) { q_lower_ = lower; q_upper_ = upper; }
    void setJointVelocityRange(double lower, double upper) { dq_lower_ = lower; dq_upper_ = upper; }

    void setCenterOfMass(const Vector3& c) { c_ = c; }
    void setMass(double m) { m_ = m; }
    void setInertia(const Matrix3& I) { I_ = I; }
    void setEquivalentRotorInertia(double Jm2) { Jm2_ = Jm2; }

    void setMaterial(int id) { materialId_ = id; }
    void setMaterial(const std::string& name);
    
    void addShapeNode(SgNode* shape, SgUpdateRef update = nullptr);
    void addVisualShapeNode(SgNode* shape, SgUpdateRef update = nullptr);
    void addCollisionShapeNode(SgNode* shape, SgUpdateRef update = nullptr);
    void removeShapeNode(SgNode* shape, SgUpdateRef update = nullptr);
    void clearShapeNodes(SgUpdateRef update = nullptr);

    [[deprecated("You don't have to use this function.")]]
    void updateShapeRs() {}

    // The following two methods should be deprecated after introducing Tb
    [[deprecated("Use T() instead.")]]
    Isometry3 Ta() const { return T(); }
    [[deprecated("Uuse R() instead.")]]
    Matrix3 attitude() const { return R(); }
    [[deprecated("Use setRotation(.) instead.")]]
    void setAttitude(const Matrix3& Ra) { R() = Ra; }
    [[deprecated]]
    Matrix3 calcRfromAttitude(const Matrix3& Ra) { return Ra; }
    [[deprecated("Use T() instead.")]]
    void getAttitudeAndTranslation(Isometry3& out_T) { out_T = T(); };

    const Mapping* info() const { return info_; }
    Mapping* info() { return info_; }
    template<typename T> T info(const std::string& key) const;
    template<typename T> T info(const std::string& key, const T& defaultValue) const;
    template<typename T> void setInfo(const std::string& key, const T& value);
    std::string info(const std::string& key, const char* defaultValue) const;
    void resetInfo(Mapping* info);

#ifdef CNOID_BACKWARD_COMPATIBILITY
    // fext, tauext
    const double& ulimit() const { return q_upper_; }  ///< the upper limit of joint values
    const double& llimit() const { return q_lower_; }  ///< the lower limit of joint values
    const double& uvlimit() const { return dq_upper_; } ///< the upper limit of joint velocities
    const double& lvlimit() const { return dq_lower_; } ///< the upper limit of joint velocities

    Matrix3 segmentAttitude() const { return R(); }
    void setSegmentAttitude(const Matrix3& Ra) { R() = Ra; }
#endif

protected:
    Referenced* doClone(CloneMap* cloneMap) const override;

private:
    int index_; 
    Link* parent_;
    LinkPtr sibling_;
    LinkPtr child_;
    Body* body_;

    Isometry3 T_;
    Isometry3 Tb_;

    short jointType_;
    short jointId_;
    short actuationMode_;
    short sensingMode_;

    Vector3 a_;
    double q_;
    double dq_;
    double ddq_;
    double q_target_;
    double dq_target_;
    double u_;
    
    Vector3 v_;
    Vector3 w_;
    Vector3 dv_;
    Vector3 dw_;
    Vector6 F_ext_; // should be Vector3 x 2?

    Vector3 c_;
    Vector3 wc_;
    double m_;
    Matrix3 I_;

    double Jm2_;
    double q_initial_;
    double q_upper_;
    double q_lower_;
    double dq_upper_;
    double dq_lower_;
    
    int materialId_;

    std::vector<ContactPoint> contactPoints_;
    
    std::string name_;
    std::string jointSpecificName_;

    ref_ptr<SgGroup> visualShape_;
    ref_ptr<SgGroup> collisionShape_;
    
    ref_ptr<Mapping> info_;

    void setBodyToSubTreeIter(Body* newBody);
};

template<> CNOID_EXPORT double Link::info(const std::string& key) const;
template<> CNOID_EXPORT double Link::info(const std::string& key, const double& defaultValue) const;
template<> CNOID_EXPORT bool Link::info(const std::string& key, const bool& defaultValue) const;
template<> CNOID_EXPORT void Link::setInfo(const std::string& key, const double& value);
template<> CNOID_EXPORT void Link::setInfo(const std::string& key, const bool& value);

}
	
#endif
