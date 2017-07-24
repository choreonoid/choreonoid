#ifndef CNOID_AGXDYNAMICS_PLUGIN_AGX_BODY_H
#define CNOID_AGXDYNAMICS_PLUGIN_AGX_BODY_H

#include <cnoid/SimulatorItem>
#include "AGXLinkBody.h"
#include <cnoid/BodyItem>
#include <cnoid/MeshExtractor>

namespace cnoid{

class AGXBody;
typedef ref_ptr<AGXBody> AGXBodyPtr;
class AGXLink;
typedef ref_ptr<AGXLink> AGXLinkPtr;
typedef std::vector<AGXLinkPtr> AGXLinkPtrs;
typedef Eigen::Matrix<float, 3, 1> Vertex;

class AGXLink : public Referenced
{
public:
    enum ControlMode{
        NONE,
        TORQUE,
        VELOCITY,
        POSITION
    };
    AGXLink(const LinkPtr link);
    AGXLink(const LinkPtr link, const AGXLinkPtr parent, const Vector3& parentOrigin, const AGXBodyPtr agxBody);
    void setParentLink(const AGXLinkPtr link);
    int getIndex() const;
    //AGXLinkBodyRef getAGXLinkBody();
    agx::RigidBodyRef  getAGXRigidBody();
     agxCollide::GeometryRef getAGXGeometry();
    agx::ConstraintRef getAGXConstraint();
    //void createLinkBody();
    void constructAGXLink();
    agx::ConstraintRef createAGXConstraint();
    void setCollision(const bool bOn);
    void setControlInputToAGX();
    void setTorqueToAGX();
    void setVelocityToAGX();
    void setPositionToAGX();
    void setLinkStateToAGX();
    void setLinkStateToCnoid();
    void setJointControlMode(const ControlMode& mode);
    AGXLink::ControlMode getJointControlMode() const;
private:
    Vector3 _origin;
    LinkPtr _orgLink;
    AGXLinkPtr _agxParentLink;
    //AGXLinkBodyRef _agxLinkBody;
    agx::RigidBodyRef _rigid;
    agxCollide::GeometryRef _geometry;
    agx::ConstraintRef _constraint;
    ControlMode _controlMode;
    agx::RigidBodyRef createAGXRigidBody();

    agxCollide::GeometryRef createAGXGeometry();
    void createAGXShape();
    void detectPrimitiveShape(MeshExtractor* extractor, AGXTrimeshDesc& td);
    Vector3 getOrigin() const;
    LinkPtr getOrgLink() const;
    AGXLinkPtr getAGXParentLink() const;
};

class AGXBody :  public SimulationBody
{
public:
    AGXBody(Body& orgBody);
    void initialize();
    void createBody();
    void createBodyClosedLoop();
    void setExtraJoints();
    void setCollision(bool bOn);
    void setTorqueToAGX();
    void setControlInputToAGX();
    void setLinkStateToAGX();
    void setLinkStateToCnoid();
    agx::RigidBodyRef getAGXRigidBody(int index);
    agx::ConstraintRef getAGXConstraint(int index);
    agx::ConstraintRef getAGXExtraConstraint(int index);
    void setAGXMaterial(const int& index, const agx::MaterialRef mat);
    int getNumLinks() const;
    void addAGXLink(AGXLinkPtr const agxLink);
private:
    AGXLinkPtrs agxLinks;
    std::vector<agx::ConstraintRef> _agxExtraConstraints;
    void addAGXExtraConstraint(agx::ConstraintRef constraint);
};


}

#endif