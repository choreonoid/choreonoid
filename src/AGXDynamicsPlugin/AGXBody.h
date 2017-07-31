#ifndef CNOID_AGXDYNAMICS_PLUGIN_AGX_BODY_H
#define CNOID_AGXDYNAMICS_PLUGIN_AGX_BODY_H

#include <cnoid/SimulatorItem>
#include "AGXObjectFactory.h"
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
    void constructAGXLink();
    void setCollision(const bool& bOn);
    void setControlInputToAGX();
    void setLinkStateToAGX();
    void setLinkStateToCnoid();
    void setJointControlMode(const ControlMode& mode);
    int getIndex() const;
    agx::RigidBodyRef       getAGXRigidBody() const;
    agxCollide::GeometryRef getAGXGeometry() const;
    agx::ConstraintRef      getAGXConstraint() const;
    AGXLink::ControlMode    getJointControlMode() const;
    std::string             getSelfCollisionGroupName() const;

private:
    LinkPtr     _orgLink;
    AGXLinkPtr  _agxParentLink;
    Vector3     _origin;
    agx::RigidBodyRef       _rigid;
    agxCollide::GeometryRef _geometry;
    agx::ConstraintRef      _constraint;
    ControlMode             _controlMode;
    std::string             _selfCollisionGroupName;

    LinkPtr    getOrgLink() const;
    AGXLinkPtr getAGXParentLink() const;
    Vector3    getOrigin() const;
    agx::RigidBodyRef       createAGXRigidBody();
    agxCollide::GeometryRef createAGXGeometry();
    void createAGXShape();
    void detectPrimitiveShape(MeshExtractor* extractor, AGXTrimeshDesc& td);
    agx::ConstraintRef createAGXConstraint();
    void setTorqueToAGX();
    void setVelocityToAGX();
    void setPositionToAGX();
};

class AGXBody :  public SimulationBody
{
public:
    AGXBody(Body& orgBody);
    void initialize();
    void createBody();
    void createBodyClosedLoop();
    void setExtraJoints();
    std::string getSelfCollisionGroupName() const;
    void setCollision(const bool& bOn);
    void setAGXMaterial(const int& index, const agx::MaterialRef& mat);
    void setControlInputToAGX();
    void setLinkStateToAGX();
    void setLinkStateToCnoid();
    int  numAGXLinks() const;
    void addAGXLink(AGXLinkPtr const agxLink);
    AGXLinkPtr getAGXLink(const int& index) const;
    int numControllableLinks() const;
    void addControllableLink(AGXLinkPtr const agxLink);
    AGXLinkPtr getControllableLink(const int& index) const;
    agx::RigidBodyRef  getAGXRigidBody(const int& index) const;
    agx::ConstraintRef getAGXConstraint(const int& index) const;
    int numAGXExtraConstraints() const;
    agx::ConstraintRef getAGXExtraConstraint(const int& index) const;
private:
    std::string _selfCollisionGroupName;
    AGXLinkPtrs _agxLinks;
    AGXLinkPtrs _controllableLinks;
    std::vector<agx::ConstraintRef> _agxExtraConstraints;
    void addAGXExtraConstraint(agx::ConstraintRef constraint);
    static unsigned int generateUID();
};


}

#endif