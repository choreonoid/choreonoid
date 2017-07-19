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
    AGXLinkBodyRef getAGXLinkBody();
    agx::RigidBodyRef  getAGXRigidBody();
    agx::ConstraintRef getAGXConstraint();
    void createLinkBody();
    void createConstraints();
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
    AGXLinkBodyRef _agxLinkBody;
    ControlMode _controlMode;
    void createAGXRigidBody();
    void createAGXGeometry();
    void createAGXShape();
    void detectPrimitiveShape(MeshExtractor* extractor, AGXTrimeshDesc& td);
    void createAGXConstraints();
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
    void setCollision(bool bOn);
    void setTorqueToAGX();
    void setControlInputToAGX();
    void setLinkStateToAGX();
    void setLinkStateToCnoid();
    agx::RigidBodyRef getAGXRigidBody(int index);
    agx::ConstraintRef getAGXConstraint(int index);
    void setAGXMaterial(const int& index, const agx::MaterialRef mat);
    int getNumLinks() const;
    void addAGXLink(AGXLinkPtr const agxLink);
private:
    AGXLinkPtrs agxLinks;
};


}

#endif