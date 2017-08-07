#ifndef CNOID_AGXDYNAMICS_PLUGIN_AGX_BODY_H
#define CNOID_AGXDYNAMICS_PLUGIN_AGX_BODY_H

#include <cnoid/SimulatorItem>
#include "AGXObjectFactory.h"
#include <cnoid/BodyItem>
#include <cnoid/MeshExtractor>

namespace cnoid{

typedef Eigen::Matrix<float, 3, 1> Vertex;
class AGXBodyPart;
typedef ref_ptr<AGXBodyPart> AGXBodyPartPtr;
typedef std::vector<AGXBodyPartPtr> AGXBodyPartPtrs;
class AGXBody;
typedef ref_ptr<AGXBody> AGXBodyPtr;
class AGXLink;
typedef ref_ptr<AGXLink> AGXLinkPtr;
typedef std::vector<AGXLinkPtr> AGXLinkPtrs;
class AGXContinousTrack;
typedef ref_ptr<AGXContinousTrack> AGXContinousTrackPtr;
typedef std::vector<AGXContinousTrackPtr> AGXContinousTrackPtrs;

static unsigned int generateUID(){
    static unsigned int i = 0;
    i++;
    return i;
}

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
    Vector3    getOrigin() const;
    LinkPtr    getOrgLink() const;
    AGXLinkPtr getAGXParentLink() const;
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
    int numAGXBodyParts() const;
    void addAGXBodyPart(AGXBodyPartPtr const bp);
    AGXBodyPartPtr getAGXBodyPart(const int& index) const;
private:
    std::string _selfCollisionGroupName;
    AGXLinkPtrs _agxLinks;
    AGXLinkPtrs _controllableLinks;
    AGXBodyPartPtrs _agxBodyParts;
    bool  findAGXLinksFromInfo(const std::string& key, const bool& defaultValue, AGXLinkPtrs& agxLinks) const;
public:
    void createExtraJoint();
    void createContinuousTrack();
};

class AGXBodyPart: public Referenced
{
public:
    AGXBodyPart();
    bool hasSelfCollisionGroupName() const;
    std::string getSelfCollisionGroupName() const ;
    int numAGXConstraints() const;
    agx::ConstraintRef getAGXConstraint(const int& index) const;
protected:
    void setSelfCollsionGroupName(const std::string& name);
    void addAGXConstraint(agx::ConstraintRef const constraint);
private:
    bool _hasSelfCollisionGroupName;
    std::string _selfCollisionGroupName;
    std::vector<agx::ConstraintRef> _constraints;
};

class AGXExtraJoint : public AGXBodyPart{
public:
    AGXExtraJoint(AGXBodyPtr agxBody);
    void createJoints(AGXBodyPtr agxBody);
};

class AGXContinousTrack : public AGXBodyPart{
public:
    AGXContinousTrack(AGXLinkPtr footLinkStart, AGXBodyPtr body);
private:
    AGXLinkPtr  _chassisLink;
    AGXLinkPtrs _feet;
    void addFoot(LinkPtr link, AGXBodyPtr body);
    void createTrackConstraint();
};

}

#endif