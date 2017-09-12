#ifndef CNOID_AGXDYNAMICS_PLUGIN_AGX_BODY_H
#define CNOID_AGXDYNAMICS_PLUGIN_AGX_BODY_H

#include <cnoid/SimulatorItem>
#include <cnoid/BodyItem>
#include <cnoid/MeshExtractor>
#include <cnoid/BasicSensorSimulationHelper>
#include "AGXObjectFactory.h"
#include "AGXBodyPart.h"

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

static unsigned int generateUID(){
    static unsigned int i = 0;
    i++;
    return i;
}

class AGXLink : public Referenced
{
public:
    AGXLink(const LinkPtr link);
    AGXLink(const LinkPtr link, const AGXLinkPtr parent, const Vector3& parentOrigin, const AGXBodyPtr agxBody);
    void constructAGXLink();
    void setCollision(const bool& bOn);
    void setControlInputToAGX();
    void setLinkStateToAGX();
    void setLinkStateToCnoid();
    int getIndex() const;
    Vector3    getOrigin() const;
    LinkPtr    getOrgLink() const;
    AGXLinkPtr getAGXParentLink() const;
    agx::RigidBodyRef       getAGXRigidBody() const;
    agxCollide::GeometryRef getAGXGeometry() const;
    void                    setAGXConstraint(agx::ConstraintRef const constraint);
    agx::ConstraintRef      getAGXConstraint() const;
    std::string             getSelfCollisionGroupName() const;

private:
    LinkPtr     _orgLink;
    AGXLinkPtr  _agxParentLink;
    Vector3     _origin;
    agx::RigidBodyRef       _rigid;
    agxCollide::GeometryRef _geometry;
    agx::ConstraintRef      _constraint;
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
    std::string getSelfCollisionGroupName() const;
    void setCollision(const bool& bOn);
    void setAGXMaterial(const int& index, const agx::MaterialRef& mat);
    void setControlInputToAGX();
    void setLinkStateToAGX();
    void setLinkStateToCnoid();
    bool hasForceSensors() const;
    bool hasGyroOrAccelerationSensors() const;
    void setSensor(const double& timeStep, const Vector3& gravity);
    void updateForceSensors();
    void updateGyroAndAccelerationSensors();
    int  numAGXLinks() const;
    void addAGXLink(AGXLinkPtr const agxLink);
    AGXLinkPtr getAGXLink(const int& index) const;
    AGXLinkPtr getAGXLink(const std::string& name) const;
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
    BasicSensorSimulationHelper sensorHelper;
    bool getAGXLinksFromInfo(const std::string& key, const bool& defaultValue, AGXLinkPtrs& agxLinks) const;
    void createExtraJoint();
    void createContinuousTrack();
    void createAGXVehicleContinousTrack();
};

//class AGXBodyPart: public Referenced
//{
//public:
//    AGXBodyPart();
//    bool hasSelfCollisionGroupName() const;
//    std::string getSelfCollisionGroupName() const ;
//    int numAGXConstraints() const;
//    agx::ConstraintRef getAGXConstraint(const int& index) const;
//protected:
//    void setSelfCollsionGroupName(const std::string& name);
//    void addAGXConstraint(agx::ConstraintRef const constraint);
//private:
//    bool _hasSelfCollisionGroupName;
//    std::string _selfCollisionGroupName;
//    std::vector<agx::ConstraintRef> _constraints;
//};
//
//class AGXExtraJoint : public AGXBodyPart{
//public:
//    AGXExtraJoint(AGXBodyPtr agxBody);
//    void createJoints(AGXBodyPtr agxBody);
//};
//
//class AGXContinousTrack : public AGXBodyPart{
//public:
//    AGXContinousTrack(AGXLinkPtr footLinkStart, AGXBodyPtr body);
//private:
//    AGXLinkPtr  _chassisLink;
//    AGXLinkPtrs _feet;
//    void addFoot(LinkPtr link, AGXBodyPtr body);
//    void createTrackConstraint();
//};

}

#endif