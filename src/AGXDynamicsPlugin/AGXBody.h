#ifndef CNOID_AGXDYNAMICS_PLUGIN_AGX_BODY_H
#define CNOID_AGXDYNAMICS_PLUGIN_AGX_BODY_H

#include <cnoid/SimulatorItem>
#include <cnoid/BodyItem>
#include <cnoid/BasicSensorSimulationHelper>
#include "AGXObjectFactory.h"
#include "AGXBodyExtension.h"
#include <set>
#include "exportdecl.h"

namespace{
typedef std::function<bool(cnoid::AGXBody* agxBody)> AGXBodyExtensionFunc;
typedef std::map<std::string, AGXBodyExtensionFunc> AGXBodyExtensionFuncMap;
}

namespace cnoid {

inline const Position convertToPosition(const agx::AffineMatrix4x4& a){
    Position pos;
    pos.translation() = Vector3(a(3,0), a(3,1), a(3,2));
    pos.linear() << a(0,0), a(1,0), a(2,0),
                    a(0,1), a(1,1), a(2,1),
                    a(0,2), a(1,2), a(2,2);
    return pos;
}

class MeshExtractor;
class AGXScene;
class AGXBody;
class CNOID_EXPORT AGXLink : public Referenced
{
public:
    AGXLink(Link* const link);
    AGXLink(Link* const link, AGXLink* const parent, const Vector3& parentOrigin, AGXBody* const agxBody, std::set<Link*>& forceSensorLinks, bool makeStatic);
    void constructAGXLink(const bool& makeStatic);
    void setAGXMaterial();
    bool setAGXMaterialFromName(const std::string& materialName);
    void setAGXMaterialFromLinkInfo();
    bool setCenterOfMassFromLinkInfo();
    bool setMassFromLinkInfo();
    bool setInertiaFromLinkInfo();
    void enableExternalCollision(const bool& bOn);
    void setControlInputToAGX();
    void addForceTorqueToAGX();
    void setLinkStateToAGX();
    void setLinkStateToCnoid();
    int getIndex() const;
    Vector3    getOrigin() const;
    Link*      getOrgLink() const;
    AGXLink*   getAGXParentLink() const;
    agx::RigidBody*         getAGXRigidBody() const;
    agxCollide::Geometry*   getAGXGeometry() const;
    void                    setAGXConstraint(agx::Constraint* const constraint);
    agx::Constraint*        getAGXConstraint() const;
    agx::Name               getCollisionGroupName() const;
    void printDebugInfo();

private:
    AGXBody* _agxBody;
    Link*    _orgLink;
    AGXLink* _agxParentLink;
    Vector3     _origin;
    agx::RigidBodyRef       _rigid;
    agxCollide::GeometryRef _geometry;
    agx::ConstraintRef      _constraint;
    agx::Name               _collisionGroupName;
    AGXBody*                getAGXBody();
    agx::RigidBodyRef       createAGXRigidBody();
    agxCollide::GeometryRef createAGXGeometry();
    void createAGXShape();
    void detectPrimitiveShape(MeshExtractor* extractor, AGXTrimeshDesc& td);
    agx::ConstraintRef createAGXConstraint();
    void setTorqueToAGX();
    void setVelocityToAGX();
    void setPositionToAGX();
    void setLinkPositionToAGX();
};
typedef ref_ptr<AGXLink> AGXLinkPtr;
typedef std::vector<AGXLinkPtr> AGXLinkPtrs;

class CNOID_EXPORT AGXBody :  public SimulationBody
{
public:
    AGXBody(Body* body);
    void initialize();
    void createBody(AGXScene* agxScene);
    void setCollision();
    void setCollisionExclude();
    void setCollisionExcludeLinks(const Mapping& cdMapping);
    void setCollisionExcludeLinksDynamic(const Mapping& cdMapping);
    void setCollisionExcludeTreeDepth(const Mapping& cdMapping);
    void setCollisionExcludeLinkGroups(const Mapping& cdMapping);
    void setCollisionExcludeSelfCollisionLinks(const Mapping& cdMapping);
    void setCollisionExcludeLinksWireCollision(const Mapping& cdMapping);
    std::string getCollisionGroupName() const;
    void enableExternalCollision(const bool& bOn);
    void enableAGXWireContact(const bool& bOn);
    void addCollisionGroupNameToDisableCollision(const std::string& name);
    const std::vector<std::string>& getCollisionGroupNamesToDisableCollision() const;
    void addCollisionGroupNameToAllLink(const std::string& name);
    void setAGXMaterial(const int& index, agx::Material* const mat);
    void setControlInputToAGX();
    void addForceTorqueToAGX();
    void setLinkStateToAGX();
    void setLinkStateToCnoid();
    bool hasForceSensors() const;
    bool hasGyroOrAccelerationSensors() const;
    void setSensor(const double& timeStep, const Vector3& gravity);
    void updateForceSensors();
    void updateGyroAndAccelerationSensors();
    AGXScene* getAGXScene() const;
    int  numAGXLinks() const;
    void addAGXLink(AGXLink* const agxLink);
    AGXLink* getAGXLink(const int& index) const;
    AGXLink* getAGXLink(const std::string& name) const;
    const AGXLinkPtrs& getAGXLinks() const;
    bool getAGXLinksFromInfo(const std::string& key, const bool& defaultValue, AGXLinkPtrs& agxLinks) const;
    int numControllableLinks() const;
    void addControllableLink(AGXLink* const agxLink);
    AGXLink* getControllableLink(const int& index) const;
    const AGXLinkPtrs& getControllableLinks() const;
    agx::RigidBodyRef  getAGXRigidBody(const int& index) const;
    agx::RigidBody*    getAGXRigidBody(const std::string& linkName) const;
    agx::ConstraintRef getAGXConstraint(const int& index) const;
    bool addAGXBodyExtension(AGXBodyExtension* const extension);
    const AGXBodyExtensionPtrs& getAGXBodyExtensions() const;
    void callExtensionFuncs();
    static void addAGXBodyExtensionAdditionalFunc(const std::string& typeName,
        std::function<bool(AGXBody* agxBody)> func);
    void updateAGXBodyExtensionFuncs();
private:
    std::string _bodyCollisionGroupName;
    std::vector<std::string> _collisionGroupNamesToDisableCollision;
    AGXScene* _agxScene;
    AGXLinkPtrs _agxLinks;
    AGXLinkPtrs _controllableLinks;
    AGXBodyExtensionPtrs _agxBodyExtensions;
    BasicSensorSimulationHelper sensorHelper;
    AGXBodyExtensionFuncMap agxBodyExtensionFuncs;
    void createExtraJoint();
};
typedef ref_ptr<AGXBody> AGXBodyPtr;

}

#endif
