#ifndef CNOID_AGXDYNAMICS_PLUGIN_AGX_BODY_H
#define CNOID_AGXDYNAMICS_PLUGIN_AGX_BODY_H

#include <cnoid/SimulatorItem>
#include <cnoid/BodyItem>
#include <cnoid/MeshExtractor>
#include <cnoid/BasicSensorSimulationHelper>
#include "AGXObjectFactory.h"
#include "AGXBodyExtension.h"
#include "exportdecl.h"

namespace{
typedef std::function<bool(cnoid::AGXBody* agxBody)> AGXBodyExtensionFunc;
typedef std::map<std::string, AGXBodyExtensionFunc> AGXBodyExtensionFuncMap;
};

namespace cnoid{

typedef Eigen::Matrix<float, 3, 1> Vertex;
class AGXBody;
typedef ref_ptr<AGXBody> AGXBodyPtr;
class AGXLink;
typedef ref_ptr<AGXLink> AGXLinkPtr;
typedef std::vector<AGXLinkPtr> AGXLinkPtrs;
typedef std::vector<AGXBodyExtensionPtr> AGXBodyExtensionPtrs;
typedef std::vector<std::string> VectorString;
class AGXScene;

static unsigned int generateUID(){
    static unsigned int i = 0;
    i++;
    return i;
}

inline const Position convertToPosition(const agx::AffineMatrix4x4& a){
    Position pos;
    pos.translation() = Vector3(a(3,0), a(3,1), a(3,2));
    pos.linear() << a(0,0), a(1,0), a(2,0),
                    a(0,1), a(1,1), a(2,1),
                    a(0,2), a(1,2), a(2,2);
    return pos;
}

//struct AGXLinkPrivate
//{
//    AGXBody*    m_agxBody;
//    //LinkPtr     _orgLink;
//    //AGXLinkPtr  _agxParentLink;
//    //Vector3     _origin;
//    //agx::RigidBodyRef       _rigid;
//    //agxCollide::GeometryRef _geometry;
//    //agx::ConstraintRef      _constraint;
//    AGXBody*                getAGXBody();
//};

class AGXLink : public Referenced
{
public:
    AGXLink(const LinkPtr link);
    AGXLink(const LinkPtr link, const AGXLinkPtr parent, const Vector3& parentOrigin, const AGXBodyPtr agxBody);
    void constructAGXLink();
    void setAGXMaterial();
    bool setAGXMaterialFromName(const std::string& materialName);
    bool setAGXMaterialFromLinkInfo();
    bool setCenterOfMassFromLinkInfo();
    bool setMassFromLinkInfo();
    bool setInertiaFromLinkInfo();
    void enableExternalCollision(const bool& bOn);
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
    void printDebugInfo();

private:
    AGXBody*    _agxBody;
    LinkPtr     _orgLink;
    AGXLinkPtr  _agxParentLink;
    Vector3     _origin;
    agx::RigidBodyRef       _rigid;
    agxCollide::GeometryRef _geometry;
    agx::ConstraintRef      _constraint;
    AGXBody*                getAGXBody();
    agx::RigidBodyRef       createAGXRigidBody();
    agxCollide::GeometryRef createAGXGeometry();
    void createAGXShape();
    void detectPrimitiveShape(MeshExtractor* extractor, AGXTrimeshDesc& td);
    agx::ConstraintRef createAGXConstraint();
    void setTorqueToAGX();
    void setVelocityToAGX();
    void setPositionToAGX();
};

class CNOID_EXPORT AGXBody :  public SimulationBody
{
public:
    AGXBody(Body& orgBody);
    void initialize();
    void createBody(AGXScene* agxScene);
    std::string getCollisionGroupName() const;
    void enableExternalCollision(const bool& bOn);
    void addCollisionGroupNameToDisableCollision(const std::string& name);
    const VectorString& getCollisionGroupNamesToDisableCollision() const;
    void setAGXMaterial(const int& index, const agx::MaterialRef& mat);
    void setControlInputToAGX();
    void setLinkStateToAGX();
    void setLinkStateToCnoid();
    bool hasForceSensors() const;
    bool hasGyroOrAccelerationSensors() const;
    void setSensor(const double& timeStep, const Vector3& gravity);
    void updateForceSensors();
    void updateGyroAndAccelerationSensors();
    AGXScene* getAGXScene() const;
    int  numAGXLinks() const;
    void addAGXLink(AGXLinkPtr const agxLink);
    AGXLinkPtr getAGXLink(const int& index) const;
    AGXLinkPtr getAGXLink(const std::string& name) const;
    int numControllableLinks() const;
    void addControllableLink(AGXLinkPtr const agxLink);
    AGXLinkPtr getControllableLink(const int& index) const;
    agx::RigidBodyRef  getAGXRigidBody(const int& index) const;
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
    bool getAGXLinksFromInfo(const std::string& key, const bool& defaultValue, AGXLinkPtrs& agxLinks) const;
    void createExtraJoint();
    bool createContinuousTrack(AGXBody* agxBody);
    bool createAGXVehicleContinousTrack(AGXBody* agxBody);
};

}

#endif