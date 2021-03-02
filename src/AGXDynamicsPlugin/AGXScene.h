#ifndef CNOID_AGXDYNAMICS_PLUGIN_AGX_SCENE_H
#define CNOID_AGXDYNAMICS_PLUGIN_AGX_SCENE_H

#include "AGXObjectFactory.h"
#include "exportdecl.h"

namespace cnoid{

struct AGXSceneDesc
{
    AGXSceneDesc(){}
    AGXSimulationDesc simdesc;
};

class CNOID_EXPORT AGXScene : public agx::Referenced
{
public:
    AGXScene(const AGXSceneDesc& desc);
    static AGXScene* create(const AGXSceneDesc& desc);
    void clear();
    void setMainWorkThread();
    void stepSimulation();
    agxSDK::SimulationRef getSimulation() const;
    agx::Bool add(agx::RigidBody* const rigid);
    agx::Bool add(agx::Constraint* const constraint);
    agx::Bool add(agxSDK::Assembly* const assembly);
    agx::MaterialRef getMaterial(const agx::String& materialName);
    agx::MaterialRef createMaterial(const AGXMaterialDesc& desc);
    agx::ContactMaterialRef createContactmaterial(agx::MaterialRef const matA, agx::MaterialRef const matB, const AGXContactMaterialDesc& desc);
    agx::ContactMaterialRef createContactMaterial(const AGXContactMaterialDesc& desc);
    void printMaterials();
    void printContactMaterialTable();
    void setCollision(const agx::Name& name, bool bOn);
    void setCollisionPair(const unsigned& id1, const unsigned& id2, bool bOn);
    void setCollisionPair(const agx::Name& name1, const agx::Name& name2, bool bOn);
    agx::Vec3 getGravity() const;
    void setGravity(const agx::Vec3& g);
    bool getEnableAutoSleep() const;
    void setEnableAutoSleep(const bool& bOn);
    bool saveSceneToAGXFile();

private:
    agxSDK::SimulationRef _agxSimulation;
};
typedef agx::ref_ptr<AGXScene> AGXSceneRef;

}
#endif
