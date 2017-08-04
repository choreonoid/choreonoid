#ifndef CNOID_AGXDYNAMICS_PLUGIN_AGX_SCENE_H
#define CNOID_AGXDYNAMICS_PLUGIN_AGX_SCENE_H

#include "AGXObjectFactory.h"

namespace cnoid{

struct AGXSceneDesc
{
    AGXSceneDesc(){}
    AGXSimulationDesc simdesc;
};

class AGXScene;
typedef agx::ref_ptr<AGXScene> AGXSceneRef;
class AGXScene : public agx::Referenced
{
public:
    AGXScene(const AGXSceneDesc& desc);
    static AGXSceneRef create(const AGXSceneDesc& desc);
    void clear();
    void stepSimulation();
    agx::Bool add(agx::RigidBodyRef const rigid);
    agx::Bool add(agx::ConstraintRef const constraint);
    agx::MaterialRef getMaterial(const agx::String& materialName);
    agx::MaterialRef createMaterial(const AGXMaterialDesc& desc);
    agx::ContactMaterialRef createContactmaterial(agx::MaterialRef const matA, agx::MaterialRef const matB, const AGXContactMaterialDesc& desc);
    agx::ContactMaterialRef createContactMaterial(const AGXContactMaterialDesc& desc);
    void setCollisionPair(const unsigned& id1, const unsigned& id2, bool bOn);
    void setCollisionPair(const agx::Name& name1, const agx::Name& name2, bool bOn);
    bool saveSceneToAGXFile();

private:
    agxSDK::SimulationRef _agxSimulation;
    agxSDK::SimulationRef getSimulation() const;
};

}
#endif