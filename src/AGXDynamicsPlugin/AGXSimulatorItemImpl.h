#ifndef CNOID_AGXDYNAMICS_PLUGIN_AGX_SIMULATOR_ITEM_IMPL_H
#define CNOID_AGXDYNAMICS_PLUGIN_AGX_SIMULATOR_ITEM_IMPL_H

#include <cnoid/SimulatorItem>
#include "AGXScene.h"
#include "AGXBody.h"
#include <iostream>

namespace cnoid {

class AGXSimulatorItem;
typedef ref_ptr<AGXSimulatorItem> AGXSimulatorItemPtr;

class AGXSimulatorItemImpl : public Referenced
{
public:
    AGXSimulatorItemPtr self;

    AGXSimulatorItemImpl(AGXSimulatorItemPtr self);
    AGXSimulatorItemImpl(AGXSimulatorItemPtr self, const AGXSimulatorItemImpl& org);
    ~AGXSimulatorItemImpl();
    // call from defualt constructer
    void initialize();
    // add parameters to property panel
    void doPutProperties(PutPropertyFunction& putProperty);
    // save simulation parameter to cnoid file
    bool store(Archive& archive);
    // store simulation parameter from cnoid file
    bool restore(const Archive& archive);

    // Function of create, step simulation
    SimulationBody* createSimulationBody(Body* orgBody);
    bool initializeSimulation(const std::vector<SimulationBody*>& simBodies);
    void createMaterialTable();
    bool stepSimulation(const std::vector<SimulationBody*>& activeSimBodies);
    void stopSimulation();
    void pauseSimulation();
    void restartSimulation();

    void setGravity(const Vector3& g);
    Vector3 getGravity() const;
    bool saveSimulationToAGXFile();

private:
    AGXSceneRef agxScene = nullptr;
    Vector3 _p_gravity;
    int     _p_numThreads;
    bool    _p_enableContactReduction;
    int     _p_contactReductionBinResolution;
    int     _p_contactReductionThreshhold;
    bool    _p_enableAutoSleep;
};
}
#endif