#ifndef CNOID_AGXDYNAMICS_PLUGIN_AGX_SIMULATOR_ITEM_IMPL_H
#define CNOID_AGXDYNAMICS_PLUGIN_AGX_SIMULATOR_ITEM_IMPL_H

#include <cnoid/SimulatorItem>
//#include "AGXInclude.h"
#include "AGXScene.h"
#include "AGXBody.h"
#include <iostream>

namespace cnoid {

const double DEFAULT_GRAVITY_ACCELERATION = 9.80665;

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
    bool stepSimulation(const std::vector<SimulationBody*>& activeSimBodies);
    void stopSimulation();
    void pauseSimulation();
    void restartSimulation();

    void setGravity(const Vector3& gravity);
    Vector3 getGravity() const;
    bool saveSimulationToAGXFile();


private:
    AGXSceneRef agxScene = nullptr;
    Vector3 _gravity;
};
}
#endif