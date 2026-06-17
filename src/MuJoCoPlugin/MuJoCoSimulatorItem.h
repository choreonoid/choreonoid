#ifndef CNOID_MUJOCO_PLUGIN_MUJOCO_SIMULATOR_ITEM_H
#define CNOID_MUJOCO_PLUGIN_MUJOCO_SIMULATOR_ITEM_H

#include <cnoid/SimulatorItem>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MuJoCoSimulatorItem : public SimulatorItem
{
public:
    static void initialize(ExtensionManager* ext);

    MuJoCoSimulatorItem();
    MuJoCoSimulatorItem(const MuJoCoSimulatorItem& org);
    virtual ~MuJoCoSimulatorItem();

    void setGravity(const Vector3& gravity);
    const Vector3& gravity() const;
    virtual Vector3 getGravity() const override;

    class Impl;

protected:
    virtual SimulationBody* createSimulationBody(Body* orgBody, CloneMap& cloneMap) override;
    virtual bool initializeSimulation(const std::vector<SimulationBody*>& simBodies) override;
    virtual bool stepSimulation(const std::vector<SimulationBody*>& activeSimBodies) override;
    virtual void finalizeSimulation() override;

    virtual Item* doDuplicate() const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    Impl* impl;
};

typedef ref_ptr<MuJoCoSimulatorItem> MuJoCoSimulatorItemPtr;

}

#endif
