#ifndef CNOID_PHYSX_PLUGIN_PHYSX_SIMULATOR_ITEM_H
#define CNOID_PHYSX_PLUGIN_PHYSX_SIMULATOR_ITEM_H

#include <cnoid/SimulatorItem>

namespace cnoid {

class PhysXSimulatorItem : public SimulatorItem
{
public:
    static void initialize(ExtensionManager* ext);

    PhysXSimulatorItem();
    PhysXSimulatorItem(const PhysXSimulatorItem& org);
    virtual ~PhysXSimulatorItem();

    void setNumThreads(int n);

    void setGravity(const Vector3& gravity);
    const Vector3& gravity() const;
    virtual Vector3 getGravity() const override;

    class Impl;

protected:
    virtual SimulationBody* createSimulationBody(Body* orgBody) override;
    virtual bool initializeSimulation(const std::vector<SimulationBody*>& simBodies) override;
    virtual bool stepSimulation(const std::vector<SimulationBody*>& activeSimBodies) override;
        
    virtual Item* doDuplicate() const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    Impl* impl;
};

typedef ref_ptr<PhysXSimulatorItem> PhysXSimulatorItemPtr;
}

#endif
