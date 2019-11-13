#ifndef CNOID_BODYPLUGIN_KINEMATIC_SIMULATOR_ITEM_H
#define CNOID_BODYPLUGIN_KINEMATIC_SIMULATOR_ITEM_H

#include "SimulatorItem.h"

namespace cnoid {

class KinematicSimulatorItem : public SimulatorItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    KinematicSimulatorItem();
    KinematicSimulatorItem(const KinematicSimulatorItem& org);
    virtual ~KinematicSimulatorItem();
    virtual Item* doDuplicate() const override;

protected:
    virtual void clearSimulation() override;
    virtual SimulationBody* createSimulationBody(Body* orgBody, CloneMap& cloneMap) override;
    virtual bool initializeSimulation(const std::vector<SimulationBody*>& simBodies) override;
    virtual bool stepSimulation(const std::vector<SimulationBody*>& activeSimBodies) override;
    virtual void finalizeSimulation() override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<KinematicSimulatorItem> KinematicSimulatorItemPtr;

}

#endif
