#ifndef CNOID_BODY_PLUGIN_KINEMATIC_SIMULATOR_ITEM_H
#define CNOID_BODY_PLUGIN_KINEMATIC_SIMULATOR_ITEM_H

#include "SimulatorItem.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT KinematicSimulatorItem : public SimulatorItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    KinematicSimulatorItem();
    virtual ~KinematicSimulatorItem();

protected:
    KinematicSimulatorItem(const KinematicSimulatorItem& org);
    virtual Item* doCloneItem(CloneMap* cloneMap) const override;
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
