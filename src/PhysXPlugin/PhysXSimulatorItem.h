/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_PHYSXPLUGIN_PHYSX_SIMULATOR_ITEM_H_INCLUDED
#define CNOID_PHYSXPLUGIN_PHYSX_SIMULATOR_ITEM_H_INCLUDED

#include <cnoid/SimulatorItem>

namespace cnoid {

class PhysXSimulatorItemImpl;
        
class PhysXSimulatorItem : public SimulatorItem
{
public:

    static void initialize(ExtensionManager* ext);
    PhysXSimulatorItem();
    PhysXSimulatorItem(const PhysXSimulatorItem& org);
    virtual ~PhysXSimulatorItem();

    virtual void setAllLinkPositionOutputMode(bool on);

protected:
        
    virtual SimulationBody* createSimulationBody(Body* orgBody);
    virtual bool initializeSimulation(const std::vector<SimulationBody*>& simBodies);
    virtual void initializeSimulationThread();
    virtual bool stepSimulation(const std::vector<SimulationBody*>& activeSimBodies);
        
    virtual Item* doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

private:
    PhysXSimulatorItemImpl* impl;
    friend class PhysXSimulatorItemImpl;
};

typedef ref_ptr<PhysXSimulatorItem> PhysXSimulatorItemPtr;
}

#endif
