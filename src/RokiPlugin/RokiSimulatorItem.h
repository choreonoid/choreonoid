/*!
  @file
  @author Shizuko Hattori
*/

#ifndef CNOID_ROKIPLUGIN_ROKI_SIMULATOR_ITEM_H_INCLUDED
#define CNOID_ROKIPLUGIN_ROKI_SIMULATOR_ITEM_H_INCLUDED

#include <cnoid/SimulatorItem>
#include <cnoid/EigenTypes>

namespace cnoid {

class RokiSimulatorItemImpl;
        
class RokiSimulatorItem : public SimulatorItem
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    RokiSimulatorItem();
    RokiSimulatorItem(const RokiSimulatorItem& org);
    virtual ~RokiSimulatorItem();

protected:
    virtual SimulationBody* createSimulationBody(Body* orgBody) override;
    virtual bool initializeSimulation(const std::vector<SimulationBody*>& simBodies) override;
    virtual bool stepSimulation(const std::vector<SimulationBody*>& activeSimBodies) override;
    virtual void finalizeSimulation() override;
        
    virtual Item* doDuplicate() const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;
    virtual std::shared_ptr<CollisionLinkPairList> getCollisions() override;

private:
    RokiSimulatorItemImpl* impl;
    friend class RokiSimulatorItemImpl;
};

typedef ref_ptr<RokiSimulatorItem> RokiSimulatorItemPtr;
}

#endif
