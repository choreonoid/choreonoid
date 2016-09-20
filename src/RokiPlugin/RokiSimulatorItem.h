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
    virtual SimulationBody* createSimulationBody(Body* orgBody);
    virtual bool initializeSimulation(const std::vector<SimulationBody*>& simBodies);
    virtual bool stepSimulation(const std::vector<SimulationBody*>& activeSimBodies);
    virtual void finalizeSimulation();
        
    virtual Item* doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);
    virtual CollisionLinkPairListPtr getCollisions();

private:
    RokiSimulatorItemImpl* impl;
    friend class RokiSimulatorItemImpl;
};

typedef ref_ptr<RokiSimulatorItem> RokiSimulatorItemPtr;
}

#endif
