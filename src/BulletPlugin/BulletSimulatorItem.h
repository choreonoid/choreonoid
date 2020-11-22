/*!
  @file
  @author Shizuko Hattori
*/

#ifndef CNOID_BULLET_PLUGIN_BULLET_SIMULATOR_ITEM_H_INCLUDED
#define CNOID_BULLET_PLUGIN_BULLET_SIMULATOR_ITEM_H_INCLUDED

#include <cnoid/SimulatorItem>
#include "exportdecl.h"

namespace cnoid {

class BulletSimulatorItemImpl;
        
class CNOID_EXPORT BulletSimulatorItem : public SimulatorItem
{
public:
    static void initialize(ExtensionManager* ext);
        
    BulletSimulatorItem();
    BulletSimulatorItem(const BulletSimulatorItem& org);
    virtual ~BulletSimulatorItem();
    virtual Vector3 getGravity() const override;

//    virtual void setAllLinkPositionOutputMode(bool on);

protected:
        
    virtual SimulationBody* createSimulationBody(Body* orgBody) override;
    virtual bool initializeSimulation(const std::vector<SimulationBody*>& simBodies) override;
    virtual void initializeSimulationThread() override;
    virtual bool stepSimulation(const std::vector<SimulationBody*>& activeSimBodies) override;
        
    virtual Item* doDuplicate() const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    BulletSimulatorItemImpl* impl;
    friend class BulletSimulatorItemImpl;
};

typedef ref_ptr<BulletSimulatorItem> BulletSimulatorItemPtr;
}

#endif
