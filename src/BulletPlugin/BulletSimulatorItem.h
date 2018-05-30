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
        
    virtual SimulationBody* createSimulationBody(Body* orgBody);
    virtual bool initializeSimulation(const std::vector<SimulationBody*>& simBodies);
    virtual void initializeSimulationThread();
    virtual bool stepSimulation(const std::vector<SimulationBody*>& activeSimBodies);
        
    virtual Item* doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

private:
    BulletSimulatorItemImpl* impl;
    friend class BulletSimulatorItemImpl;
};

typedef ref_ptr<BulletSimulatorItem> BulletSimulatorItemPtr;
}

#endif
