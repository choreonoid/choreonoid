/*!
  @file
  @author Yuichi Tazaki
*/

#ifndef CNOID_SPRINGHEADPLUGIN_SPRINGHEAD_SIMULATOR_ITEM_H
#define CNOID_SPRINGHEADPLUGIN_SPRINGHEAD_SIMULATOR_ITEM_H

#include <cnoid/SimulatorItem>
#include <cnoid/EigenTypes>
#include "exportdecl.h"

namespace cnoid {

class SpringheadSimulatorItemImpl;
        
class CNOID_EXPORT SpringheadSimulatorItem : public SimulatorItem
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    SpringheadSimulatorItem();
    SpringheadSimulatorItem(const SpringheadSimulatorItem& org);
    virtual ~SpringheadSimulatorItem();

    void setGravity               (const Vector3& gravity);
    void setStaticFriction        (double mu0);
	void setDynamicFriction       (double mu);
	void setBouncingFactor        (double e);
	void setContactSpring         (double K);
	void setContactDamper         (double D);
    void setNumIterations         (int n);
    void useWorldCollisionDetector(bool on);

protected:
        
    virtual SimulationBody* createSimulationBody(Body* orgBody);
    virtual bool initializeSimulation(const std::vector<SimulationBody*>& simBodies);
    virtual void initializeSimulationThread();
    virtual bool stepSimulation(const std::vector<SimulationBody*>& activeSimBodies);
    virtual void finalizeSimulation();

    virtual Item* doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

private:
    SpringheadSimulatorItemImpl* impl;
    friend class SpringheadSimulatorItemImpl;
};

typedef ref_ptr<SpringheadSimulatorItem> SpringheadSimulatorItemPtr;
}

#endif
