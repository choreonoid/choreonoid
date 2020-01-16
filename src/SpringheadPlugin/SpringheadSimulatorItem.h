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
    virtual Item*           doDuplicate         ()                                              const override;
    virtual void            doPutProperties     (PutPropertyFunction& putProperty)                    override;
    virtual bool            store               (Archive& archive)                                    override;
    virtual bool            restore             (const Archive& archive)                              override;
	virtual SimulationBody* createSimulationBody(Body* orgBody)                                       override;
    virtual bool            initializeSimulation(const std::vector<SimulationBody*>& simBodies)       override;
    virtual bool            stepSimulation      (const std::vector<SimulationBody*>& activeSimBodies) override;
	
private:
    SpringheadSimulatorItemImpl* impl;
    friend class SpringheadSimulatorItemImpl;
};

typedef ref_ptr<SpringheadSimulatorItem> SpringheadSimulatorItemPtr;
}

#endif
