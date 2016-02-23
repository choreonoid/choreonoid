/*!
  @file
  @author Shizuko Hattori
*/
#ifndef CNOID_AGXPLUGIN_AGX_SIMULATOR_ITEM_H_INCLUDED
#define CNOID_AGXPLUGIN_AGX_SIMULATOR_ITEM_H_INCLUDED

#include <cnoid/SimulatorItem>
#include "exportdecl.h"

namespace cnoid {

class AgXSimulatorItemImpl;

class CNOID_EXPORT AgXSimulatorItem : public SimulatorItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    AgXSimulatorItem();
    AgXSimulatorItem(const AgXSimulatorItem& org);
    virtual ~AgXSimulatorItem();

    enum DynamicsMode { FORWARD_DYNAMICS = 0, HG_DYNAMICS, N_DYNAMICS_MODES };
    enum ControlMode { DEFAULT=0, HIGH_GAIN, TORQUE, FREE };
    void setJointControlMode(Link* joint, ControlMode type);
    void setJointCompliance(Link* joint, double spring, double damping);

protected:

    virtual SimulationBody* createSimulationBody(Body* orgBody);
    virtual bool initializeSimulation(const std::vector<SimulationBody*>& simBodies);
    virtual void initializeSimulationThread();
    virtual void finalizeSimulationThread();
    virtual bool stepSimulation(const std::vector<SimulationBody*>& activeSimBodies);
    virtual void finalizeSimulation();
    virtual CollisionLinkPairListPtr getCollisions();
    virtual ControllerItem* createBodyMotionController(BodyItem* bodyItem, BodyMotionItem* bodyMotionItem);

    virtual Item* doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

private:
    AgXSimulatorItemImpl* impl;
    friend class AgXSimulatorItemImpl;
};

typedef ref_ptr<AgXSimulatorItem> AgXSimulatorItemPtr;
}

#endif
