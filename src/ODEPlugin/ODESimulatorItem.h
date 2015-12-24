/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_ODEPLUGIN_ODE_SIMULATOR_ITEM_H
#define CNOID_ODEPLUGIN_ODE_SIMULATOR_ITEM_H

#include <cnoid/SimulatorItem>
#include <cnoid/EigenTypes>
#include "exportdecl.h"
#ifdef GAZEBO_ODE
#define ODESimulatorItem GazeboODESimulatorItem
#endif

namespace cnoid {

class ODESimulatorItemImpl;
        
class CNOID_EXPORT ODESimulatorItem : public SimulatorItem
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    ODESimulatorItem();
    ODESimulatorItem(const ODESimulatorItem& org);
    virtual ~ODESimulatorItem();

    virtual void setAllLinkPositionOutputMode(bool on);

    enum StepMode { STEP_ITERATIVE, STEP_BIG_MATRIX, NUM_STEP_MODES };

    void setStepMode(int value);
    void setGravity(const Vector3& gravity);
    void setFriction(double friction);
    void setJointLimitMode(bool on);
    void set2Dmode(bool on);
    void setGlobalERP(double erp);
    void setGlobalCFM(double value);
    void setNumIterations(int n);
    void setOverRelaxation(double value);
    void setCorrectingVelocityLimitMode(bool on);
    void setMaxCorrectingVelocity(double vel);
    void setSurfaceLayerDepth(double value);
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
    ODESimulatorItemImpl* impl;
    friend class ODESimulatorItemImpl;
};

typedef ref_ptr<ODESimulatorItem> ODESimulatorItemPtr;
}

#endif
