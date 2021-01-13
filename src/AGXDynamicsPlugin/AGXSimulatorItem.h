#ifndef CNOID_AGXDYNAMICS_PLUGIN_AGX_SIMULATOR_ITEM_H
#define CNOID_AGXDYNAMICS_PLUGIN_AGX_SIMULATOR_ITEM_H

#include <cnoid/SimulatorItem>
#include "exportdecl.h"

namespace cnoid {

class AGXSimulatorItemImpl;
typedef ref_ptr<AGXSimulatorItemImpl> AGXSimulatorItemImplPtr;

class CNOID_EXPORT AGXSimulatorItem : public SimulatorItem
{
public:
    static void initializeClass(ExtensionManager* ext);
    AGXSimulatorItem();
    AGXSimulatorItem(const AGXSimulatorItem& org);
    virtual ~AGXSimulatorItem();
    bool saveSimulationToAGXFile();

    virtual Vector3 getGravity() const override;
    void setNumThreads(unsigned int num);
    void setEnableContactReduction(bool bOn);
    void setContactReductionBinResolution(int r);
    void setContactReductionThreshhold(int t);
    void setEnableContactWarmstarting(bool bOn);
    void setEnableAMOR(bool bOn);
    
protected:
    virtual Item* doDuplicate() const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;
    virtual SimulationBody* createSimulationBody(Body* orgBody) override;
    virtual bool initializeSimulation(const std::vector<SimulationBody*>& simBodies) override;
    //virtual void initializeSimulationThread();
    //virtual void finalizeSimulationThread();
    //virtual bool startSimulation(bool doReset = true);
    virtual bool stepSimulation(const std::vector<SimulationBody*>& activeSimBodies) override;

private:
    AGXSimulatorItemImplPtr impl;
//    friend class AGXSimulatorItemImpl;
};
typedef ref_ptr<AGXSimulatorItem> AGXSimulatorItemPtr;

}

#endif
