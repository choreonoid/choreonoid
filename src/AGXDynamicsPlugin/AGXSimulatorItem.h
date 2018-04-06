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
    virtual Item* doDuplicate() const;
    void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);
    virtual SimulationBody* createSimulationBody(Body* orgBody);
    virtual bool initializeSimulation(const std::vector<SimulationBody*>& simBodies);
//    virtual void initializeSimulationThread();
//    virtual void finalizeSimulationThread();
    //virtual bool startSimulation(bool doReset = true);
    virtual bool stepSimulation(const std::vector<SimulationBody*>& activeSimBodies);
    virtual void stopSimulation();
    virtual void pauseSimulation();
    virtual void restartSimulation();


private:
    AGXSimulatorItemImplPtr impl;
//    friend class AGXSimulatorItemImpl;
};
typedef ref_ptr<AGXSimulatorItem> AGXSimulatorItemPtr;

}

#endif
