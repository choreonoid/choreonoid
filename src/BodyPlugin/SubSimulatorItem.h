/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_SUB_SIMULATOR_ITEM_H
#define CNOID_BODY_PLUGIN_SUB_SIMULATOR_ITEM_H

#include <cnoid/Item>
#include "exportdecl.h"

namespace cnoid {

class SimulatorItem;

class CNOID_EXPORT SubSimulatorItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);
    
    SubSimulatorItem();
    SubSimulatorItem(const SubSimulatorItem& org);
    
    virtual bool isEnabled();
    virtual bool setEnabled(bool on);
    virtual bool initializeSimulation(SimulatorItem* simulatorItem);
    virtual void finalizeSimulation();

protected:
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);
    
private:
    bool isEnabled_;
};

typedef ref_ptr<SubSimulatorItem> SubSimulatorItemPtr;
}

#endif
