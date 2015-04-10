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
    virtual bool isEnabled();
    virtual bool initializeSimulation(SimulatorItem* simulatorItem) = 0;
    virtual void finalizeSimulation() = 0;
};

typedef ref_ptr<SubSimulatorItem> SubSimulatorItemPtr;
}

#endif
