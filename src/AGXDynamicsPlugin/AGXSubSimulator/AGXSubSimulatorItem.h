#ifndef CNOID_AGX_PLUGIN_SUBSIMULATORITEM_H
#define CNOID_AGX_PLUGIN_SUBSIMULATORITEM_H

#include <cnoid/SubSimulatorItem>
#include <cnoid/BodyItem>
//#include "exportdecl.h"


namespace cnoid {

class AGXSimulatorItem;

//class CNOID_EXPORT AGXSubSimulatorItem : public SubSimulatorItem
class AGXSubSimulatorItem : public SubSimulatorItem
{
public:
    static void initializeClass(ExtensionManager* ext);
    AGXSubSimulatorItem();
    AGXSubSimulatorItem(const AGXSubSimulatorItem& org);
    ~AGXSubSimulatorItem();
    virtual bool initializeSimulation(SimulatorItem* simulatorItem);

protected:
    virtual Item* doDuplicate() const;
//    virtual void doPutProperties(PutPropertyFunction& putProperty);
//    virtual bool store(Archive& archive);
//    virtual bool restore(const Archive& archive);
//
private:
    SimulatorItem* simulatorItem;
    AGXSimulatorItem* agxSimulatorItem;
   void initialize();
};
typedef ref_ptr<AGXSubSimulatorItem> AGXSubSimulatorItemPtr;

}
#endif
