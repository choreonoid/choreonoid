/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "SimulationScriptItem.h"
#include <cnoid/ItemManager>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Archive>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class SimulationScriptItemImpl
{
public:
    SimulationScriptItem* self;

    Selection executionTiming;
    double executionDelay;
    bool isOnlyExecutedAsSimulationScript;
    
    SimulationScriptItemImpl(SimulationScriptItem* self);
    SimulationScriptItemImpl(SimulationScriptItem* self, const SimulationScriptItemImpl& org);
};

}


void SimulationScriptItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerAbstractClass<SimulationScriptItem, ScriptItem>();
}


SimulationScriptItem::SimulationScriptItem()
{
    impl = new SimulationScriptItemImpl(this);
}


SimulationScriptItemImpl::SimulationScriptItemImpl(SimulationScriptItem* self)
    : self(self),
      executionTiming(SimulationScriptItem::NUM_TIMINGS, CNOID_GETTEXT_DOMAIN_NAME)
{
    executionTiming.setSymbol(SimulationScriptItem::BEFORE_INITIALIZATION, N_("Before init."));
    executionTiming.setSymbol(SimulationScriptItem::DURING_INITIALIZATION, N_("During init."));
    executionTiming.setSymbol(SimulationScriptItem::AFTER_INITIALIZATION, N_("After init."));
    executionTiming.setSymbol(SimulationScriptItem::DURING_FINALIZATION, N_("During final."));
    executionTiming.setSymbol(SimulationScriptItem::AFTER_FINALIZATION, N_("After final."));

    executionTiming.select(SimulationScriptItem::AFTER_INITIALIZATION);
    
    executionDelay = 0.0;
    isOnlyExecutedAsSimulationScript = true;
}


SimulationScriptItem::SimulationScriptItem(const SimulationScriptItem& org)
    : ScriptItem(org)
{
    impl = new SimulationScriptItemImpl(this, *org.impl);
}


SimulationScriptItemImpl::SimulationScriptItemImpl(SimulationScriptItem* self, const SimulationScriptItemImpl& org)
    : self(self),
      executionTiming(org.executionTiming)
{
    executionDelay = org.executionDelay;
    isOnlyExecutedAsSimulationScript = org.isOnlyExecutedAsSimulationScript;
}


SimulationScriptItem::~SimulationScriptItem()
{
    delete impl;
}


SimulationScriptItem::ExecutionTiming SimulationScriptItem::executionTiming() const
{
    return static_cast<SimulationScriptItem::ExecutionTiming>(impl->executionTiming.selectedIndex());
}


void SimulationScriptItem::setExecutionTiming(SimulationScriptItem::ExecutionTiming timing)
{
    impl->executionTiming.select(timing);
}


double SimulationScriptItem::executionDelay() const
{
    return impl->executionDelay;
}


void SimulationScriptItem::setExecutionDelay(double t)
{
    impl->executionDelay = t;
}


bool SimulationScriptItem::execute()
{
    if(impl->isOnlyExecutedAsSimulationScript){
        return false;
    } else {
        return executeAsSimulationScript();
    }
}


void SimulationScriptItem::doPutProperties(PutPropertyFunction& putProperty)
{
    ScriptItem::doPutProperties(putProperty);
    
    putProperty(_("Timing"), impl->executionTiming,
                [&](int which){ return impl->executionTiming.select(which); });
    putProperty(_("Delay"), impl->executionDelay, changeProperty(impl->executionDelay));
    putProperty(_("Simulation only"), impl->isOnlyExecutedAsSimulationScript,
                changeProperty(impl->isOnlyExecutedAsSimulationScript));
}


bool SimulationScriptItem::store(Archive& archive)
{
    if(ScriptItem::store(archive)){
        archive.write("timing", impl->executionTiming.selectedSymbol());
        archive.write("delay", impl->executionDelay);
        archive.write("simulationOnly", impl->isOnlyExecutedAsSimulationScript);
        return true;
    }
    return false;
}


bool SimulationScriptItem::restore(const Archive& archive)
{
    if(ScriptItem::restore(archive)){
        string symbol;
        if(archive.read("timing", symbol)){
            impl->executionTiming.select(symbol);
        }
        archive.read("delay", impl->executionDelay);
        archive.read("simulationOnly", impl->isOnlyExecutedAsSimulationScript);
        return true;
    }
    return false;
}
