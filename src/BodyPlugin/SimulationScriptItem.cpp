/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "SimulationScriptItem.h"
#include <cnoid/Archive>
#include <boost/bind.hpp>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace cnoid {

class SimulationScriptItemImpl
{
public:
    SimulationScriptItem* self;

    Selection execTiming;
    double execDelay;
    bool isOnlyExecutedAsSimulationScript;
    
    SimulationScriptItemImpl(SimulationScriptItem* self);
    SimulationScriptItemImpl(SimulationScriptItem* self, const SimulationScriptItemImpl& org);
};
}


SimulationScriptItem::SimulationScriptItem()
{
    impl = new SimulationScriptItemImpl(this);
}


SimulationScriptItemImpl::SimulationScriptItemImpl(SimulationScriptItem* self)
    : self(self),
      execTiming(SimulationScriptItem::NUM_TIMINGS, CNOID_GETTEXT_DOMAIN_NAME)
{
    execTiming.setSymbol(SimulationScriptItem::BEFORE_INITIALIZATION, N_("Before init."));
    execTiming.setSymbol(SimulationScriptItem::DURING_INITIALIZATION, N_("During init."));
    execTiming.setSymbol(SimulationScriptItem::AFTER_INITIALIZATION, N_("After init."));
    execTiming.setSymbol(SimulationScriptItem::DURING_FINALIZATION, N_("During final."));
    execTiming.setSymbol(SimulationScriptItem::AFTER_FINALIZATION, N_("After final."));

    execTiming.select(SimulationScriptItem::AFTER_INITIALIZATION);
    
    execDelay = 0.0;
    isOnlyExecutedAsSimulationScript = true;
}


SimulationScriptItem::SimulationScriptItem(const SimulationScriptItem& org)
    : ScriptItem(org)
{
    impl = new SimulationScriptItemImpl(this, *org.impl);
}


SimulationScriptItemImpl::SimulationScriptItemImpl(SimulationScriptItem* self, const SimulationScriptItemImpl& org)
    : self(self),
      execTiming(org.execTiming)
{
    execDelay = org.execDelay;
    isOnlyExecutedAsSimulationScript = org.isOnlyExecutedAsSimulationScript;
}


SimulationScriptItem::~SimulationScriptItem()
{
    delete impl;
}


SimulationScriptItem::ExecTiming SimulationScriptItem::execTiming() const
{
    return static_cast<SimulationScriptItem::ExecTiming>(impl->execTiming.selectedIndex());
}


double SimulationScriptItem::execDelay() const
{
    return impl->execDelay;
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
    
    putProperty(_("Timing"), impl->execTiming,
                bind((bool(Selection::*)(int))&Selection::select, &impl->execTiming, _1));
    putProperty(_("Delay"), impl->execDelay, changeProperty(impl->execDelay));
    putProperty(_("Simulation only"), impl->isOnlyExecutedAsSimulationScript,
                changeProperty(impl->isOnlyExecutedAsSimulationScript));
}


bool SimulationScriptItem::store(Archive& archive)
{
    if(ScriptItem::store(archive)){
        archive.write("timing", impl->execTiming.selectedSymbol());
        archive.write("delay", impl->execDelay);
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
            impl->execTiming.select(symbol);
        }
        archive.read("delay", impl->execDelay);
        archive.read("simulationOnly", impl->isOnlyExecutedAsSimulationScript);
        return true;
    }
    return false;
}
