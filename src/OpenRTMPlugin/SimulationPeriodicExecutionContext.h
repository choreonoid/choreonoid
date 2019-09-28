/*!
  \file SimulationPeriodicExecutionContext.h
  \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_OPENRTM_PLUGIN_SIMULATION_PERIODIC_EXECUTION_CONTEXT_H
#define CNOID_OPENRTM_PLUGIN_SIMULATION_PERIODIC_EXECUTION_CONTEXT_H

#include <rtm/PeriodicExecutionContext.h>

#ifdef _WIN32
#pragma warning( disable : 4290 )
#endif

namespace cnoid {

#ifdef OPENRTM_VERSION11
/**
   This EC has an asynchronous version of the deactivate function.
   By using this EC, the deactivation when the simulation is stopped does not block a while
   even if the RTC has long periodic cycle.
*/
class SimulationPeriodicExecutionContext : public virtual RTC::PeriodicExecutionContext
{
public:
    virtual RTC::ReturnCode_t deactivate_component(RTC::LightweightRTObject_ptr comp) throw (CORBA::SystemException);
    
};

#elif defined(OPENRTM_VERSION12)
// Temporary. Implement the similar EC for OpenRTM version 1.2.0
typedef RTC_exp::PeriodicExecutionContext SimulationPeriodicExecutionContext;

#endif

}

#endif
