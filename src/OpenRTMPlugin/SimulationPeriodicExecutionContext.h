/*!
  \file SimulationPeriodicExecutionContext.h
  \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_OPENRTM_PLUGIN_SIMULATION_PERIODIC_EXECUTION_CONTEXT_H
#define CNOID_OPENRTM_PLUGIN_SIMULATION_PERIODIC_EXECUTION_CONTEXT_H

#include <rtm/PeriodicExecutionContext.h>

#ifdef WIN32
#pragma warning( disable : 4290 )
#endif

namespace cnoid {

#ifdef OPENRTM_VERSION11
class SimulationPeriodicExecutionContext : public virtual RTC::PeriodicExecutionContext
{
public:
    virtual RTC::ReturnCode_t deactivate_component(RTC::LightweightRTObject_ptr comp) throw (CORBA::SystemException);
    
};

#elif defined(OPENRTM_VERSION12)
// Temporary
typedef RTC_exp::PeriodicExecutionContext SimulationPeriodicExecutionContext;

#endif

}

#endif
