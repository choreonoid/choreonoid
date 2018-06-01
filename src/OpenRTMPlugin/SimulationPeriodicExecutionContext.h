/*!
  \file SimulationPeriodicExecutionContext.h
  \author Shizuko Hattori
  \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_OPENRTM_PLUGIN_SIMULATION_PERIODIC_EXECUTION_CONTEXT_H
#define CNOID_OPENRTM_PLUGIN_SIMULATION_PERIODIC_EXECUTION_CONTEXT_H

#include <rtm/RTC.h>
#include <rtm/Manager.h>
#include <rtm/PeriodicExecutionContext.h>
#include <coil/Task.h>

#ifdef WIN32
#pragma warning( disable : 4290 )
#endif

namespace cnoid
{
/*!
  To call 'tick()' when the 'deactivate_component" function is called,
  OpenHRPExecutionContext is redefined as this class in the OpenRTM plugin.
  See post 02356 to the openrtm-users mailing list.
*/
#ifdef OPENRTM_VERSION11
class SimulationPeriodicExecutionContext : public virtual RTC::PeriodicExecutionContext
#else
class SimulationPeriodicExecutionContext : public virtual RTC_exp::PeriodicExecutionContext
#endif
{
public:
    SimulationPeriodicExecutionContext();
    virtual ~SimulationPeriodicExecutionContext(void);

    virtual RTC::ReturnCode_t activate_component(RTC::LightweightRTObject_ptr comp) throw (CORBA::SystemException);
    virtual RTC::ReturnCode_t deactivate_component(RTC::LightweightRTObject_ptr comp) throw (CORBA::SystemException);
    virtual RTC::ReturnCode_t reset_component(RTC::LightweightRTObject_ptr comp) throw (CORBA::SystemException);
};

}

#endif
