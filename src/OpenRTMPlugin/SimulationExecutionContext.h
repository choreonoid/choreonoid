/*!
  @file SimulationExecutionContext.h
*/

#ifndef CNOID_OPENRTM_PLUGIN_SIMULATION_EXECUTION_CONTEXT_H
#define CNOID_OPENRTM_PLUGIN_SIMULATION_EXECUTION_CONTEXT_H

#include <rtm/RTC.h>
#include <coil/Task.h>
#include <rtm/Manager.h>

#if defined(OPENRTM_VERSION11)
#include <rtm/PeriodicExecutionContext.h>
#elif defined(OPENRTM_VERSION12)
#include <rtm/OpenHRPExecutionContext.h>
#endif

#ifdef _WIN32
#pragma warning( disable : 4290 )
#endif

namespace cnoid {
/*!
  To call 'tick()' when the 'deactivate_component" function is called,
  OpenHRPExecutionContext is redefined as this class in the OpenRTM plugin.
  See post 02356 to the openrtm-users mailing list.
*/
#ifdef OPENRTM_VERSION11
class SimulationExecutionContext : public virtual RTC::PeriodicExecutionContext
#elif defined(OPENRTM_VERSION12)
class SimulationExecutionContext : public RTC::OpenHRPExecutionContext
#endif
{
public:
    SimulationExecutionContext();
    virtual ~SimulationExecutionContext(void);
    virtual void tick(void) throw (CORBA::SystemException);
    virtual int svc(void);
    virtual RTC::ReturnCode_t activate_component(RTC::LightweightRTObject_ptr comp) throw (CORBA::SystemException);
    virtual RTC::ReturnCode_t deactivate_component(RTC::LightweightRTObject_ptr comp) throw (CORBA::SystemException);
    virtual RTC::ReturnCode_t reset_component(RTC::LightweightRTObject_ptr comp) throw (CORBA::SystemException);
};

}

#endif
