/*!
  @file ChoreonoidPeriodicExecutionContext.h
  @author Shizuko Hattori
*/

#ifndef CNOID_OPENRTM_PLUGIN_CHOREONOID_PERIODIC_EXECUTION_CONTEXT_H
#define CNOID_OPENRTM_PLUGIN_CHOREONOID_PERIODIC_EXECUTION_CONTEXT_H

#include <rtm/RTC.h>
#include <coil/Task.h>
#include <rtm/Manager.h>
#include <rtm/PeriodicExecutionContext.h>

#ifdef _WIN32
#pragma warning( disable : 4290 )
#endif

namespace cnoid
{
/**
   This EC causes a problem of calling onDeactivae twice, so do not use it.
   You can use SimulationPeriodicExecutionContext instead of it.
*/
#if defined(OPENRTM_VERSION11)
class ChoreonoidPeriodicExecutionContext : public virtual RTC::PeriodicExecutionContext
#elif defined(OPENRTM_VERSION12)
class ChoreonoidPeriodicExecutionContext : public virtual RTC_exp::PeriodicExecutionContext
#endif
{
public:
    ChoreonoidPeriodicExecutionContext();
    virtual ~ChoreonoidPeriodicExecutionContext(void);
    virtual RTC::ReturnCode_t deactivate_component(RTC::LightweightRTObject_ptr comp) throw (CORBA::SystemException);
};

}

#endif
