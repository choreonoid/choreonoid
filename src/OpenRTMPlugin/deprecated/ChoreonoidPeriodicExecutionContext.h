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
#ifdef OPENRTM_VERSION110
class ChoreonoidPeriodicExecutionContext : public virtual RTC::PeriodicExecutionContext
#else
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
