/*!
  @file ChoreonoidPeriodicExecutionContext.h
  @author Shizuko Hattori
*/

#ifndef CNOID_OPENRTM_PLUGIN_CHOREONOID_PERIODIC_EXECUTION_CONTEXT_H_INCLUDED
#define CNOID_OPENRTM_PLUGIN_CHOREONOID_PERIODIC_EXECUTION_CONTEXT_H_INCLUDED

#include <rtm/RTC.h>
#include <coil/Task.h>
#include <rtm/Manager.h>
#ifdef OPENRTM_VERSION110
  #include <rtm/PeriodicExecutionContext.h>
#else
  #include <rtm/OpenHRPExecutionContext.h>
#endif

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
  class ChoreonoidPeriodicExecutionContext : public RTC::PeriodicExecutionContext
#endif
{
public:
    ChoreonoidPeriodicExecutionContext();
    virtual ~ChoreonoidPeriodicExecutionContext(void);
    virtual RTC::ReturnCode_t deactivate_component(RTC::LightweightRTObject_ptr comp) throw (CORBA::SystemException);
};
};

#endif

