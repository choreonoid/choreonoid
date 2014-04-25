/*!
  @file ChoreonoidExecutionContext.h
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_OPENRTM_PLUGIN_CHOREONOID_EXECUTION_CONTEXT_H_INCLUDED
#define CNOID_OPENRTM_PLUGIN_CHOREONOID_EXECUTION_CONTEXT_H_INCLUDED

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
class ChoreonoidExecutionContext : public virtual RTC::PeriodicExecutionContext
{
public:
    ChoreonoidExecutionContext();
    virtual ~ChoreonoidExecutionContext(void);
    virtual void tick(void) throw (CORBA::SystemException);
    virtual int svc(void);
    virtual RTC::ReturnCode_t deactivate_component(RTC::LightweightRTObject_ptr comp) throw (CORBA::SystemException);
};
};

#endif
