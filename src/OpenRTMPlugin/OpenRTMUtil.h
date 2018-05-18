/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_OPENRTM_PLUGIN_OPENRTM_UTIL_H
#define CNOID_OPENRTM_PLUGIN_OPENRTM_UTIL_H

#include <cnoid/Signal>
#include <rtm/Manager.h>
#include <rtm/ManagerServant.h>
#include "exportdecl.h"

namespace cnoid {

// CNOID_EXPORT RTM::Manager_ptr getRTCManagerServant();

/**
   Components managed by plugins should be created by using this function instead of using RTC::Manager::createComponent().
   A component created by this function is registered as a plugin-managed component,
   and functions such as 'removeUnmanagedComponents()' ignore those components.
*/
CNOID_EXPORT RTC::RTObject_impl* createManagedRTC(const std::string& comp_args);

CNOID_EXPORT int numUnmanagedRTCs();
CNOID_EXPORT int deleteUnmanagedRTCs();

bool isManagedRTC(RTC::RTObject_ptr rtc);

CNOID_EXPORT bool deleteRTC(RTC::RtcBase* rtc);

CNOID_EXPORT SignalProxy<void()> sigAboutToFinalizeRTM();

//CNOID_EXPORT RTM::Manager_ptr getRTCManagerServant();

template<class ServiceType>
typename ServiceType::_ptr_type findRTCService(RTC::RTObject_ptr rtc, const std::string& name)
{
    CORBA::Object_var obj = findRTCService<CORBA::Object>(rtc, name);
    return CORBA::is_nil(obj) ? ServiceType::_nil() : ServiceType::_narrow(obj);
}

template<> CNOID_EXPORT CORBA::Object::_ptr_type findRTCService<CORBA::Object>(RTC::RTObject_ptr rtc, const std::string& name);

}

#endif
