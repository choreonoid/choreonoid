/*!
  @file ChoreonoidExecutionContext.cpp
  @author Shin'ichiro Nakaoka
*/

#include "ChoreonoidExecutionContext.h"
#include <rtm/ECFactory.h>

#if defined(OPENRTM_VERSION12)
 #include <rtm/RTObjectStateMachine.h>
#endif

using namespace cnoid;

#if defined(OPENRTM_VERSION11)
  ChoreonoidExecutionContext::ChoreonoidExecutionContext()
      : PeriodicExecutionContext()
#elif defined(OPENRTM_VERSION12)
  ChoreonoidExecutionContext::ChoreonoidExecutionContext()
      : OpenHRPExecutionContext()
#endif
{

}


ChoreonoidExecutionContext::~ChoreonoidExecutionContext()
{

}


void ChoreonoidExecutionContext::tick() throw (CORBA::SystemException)
{
#if defined(OPENRTM_VERSION11)
    std::for_each(m_comps.begin(), m_comps.end(), invoke_worker());
#elif defined(OPENRTM_VERSION12)
    invokeWorker();
#endif
}


int ChoreonoidExecutionContext::svc(void)
{
    return 0;
}


RTC::ReturnCode_t ChoreonoidExecutionContext::deactivate_component(RTC::LightweightRTObject_ptr comp)
    throw (CORBA::SystemException)
{
#if defined(OPENRTM_VERSION11)
    RTC_TRACE(("deactivate_component()"));

    CompItr it = std::find_if(m_comps.begin(), m_comps.end(), find_comp(comp));
    if(it == m_comps.end()) {
        return RTC::BAD_PARAMETER;
    }
    if(!(it->_sm.m_sm.isIn(RTC::ACTIVE_STATE))){
        return RTC::PRECONDITION_NOT_MET;
    }
    
    it->_sm.m_sm.goTo(RTC::INACTIVE_STATE);

    tick();
    
    if(it->_sm.m_sm.isIn(RTC::INACTIVE_STATE)){
        RTC_TRACE(("The component has been properly deactivated."));
        return RTC::RTC_OK;
    }
    
    RTC_ERROR(("The component could not be deactivated."));
    return RTC::RTC_ERROR;
#elif defined(OPENRTM_VERSION12)
    RTC_impl::RTObjectStateMachine* rtobj = m_worker.findComponent(comp);

    if (rtobj == NULL)
    {
        return RTC::BAD_PARAMETER;
    }
    if (!(rtobj->isCurrentState(RTC::ACTIVE_STATE)))
    {
        return RTC::PRECONDITION_NOT_MET;
    }
    rtobj->goTo(RTC::INACTIVE_STATE);

    tick();

    if (rtobj->isCurrentState(RTC::INACTIVE_STATE))
    {
        return RTC::RTC_OK;
    }

    return RTC::RTC_ERROR;
#endif
}    
