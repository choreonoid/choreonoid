/*!
 * @file ChoreonoidPeriodicExecutionContext.cpp
 *  @author  hattori
 */

#include "ChoreonoidPeriodicExecutionContext.h"
#include <rtm/ECFactory.h>
#if defined(OPENRTM_VERSION12)
 #include <rtm/RTObjectStateMachine.h>
#endif

using namespace cnoid;

ChoreonoidPeriodicExecutionContext::ChoreonoidPeriodicExecutionContext()
    : PeriodicExecutionContext()
{

}


ChoreonoidPeriodicExecutionContext::~ChoreonoidPeriodicExecutionContext()
{

}


RTC::ReturnCode_t ChoreonoidPeriodicExecutionContext::deactivate_component(RTC::LightweightRTObject_ptr comp)
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

    it->_sm.worker();

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

    rtobj->workerDo();

    if (rtobj->isCurrentState(RTC::INACTIVE_STATE))
    {
    	return RTC::RTC_OK;
    }
    return RTC::RTC_ERROR;
#endif
}
