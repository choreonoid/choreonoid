/*!
  @file ChoreonoidExecutionContext.cpp
  @author Shin'ichiro Nakaoka
*/

#include "ChoreonoidExecutionContext.h"
#include <rtm/ECFactory.h>

using namespace cnoid;


ChoreonoidExecutionContext::ChoreonoidExecutionContext()
    : PeriodicExecutionContext()
{

}


ChoreonoidExecutionContext::~ChoreonoidExecutionContext()
{

}


void ChoreonoidExecutionContext::tick() throw (CORBA::SystemException)
{
    std::for_each(m_comps.begin(), m_comps.end(), invoke_worker());
}


int ChoreonoidExecutionContext::svc(void)
{
    return 0;
}


RTC::ReturnCode_t ChoreonoidExecutionContext::deactivate_component(RTC::LightweightRTObject_ptr comp) throw (CORBA::SystemException)
{
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
}    
