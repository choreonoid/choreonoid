/*!
  \file SimulationPeriodicExecutionContext.cpp
  \author Shin'ichiro Nakaoka
*/

#include "SimulationPeriodicExecutionContext.h"

using namespace cnoid;

#ifdef OPENRTM_VERSION11

RTC::ReturnCode_t SimulationPeriodicExecutionContext::deactivate_component(RTC::LightweightRTObject_ptr comp)
throw (CORBA::SystemException)
{
    RTC_TRACE(("deactivate_component()"));
    
    CompItr it = std::find_if(m_comps.begin(), m_comps.end(), find_comp(comp));
    if(it == m_comps.end()) { return RTC::BAD_PARAMETER; }
    if(!(it->_sm.m_sm.isIn(RTC::ACTIVE_STATE))){
        return RTC::PRECONDITION_NOT_MET;
    }
    it->_sm.m_sm.goTo(RTC::INACTIVE_STATE);
    
    return RTC::RTC_OK;
}

#endif
