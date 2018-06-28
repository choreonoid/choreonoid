/*!
  \file SimulationPeriodicExecutionContext.cpp
  \author Shizuko Hattori
  \author Shin'ichiro Nakaoka
*/

#include "SimulationPeriodicExecutionContext.h"
#include "LoggerUtil.h"
#include <rtm/ECFactory.h>

#ifndef OPENRTM_VERSION11
 #include <rtm/RTObjectStateMachine.h>
#endif

using namespace cnoid;

SimulationPeriodicExecutionContext::SimulationPeriodicExecutionContext()
    : PeriodicExecutionContext()
{
    DDEBUG("SimulationPeriodicExecutionContext::SimulationPeriodicExecutionContext");
}


SimulationPeriodicExecutionContext::~SimulationPeriodicExecutionContext()
{
    
}


RTC::ReturnCode_t SimulationPeriodicExecutionContext::activate_component(RTC::LightweightRTObject_ptr comp)
    throw (CORBA::SystemException)
{
	DDEBUG("SimulationPeriodicExecutionContext::activate_component");

#ifdef OPENRTM_VERSION11
	CompItr it = std::find_if(m_comps.begin(), m_comps.end(), find_comp(comp));
	if(it == m_comps.end()){
		return RTC::BAD_PARAMETER;
    }
	if(!(it->_sm.m_sm.isIn(RTC::INACTIVE_STATE))){
		return RTC::PRECONDITION_NOT_MET;
    }
	it->_sm.m_sm.goTo(RTC::ACTIVE_STATE);
	it->_sm.worker();

	if ((it->_sm.m_sm.isIn(RTC::ACTIVE_STATE))){
		return RTC::RTC_OK;
    }

	return RTC::RTC_ERROR;
    
#else
    
	RTC_impl::RTObjectStateMachine* rtobj = m_worker.findComponent(comp);

	if(rtobj == NULL){
		return RTC::BAD_PARAMETER;
	}
	if(!(rtobj->isCurrentState(RTC::INACTIVE_STATE))){
		return RTC::PRECONDITION_NOT_MET;
	}
	m_syncActivation = false;

	RTC::ReturnCode_t ret = ExecutionContextBase::activateComponent(comp);
	invokeWorkerPreDo();
	if((rtobj->isCurrentState(RTC::ACTIVE_STATE))){
		return RTC::RTC_OK;
	}
	return RTC::RTC_ERROR;
    
#endif
}


RTC::ReturnCode_t SimulationPeriodicExecutionContext::deactivate_component(RTC::LightweightRTObject_ptr comp)
    throw (CORBA::SystemException)
{
    RTC_TRACE(("deactivate_component()"));

#ifdef OPENRTM_VERSION11

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

#else
    
    RTC_impl::RTObjectStateMachine* rtobj = m_worker.findComponent(comp);

    if (rtobj == NULL) {
        return RTC::BAD_PARAMETER;
    }
    if (!(rtobj->isCurrentState(RTC::ACTIVE_STATE))) {
        return RTC::PRECONDITION_NOT_MET;
    }

    m_syncDeactivation = false;
    RTC::ReturnCode_t ret = ExecutionContextBase::deactivateComponent(comp);
    invokeWorkerPreDo();

    if(!(rtobj->isCurrentState(RTC::INACTIVE_STATE))){
    	return RTC::RTC_OK;
    }
    
    return RTC::RTC_ERROR;

#endif
}


RTC::ReturnCode_t SimulationPeriodicExecutionContext::reset_component(RTC::LightweightRTObject_ptr comp)
    throw (CORBA::SystemException)
{
#ifdef OPENRTM_VERSION11
	CompItr it = std::find_if(m_comps.begin(), m_comps.end(), find_comp(comp));
	if (it == m_comps.end()){
		return RTC::BAD_PARAMETER;
    }

	if(!(it->_sm.m_sm.isIn(RTC::ERROR_STATE))){
		return RTC::PRECONDITION_NOT_MET;
    }

	it->_sm.m_sm.goTo(RTC::INACTIVE_STATE);
	it->_sm.worker();

	if ((it->_sm.m_sm.isIn(RTC::INACTIVE_STATE))){
		return RTC::RTC_OK;
    }

	return RTC::RTC_ERROR;

#else

	RTC_impl::RTObjectStateMachine* rtobj = m_worker.findComponent(comp);

	if(rtobj == NULL){
		return RTC::BAD_PARAMETER;
	}
	if(!(rtobj->isCurrentState(RTC::ERROR_STATE))){
		return RTC::PRECONDITION_NOT_MET;
	}
	m_syncReset = false;
	RTC::ReturnCode_t ret = ExecutionContextBase::resetComponent(comp);
	invokeWorkerPreDo();

	if((rtobj->isCurrentState(RTC::INACTIVE_STATE))){
		return RTC::RTC_OK;
	}
	return RTC::RTC_ERROR;
    
#endif
}
