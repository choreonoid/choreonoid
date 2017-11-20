/*!
 * @brief  This is a definition of Items for OpenRTM-aist.

 * @file
 */
#include "OpenRTMItem.h"
#include <cnoid/CorbaUtil>

#include "LoggerUtil.h"

#include "gettext.h"

namespace cnoid { 

void RTCWrapper::setRTObject(RTC::RTObject_ptr target) {
	rtc_ = 0;

	if (!NamingContextHelper::isObjectAlive(target)) {
		ownedExeContList_ = 0;
		return;
	}
	rtc_ = target;
	ownedExeContList_ = rtc_->get_owned_contexts();
}

bool RTCWrapper::activateComponent() {
	if (rtc_ == 0) return false;
	if (ownedExeContList_->length() == 0) return false;

	RTC::ReturnCode_t ret = ownedExeContList_[0]->activate_component(rtc_);
	if (ret != RTC::ReturnCode_t::RTC_OK) return false;
	return true;
}

bool RTCWrapper::deactivateComponent() {
	if (rtc_ == 0) return false;
	if (ownedExeContList_->length() == 0) return false;

	RTC::ReturnCode_t ret = ownedExeContList_[0]->deactivate_component(rtc_);
	if (ret != RTC::ReturnCode_t::RTC_OK) return false;
	return true;
}

bool RTCWrapper::resetComponent() {
	if (rtc_ == 0) return false;
	if (ownedExeContList_->length() == 0) return false;

	RTC::ReturnCode_t ret = ownedExeContList_[0]->reset_component(rtc_);
	if (ret != RTC::ReturnCode_t::RTC_OK) return false;
	return true;
}

bool RTCWrapper::finalizeComponent() {
	if (rtc_ == 0) return false;

	RTC::ReturnCode_t ret = rtc_->exit();
	if (ret != RTC::ReturnCode_t::RTC_OK) return false;
	return true;
}

bool RTCWrapper::startExecutionContext() {
	if (ownedExeContList_->length() == 0) return false;

	RTC::ReturnCode_t ret = ownedExeContList_[0]->start();
	if (ret != RTC::ReturnCode_t::RTC_OK) return false;
	return true;
}

bool RTCWrapper::stopExecutionContext() {
	if (ownedExeContList_->length() == 0) return false;

	RTC::ReturnCode_t ret = ownedExeContList_[0]->stop();
	if (ret != RTC::ReturnCode_t::RTC_OK) return false;
	return true;
}

}
