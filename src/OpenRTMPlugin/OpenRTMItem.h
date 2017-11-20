/*!
 * @brief  This is a definition of Items for OpenRTM-aist.
 * @file
 */
#ifndef CNOID_OPENRTM_PLUGIN_OPENRTM_ITEM_H
#define CNOID_OPENRTM_PLUGIN_OPENRTM_ITEM_H

#include <rtm/idl/RTC.hh>

namespace cnoid {

class RTCWrapper {
public:
	RTCWrapper() : rtc_(0), ownedExeContList_(0){};
	~RTCWrapper() {};

	RTC::RTObject_ptr rtc_;
	RTC::ExecutionContextList_var ownedExeContList_;

	bool activateComponent();
	bool deactivateComponent();
	bool resetComponent();
	bool finalizeComponent();
	bool startExecutionContext();
	bool stopExecutionContext();

protected:
	void setRTObject(RTC::RTObject_ptr target);
};

}
#endif
