/*!
 * @brief  This is a definition of Items for OpenRTM-aist.

 * @file
 */
#include "OpenRTMItem.h"

#include <rtm/NVUtil.h>
#include <rtm/idl/RTC.hh>
#include <rtm/CORBA_SeqUtil.h>
#include <coil/Properties.h>

#include <cnoid/CorbaUtil>

#include "RTSNameServerView.h"
#include "RTSConfigurationView.h"

#include "LoggerUtil.h"

#include "gettext.h"

using namespace RTC;

namespace cnoid { 

bool RTCWrapper::getConfiguration(NamingContextHelper::ObjectInfo& target, std::vector<ConfigurationSetParamPtr>& configSetList) {
  DDEBUG("RTCWrapper::getConfiguration");
 
  RTSNameServerView* nsView = RTSNameServerView::instance();
  ncHelper_.setLocation(nsView->getHost(), nsView->getPort());

	RTCWrapper result;
  if (target.fullPath.size() == 0) {
    rtc_ = ncHelper_.findObject<RTC::RTObject>(target.id, "rtc");
	} else {
    CORBA::Object::_ptr_type obj = ncHelper_.findObject(target.fullPath);
		if (CORBA::is_nil(obj)) {
      rtc_ = RTC::RTObject::_nil();
		} else {
      rtc_ = RTC::RTObject::_narrow(obj);
			CORBA::release(obj);
		}
	}
  if (!ncHelper_.isObjectAlive(rtc_)) return false;

  compProfile_ = rtc_->get_component_profile();
  configuration_ = rtc_->get_configuration();

	QString activeName;
	try {
    SDOPackage::ConfigurationSet* activeConf = configuration_->get_active_configuration_set();
    if (activeConf) {
      activeName = QString(string(activeConf->id).c_str());
		}
	} catch (...) {
  }

	SDOPackage::ConfigurationSetList_var confSet = configuration_->get_configuration_sets();
  int setNum = confSet->length();
	for (int index = 0; index < setNum; index++) {
		SDOPackage::ConfigurationSet conf = confSet[index];
		QString name = QString(string(conf.id).c_str());
		ConfigurationSetParamPtr configSet = std::make_shared<ConfigurationSetParam>(index + 1, name);
		if (name == activeName) configSet->setActive(true);
		configSetList.push_back(configSet);
		//
		coil::Properties confs = NVUtil::toProperties(conf.configuration_data);
		vector<string> confNames = confs.propertyNames();
		int id = 1;
		for (vector<string>::iterator iter = confNames.begin(); iter != confNames.end(); iter++) {
			QString name = QString((*iter).c_str());
			QString value = QString((confs[*iter]).c_str());
			ConfigurationParamPtr param = std::make_shared<ConfigurationParam>(id, name, value);
			configSet->addConfiguration(param);
			id++;
		}
	}
  return true;
}

void RTCWrapper::updateConfiguration(std::vector<ConfigurationSetParamPtr>& configList) {
	QString activeName;
	for (int index = 0; index < configList.size(); index++) {
		ConfigurationSetParamPtr target = configList[index];
		if (target->getActive()) {
			activeName = target->getName();
		}

		if (target->getMode() == ParamMode::MODE_INSERT) {
			SDOPackage::ConfigurationSet newSet;
			newSet.id = CORBA::string_dup(target->getName().toStdString().c_str());
			NVList configList;
			std::vector<ConfigurationParamPtr> configSetList = target->getConfigurationList();
			for (int idxDetail = 0; idxDetail < configSetList.size(); idxDetail++) {
				ConfigurationParamPtr param = configSetList[idxDetail];
				if (param->getMode() == MODE_DELETE || param->getMode() == MODE_IGNORE) continue;
				DDEBUG_V("id:%d", param->getId());
				CORBA_SeqUtil::push_back(configList,
					NVUtil::newNV(param->getName().toStdString().c_str(), param->getValue().toStdString().c_str()));
			}
			newSet.configuration_data = configList;
			configuration_->add_configuration_set(newSet);

		} else if (target->getMode() == ParamMode::MODE_UPDATE) {
			SDOPackage::ConfigurationSetList_var confSet = configuration_->get_configuration_sets();
			for (int idxConf = 0; idxConf < confSet->length(); idxConf++) {
				SDOPackage::ConfigurationSet conf = confSet[idxConf];
				QString name = QString(string(conf.id).c_str());
				if (name == target->getNameOrg()) {
					conf.id = CORBA::string_dup(target->getName().toStdString().c_str());
					NVList configList;
					std::vector<ConfigurationParamPtr> configSetList = target->getConfigurationList();
					for (int idxDetail = 0; idxDetail < configSetList.size(); idxDetail++) {
						ConfigurationParamPtr param = configSetList[idxDetail];
						if (param->getMode() == MODE_DELETE || param->getMode() == MODE_IGNORE) continue;
						DDEBUG_V("id:%d", param->getId());
						CORBA_SeqUtil::push_back(configList,
							NVUtil::newNV(param->getName().toStdString().c_str(), param->getValue().toStdString().c_str()));
					}
					conf.configuration_data = configList;
					configuration_->set_configuration_set_values(conf);
				}
			}

		} else if (target->getMode() == ParamMode::MODE_DELETE) {
			configuration_->remove_configuration_set(CORBA::string_dup(target->getName().toStdString().c_str()));
		}
	}
	configuration_->activate_configuration_set(activeName.toStdString().c_str());
}

void RTCWrapper::setRTObject(RTC::RTObject_ptr target) {
	DDEBUG("RTSComp::setRTObject");
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

  for (CORBA::ULong index = 0; index < ownedExeContList_->length(); ++index) {
    if (ownedExeContList_[index]->is_running()) {
      activeIndex_ = index;
      break;
    }
  }

	RTC::ReturnCode_t ret = ownedExeContList_[activeIndex_]->activate_component(rtc_);
	if (ret != RTC::ReturnCode_t::RTC_OK) return false;
	return true;
}

bool RTCWrapper::deactivateComponent() {
	if (rtc_ == 0) return false;
	if (ownedExeContList_->length() == 0) return false;

  for (CORBA::ULong index = 0; index < ownedExeContList_->length(); ++index) {
    if (ownedExeContList_[index]->is_running()) {
      activeIndex_ = index;
      break;
    }
  }

  RTC::ReturnCode_t ret = ownedExeContList_[activeIndex_]->deactivate_component(rtc_);
	if (ret != RTC::ReturnCode_t::RTC_OK) return false;
	return true;
}

bool RTCWrapper::resetComponent() {
	if (rtc_ == 0) return false;
	if (ownedExeContList_->length() == 0) return false;

  for (CORBA::ULong index = 0; index < ownedExeContList_->length(); ++index) {
    if (ownedExeContList_[index]->is_running()) {
      activeIndex_ = index;
      break;
    }
  }
 
  RTC::ReturnCode_t ret = ownedExeContList_[activeIndex_]->reset_component(rtc_);
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

	RTC::ReturnCode_t ret = ownedExeContList_[activeIndex_]->start();
	if (ret != RTC::ReturnCode_t::RTC_OK) return false;
	return true;
}

bool RTCWrapper::stopExecutionContext() {
	if (ownedExeContList_->length() == 0) return false;

  for (CORBA::ULong index = 0; index < ownedExeContList_->length(); ++index) {
    if (ownedExeContList_[index]->is_running()) {
      activeIndex_ = index;
      RTC::ReturnCode_t ret = ownedExeContList_[index]->stop();
      if (ret != RTC::ReturnCode_t::RTC_OK) return false;
      break;
    }
  }

	return true;
}

RTC_STATUS RTCWrapper::getRTCState() {
	if (CORBA::is_nil(rtc_) || rtc_->_non_existent()) return RTC_FINALIZE;
	if (ownedExeContList_->length() == 0) return RTC_UNKNOWN;

  for (CORBA::ULong index = 0; index < ownedExeContList_->length(); ++index) {
    if (ownedExeContList_[index]->is_running()) {
      activeIndex_ = index;
      break;
    }
  }

	LifeCycleState state = ownedExeContList_[activeIndex_]->get_component_state(rtc_);
	if (state == RTC::ERROR_STATE) {
		return RTC_ERROR;
	} else if (state == RTC::ACTIVE_STATE) {
		return RTC_ACTIVE;
	} else {
		return RTC_INACTIVE;
	}
}

}
