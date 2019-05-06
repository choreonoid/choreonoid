/*!
 * @brief  This is a definition of Items for OpenRTM-aist.

 * @file
 */
#include "RTCWrapper.h"
#include "RTSConfigurationView.h"
#include "RTSCommonUtil.h"
#include "LoggerUtil.h"
#include <rtm/RTC.h>
#include <rtm/NVUtil.h>
#include <rtm/CORBA_SeqUtil.h>

using namespace cnoid;
using namespace RTC;

bool RTCWrapper::getConfiguration(NamingContextHelper::ObjectInfo& target, std::vector<ConfigurationSetParamPtr>& configSetList)
{
    DDEBUG("RTCWrapper::getConfiguration");

    std::vector<NamingContextHelper::ObjectPath> pathList;

    if (target.fullPath_.size() == 0) {
        NamingContextHelper::ObjectPath path(target.id_, "rtc");
        pathList.push_back(path);
    } else {
        pathList = target.fullPath_;
    }

    auto ncHelper = NameServerManager::instance()->getNCHelper();
    rtc_ = ncHelper->findObject<RTC::RTObject>(pathList);

    if (CORBA::is_nil(rtc_)) {
        return false;
    }

    QString activeName;
    try {
        compProfile_ = rtc_->get_component_profile();
        configuration_ = rtc_->get_configuration();

        SDOPackage::ConfigurationSet* activeConf = configuration_->get_active_configuration_set();
        if (activeConf) {
            activeName = QString(string(activeConf->id).c_str());
        }
    } catch (CORBA::SystemException& ex) {
        ncHelper->putExceptionMessage(ex);
        return false;
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

void RTCWrapper::updateConfiguration(std::vector<ConfigurationSetParamPtr>& configList)
{
    if (!isObjectAlive(configuration_)) return;

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

void RTCWrapper::setRTObject(RTC::RTObject_ptr target)
{
    DDEBUG("RTSComp::setRTObject");
    rtc_ = RTC::RTObject::_nil();

    if (!isObjectAlive(target)) {
        ownedExeContList_ = 0;
        DDEBUG("RTSComp::setRTObject NULL");
        return;
    }
    rtc_ = target;
    ownedExeContList_ = rtc_->get_owned_contexts();
    //
    compProfile_ = rtc_->get_component_profile();
    category_ = string(compProfile_->category);
    vendor_ = string(compProfile_->vendor);
    version_ = string(compProfile_->version);
}

bool RTCWrapper::activateComponent()
{
    if (!isObjectAlive(rtc_)) return false;
    if (ownedExeContList_->length() == 0) return false;
    if (!searchActiveEC()) return false;

    RTC::ReturnCode_t ret = ownedExeContList_[activeIndex_]->activate_component(rtc_);
    if (ret != RTC::ReturnCode_t::RTC_OK) return false;
    return true;
}

bool RTCWrapper::deactivateComponent()
{
    if (!isObjectAlive(rtc_)) return false;
    if (ownedExeContList_->length() == 0) return false;
    if (!searchActiveEC()) return false;

    RTC::ReturnCode_t ret = ownedExeContList_[activeIndex_]->deactivate_component(rtc_);
    if (ret != RTC::ReturnCode_t::RTC_OK) return false;
    return true;
}

bool RTCWrapper::resetComponent()
{
    if (!isObjectAlive(rtc_)) return false;
    if (ownedExeContList_->length() == 0) return false;
    if (!searchActiveEC()) return false;

    RTC::ReturnCode_t ret = ownedExeContList_[activeIndex_]->reset_component(rtc_);
    if (ret != RTC::ReturnCode_t::RTC_OK) return false;
    return true;
}

bool RTCWrapper::finalizeComponent()
{
    if (!isObjectAlive(rtc_)) return false;
    RTC::ReturnCode_t ret = rtc_->exit();
    if (ret != RTC::ReturnCode_t::RTC_OK) return false;
    return true;
}

bool RTCWrapper::startExecutionContext()
{
    if (ownedExeContList_->length() == 0) return false;
    if (!isObjectAlive(ownedExeContList_[activeIndex_])) return false;

    RTC::ReturnCode_t ret = ownedExeContList_[activeIndex_]->start();
    if (ret != RTC::ReturnCode_t::RTC_OK) return false;
    return true;
}

bool RTCWrapper::stopExecutionContext()
{
    bool result = false;
    if (ownedExeContList_->length() == 0) return result;

    for (CORBA::ULong index = 0; index < ownedExeContList_->length(); ++index) {
        if (isObjectAlive(ownedExeContList_[index])) {
            if (ownedExeContList_[index]->is_running()) {
                activeIndex_ = index;
                RTC::ReturnCode_t ret = ownedExeContList_[index]->stop();
                if (ret != RTC::ReturnCode_t::RTC_OK) return result;
                result = true;
                break;
            }
        }
    }
    return result;
}

RTC_STATUS RTCWrapper::getRTCState()
{
    if (CORBA::is_nil(rtc_) || rtc_->_non_existent()) return RTC_FINALIZE;
    if (ownedExeContList_->length() == 0) return RTC_UNKNOWN;

    if (searchActiveEC() == false) return RTC_UNKNOWN;

    LifeCycleState state = ownedExeContList_[activeIndex_]->get_component_state(rtc_);
    if (state == RTC::ERROR_STATE) {
        return RTC_ERROR;
    } else if (state == RTC::ACTIVE_STATE) {
        return RTC_ACTIVE;
    } else {
        return RTC_INACTIVE;
    }
}

bool RTCWrapper::searchActiveEC()
{
    bool result = false;
    for (CORBA::ULong index = 0; index < ownedExeContList_->length(); ++index) {
        if (isObjectAlive(ownedExeContList_[index])) {
            if (ownedExeContList_[index]->is_running()) {
                activeIndex_ = index;
                result = true;
                break;
            }
        }
    }
    return result;
}
