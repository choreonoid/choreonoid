#include "RTSystemItemEx.h"
#include "../RTSTypeUtil.h"
#include "../ProfileHandler.h"
#include "../LoggerUtil.h"
#include <cnoid/MessageView>
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/EigenArchive>
#include <cnoid/AppConfig>
#include <cnoid/Timer>
#include <cnoid/LazyCaller>
#include <rtm/CORBA_SeqUtil.h>
#include <fmt/format.h>
#include <thread>
#include "../gettext.h"

using namespace cnoid;
using namespace std;
using namespace RTC;
using fmt::format;

namespace cnoid {

struct RTSPortComparator
{
    std::string name_;

    RTSPortComparator(std::string value)
    {
        name_ = value;
    }
    bool operator()(const RTSPortPtr elem) const
    {
        return (name_ == elem->name);
    }
};
//////////
class RTSystemItemExImpl
{
public:
    RTSystemItemEx* self;

    map<string, RTSCompPtr> rtsComps;
    RTSystemItemEx::RTSConnectionMap rtsConnections;
    bool autoConnection;

    map<string, RTSCompPtr> rtsCompsCheck; // Ext2
    RTSystemItemEx::RTSConnectionMap rtsConnectionsCheck; // Ext2
    RTSystemItemEx::RTSConnectionMap rtsConnectionsAdded; // Ext2

    std::string vendorName;
    std::string version;
    int pollingCycle;
    bool checkAtLoading;

#if defined(OPENRTM_VERSION12)
    int heartBeatPeriod;
#endif

    Selection stateCheckMode;

    enum PollingThreadType { FOREGROUND, BACKGROUND, N_POLLING_THREAD_TYPES };
    Selection pollingThreadType;
    
    Connection locationChangedConnection;
    int connectionNo;

    Signal<void(bool)> sigStatusUpdate;

    Timer timer;
    ScopedConnection timeOutConnection;

    // For the background polling
    cnoid::LazyCaller updateStateLater;
    bool statusModified;
    bool isCheckingStatus;
    bool doStatusChecking;
    std::thread pollingThread;

    RTSystemItemExImpl(RTSystemItemEx* self);
    RTSystemItemExImpl(RTSystemItemEx* self, const RTSystemItemExImpl& org);
    ~RTSystemItemExImpl();

    void onLocationChanged(std::string host, int port);
    RTSComp* addRTSComp(const string& name, const QPointF& pos);
    RTSComp* addRTSComp(const NamingContextHelper::ObjectInfo& info, const QPointF& pos);
    void deleteRTSComp(const string& name);
    RTSComp* nameToRTSComp(const string& name);
    bool compIsAlive(RTSComp* rtsComp);
    RTSComp* nameToRTSCompForChecking(const string& name); // Ext2
    bool compIsAliveForChecking(RTSComp* rtsComp);
    RTSConnection* addRTSConnection(
        const string& id, const string& name,
        RTSPort* sourcePort, RTSPort* targetPort, const std::vector<NamedValuePtr>& propList,
        const bool setPos, const Vector2 pos[]);
    RTSConnection* addRTSConnectionName(
        const string& id, const string& name,
        const string& sourceComp, const string& sourcePort,
        const string& targetComp, const string& targetPort,
        const string& dataflow, const string& subscription,
        const bool setPos, const Vector2 pos[]);
    bool connectionCheck(); // Ext1
    void RTSCompToConnectionList(const RTSComp* rtsComp, list<RTSConnection*>& rtsConnectionList, int mode);
    void removeConnection(RTSConnection* connection);
    void doPutProperties(PutPropertyFunction& putProperty);
    bool saveRtsProfile(const string& filename);
    void restoreRTSystem(const Archive& archive);
    void restoreRTSComp(const string& name, const Vector2& pos,
            const vector<pair<string, bool>>& inPorts, const vector<pair<string, bool>>& outPorts);
    string getConnectionNumber();
    void setStateCheckModeByString(const string& mode);
    void checkStatus();
    void checkStatusForeground();
    void checkStatusBackground();
    void statusChecking();

    void onActivated();
    void changePollingPeriod(int value);
    void changeStateCheckMode();

    void setStateCheckMode(int mode);
    void copyForStatusChecking();
    void restoreForStatusChecking();
};

}
//////////
RTSPort::RTSPort(const string& name_, PortService_var port_, RTSComp* parent)
    : rtsComp(parent), isConnected_(false)
{
    isInPort = true;
    name = name_;
    port = port_;
    //
    if (port && isObjectAlive(port)) {
        RTC::PortProfile_var profile = port->get_port_profile();
        RTC::PortInterfaceProfileList interfaceList = profile->interfaces;
        for (CORBA::ULong index = 0; index < interfaceList.length(); ++index) {
            RTC::PortInterfaceProfile ifProfile = interfaceList[index];
            PortInterfacePtr portIf(new PortInterface());
            if (parent) {
                portIf->rtc_name = parent->name;
            }
            const char* port_name;
            portIf->port_name = string((const char*)profile->name);
            if (ifProfile.polarity == PortInterfacePolarity::REQUIRED) {
                portIf->if_polarity = "required";
            } else {
                portIf->if_polarity = "provided";
            }
            const char* if_iname;
            const char* if_tname;
            if_iname = (const char*)ifProfile.instance_name;
            if_tname = (const char*)ifProfile.type_name;
            DDEBUG_V("name: %s, IF name: %s, instance_name: %s, type_name: %s", name_.c_str(), portIf->port_name.c_str(), if_iname, if_tname);
            portIf->if_tname = if_tname;
            portIf->if_iname = if_iname;
            interList.push_back(portIf);
        }
        isConnected_ = checkConnected();
        DDEBUG_V("RTSPort::RTSPort Port isConnected_ %s,%d", name.c_str(), isConnected_);
    }
}


// Ext2
RTSPortPtr RTSPort::copyForChecking() {
  //DDEBUG("RTSPort::copyForChecking");
  RTSPortPtr result = new RTSPort(this->name, this->port, this->rtsComp);
  result->isServicePort = this->isServicePort;
  result->isInPort = this->isInPort;
  result->isConnected_ = this->isConnected_;
  result->orgPort_ = this;
  return result;
}


// Ext2
bool RTSPort::checkConnected()
{
    if (!port || !isObjectAlive(port)) {
        return false;
    }
    ConnectorProfileList_var connectorProfiles = port->get_connector_profiles();
    return connectorProfiles->length() != 0;
}


bool RTSPort::isConnectedWith(RTSPort* target)
{
    DDEBUG("RTSPort::isConnectedWith");
    if( !cnoid::isObjectAlive(port) || !cnoid::isObjectAlive(target->port)) {
        DDEBUG("RTSPort::isConnectedWith False");
        return false;
    }

    DDEBUG("RTSPort::isConnectedWith Check");
    ConnectorProfileList_var connectorProfiles = port->get_connector_profiles();

    for (CORBA::ULong i = 0; i < connectorProfiles->length(); ++i) {
        ConnectorProfile& connectorProfile = connectorProfiles[i];
        PortServiceList& connectedPorts = connectorProfile.ports;
        for (CORBA::ULong j = 0; j < connectedPorts.length(); ++j) {
            PortService_ptr connectedPortRef = connectedPorts[j];
            if (connectedPortRef->_is_equivalent(target->port)) {
                DDEBUG("RTSPort::isConnectedWith True");
                return true;
            }
        }
    }

    DDEBUG("RTSPort::isConnectedWith False");
    return false;
}


bool RTSPort::checkConnectablePort(RTSPort* target)
{
    DDEBUG("RTSPort::checkConnectablePort");
    if (rtsComp == target->rtsComp) {
        return false;
    }
    if (!port || !target->port) {
        return false;
    }

    if ((isInPort && target->isInPort) ||
        (isInPort && target->isServicePort) ||
        (!isInPort && !isServicePort && !target->isInPort) ||
        (isServicePort && !target->isServicePort)) {
        return false;
    }

    //In case of connection between data ports
    if (!isServicePort && !target->isServicePort) {
        vector<string> dataTypes = RTSTypeUtil::getAllowDataTypes(this, target);
        vector<string> ifTypes = RTSTypeUtil::getAllowInterfaceTypes(this, target);
        vector<string> subTypes = RTSTypeUtil::getAllowSubscriptionTypes(this, target);
        if (dataTypes.size() == 0 || ifTypes.size() == 0 || subTypes.size() == 0) {
            return false;
        }
    }

    return true;
}


vector<string> RTSPort::getDataTypes()
{
    return getProperty("dataport.data_type");
}


vector<string> RTSPort::getInterfaceTypes()
{
    return getProperty("dataport.interface_type");
}


vector<string> RTSPort::getDataflowTypes()
{
    return getProperty("dataport.dataflow_type");
}


vector<string> RTSPort::getSubscriptionTypes()
{
    return getProperty("dataport.subscription_type");
}


vector<string> RTSPort::getProperty(const string& key)
{
    vector<string> result;

    NVList properties = port->get_port_profile()->properties;
    for (int index = 0; index < properties.length(); ++index) {
        string strName(properties[index].name._ptr);
        if (strName == key) {
            const char* nvValue;
            properties[index].value >>= nvValue;
            string strValue(nvValue);
            result = RTCCommonUtil::split(strValue, ',');
            break;
        }
    }

    return result;
}

RTSPort* RTSComp::nameToRTSPort(const string& name)
{
    //DDEBUG_V("RTSComp::nameToRTSPort %s", name.c_str());

    auto it = find_if(inPorts.begin(), inPorts.end(), RTSPortComparator(name));
    if (it != inPorts.end()) {
        return *it;
    }
    it = find_if(outPorts.begin(), outPorts.end(), RTSPortComparator(name));
    if (it != outPorts.end()) {
        return *it;
    }
    return nullptr;
}
//////////
RTSConnection::RTSConnection(const string& id, const string& name,
        const string& sourceRtcName, const string& sourcePortName,
        const string& targetRtcName, const string& targetPortName)
    : id(id), name(name), sourceRtcName(sourceRtcName), sourcePortName(sourcePortName),
    targetRtcName(targetRtcName), targetPortName(targetPortName), setPos(false)
{
    isAlive_ = false;
}

// Ext2
RTSConnectionPtr RTSConnection::copyForChecking()
{
    DDEBUG("RTSConnection::copyForChecking");
    RTSConnectionPtr result = new RTSConnection(
                                  this->id, this->name,
                                  this->sourceRtcName, this->sourcePortName,
                                  this->targetRtcName, this->targetRtcName);
    result->srcRTC = this->srcRTC;
    result->sourcePort = this->sourcePort;
    result->targetRTC = this->targetRTC;
    result->targetPort = this->targetPort;
    result->isAlive_ = this->isAlive_;

    return result;
}

bool RTSConnection::connect()
{
    DDEBUG("RTSConnection::connect");

    if (!sourcePort || !targetPort) {
        return false;
    }

    if (!sourcePort->port || !targetPort->port ||
       CORBA::is_nil(sourcePort->port) || sourcePort->port->_non_existent() ||
       CORBA::is_nil(targetPort->port) || targetPort->port->_non_existent()) {
        DDEBUG("RTSConnection::connect False");
        return false;
    }

    if (sourcePort->isConnectedWith(targetPort)) {
        return false;
    }

    ConnectorProfile cprof;
    cprof.connector_id = CORBA::string_dup(id.c_str());
    cprof.name = CORBA::string_dup(name.c_str());
    cprof.ports.length(2);
    cprof.ports[0] = PortService::_duplicate(sourcePort->port);
    cprof.ports[1] = PortService::_duplicate(targetPort->port);

    for (int index = 0; index < propList.size(); index++) {
        NamedValuePtr param = propList[index];
        CORBA_SeqUtil::push_back(
            cprof.properties, NVUtil::newNV(param->name_.c_str(), param->value_.c_str()));
    }

    RTC::ReturnCode_t result = sourcePort->port->connect(cprof);
    if (result == RTC::RTC_OK) {
        PortProfile_var portprofile = sourcePort->port->get_port_profile();
        ConnectorProfileList connections = portprofile->connector_profiles;
        for (CORBA::ULong i = 0; i < connections.length(); i++) {
            ConnectorProfile& connector = connections[i];
            PortServiceList& connectedPorts = connector.ports;
            for (CORBA::ULong j = 0; j < connectedPorts.length(); ++j) {
                PortService_ptr connectedPortRef = connectedPorts[j];
                if (connectedPortRef->_is_equivalent(targetPort->port)) {
                    id = string(connector.connector_id);
                    isAlive_ = true;
                    DDEBUG("RTSConnection::connect End");
                    return true;
                }
            }
        }
        return false;
    }

    DDEBUG("connect Error");
    return false;
}


bool RTSConnection::disconnect()
{
    isAlive_ = false;

    if (CORBA::is_nil(sourcePort->port) || sourcePort->port->_non_existent()) {
        return false;
    }

    return (sourcePort->port->disconnect(id.c_str()) == RTC_OK);
}


void RTSConnection::setPosition(const Vector2 pos[])
{
    for (int i = 0; i < 6; ++i) {
        position[i] = pos[i];
    }
    setPos = true;
    srcRTC->rtsItem()->suggestFileUpdate();
}
//////////
RTSComp::RTSComp
(const string& name, const std::string& fullPath, RTC::RTObject_ptr rtc, Item* rtsItem, const QPointF& pos,
 const string& host, int port, bool isDefault)
    : rtsItem_(rtsItem),
      pos_(pos),
      name(name),
      fullPath(fullPath),
      hostAddress(host),
      portNo(port),
      isDefaultNS(isDefault)
{
    setRtc(rtc);
}

// Ext2
RTSCompPtr RTSComp::copyForChecking() {
  DDEBUG("RTSComp::copyForChecking");
  RTSCompPtr result= new RTSComp(
                          this->name, this->fullPath, NULL, 
                          this->rtsItem_, this->pos_,
                          this->hostAddress, this->portNo, this->isDefaultNS);
  result->rtc_ = this->rtc_;
  result->orgComp_ = this;
  result->rtc_status_ = this->rtc_status_;
  result->isAlive_ = this->isAlive_;

  if (!isObjectAlive(this->rtc_)) {
    DDEBUG("RTSComp::copyForChecking RTC NOT ALIVE");
    result->participatingExeContList = 0;

    result->inPorts.clear();
    for (auto it = this->inPorts.begin(); it != this->inPorts.end(); it++) {
      RTSPortPtr port = (*it)->copyForChecking();
      result->inPorts.push_back(port);
    }
    result->outPorts.clear();
    for (auto it = this->outPorts.begin(); it != this->outPorts.end(); it++) {
      RTSPortPtr port = (*it)->copyForChecking();
      result->outPorts.push_back(port);
    }

  } else {
    DDEBUG("RTSComp::copyForChecking RTC ALIVE");
    result->ownedExeContList_ = this->ownedExeContList_;
    result->activeIndex_ = this->activeIndex_;

    result->inPorts.clear();
    for (auto it = this->inPorts.begin(); it != this->inPorts.end(); it++) {
      result->inPorts.push_back((*it)->copyForChecking());
    }
    result->outPorts.clear();
    for (auto it = this->outPorts.begin(); it != this->outPorts.end(); it++) {
      result->outPorts.push_back((*it)->copyForChecking());
    }
  }
  DDEBUG("RTSComp::copyForChecking End");
  return result;
}


void RTSComp::setRtc(RTObject_ptr rtc)
{
    DDEBUG("RTSComp::setRtc");
    rtc_ = 0;

    //rtsItem_->suggestFileUpdate();

    setRTObject(rtc);

    if (!isObjectAlive(rtc)) {
        participatingExeContList = 0;
        for (auto it = inPorts.begin(); it != inPorts.end(); ++it) {
            RTSPort* port = *it;
            port->port = 0;
        }
        for (auto it = outPorts.begin(); it != outPorts.end(); ++it) {
            RTSPort* port = *it;
            port->port = 0;
        }
        isAlive_ = false;
        DDEBUG("RTSComp::setRtc Failed");
        return;
    }

    participatingExeContList = rtc_->get_participating_contexts();
    rtc_status_ = getRTCState();

    inPorts.clear();
    outPorts.clear();

    PortServiceList_var portlist = rtc_->get_ports();
    for (CORBA::ULong i = 0; i < portlist->length(); ++i) {
        PortProfile_var portprofile = portlist[i]->get_port_profile();
        coil::Properties pproperties = NVUtil::toProperties(portprofile->properties);
        string portType = pproperties["port.port_type"];
        RTSPortPtr rtsPort = new RTSPort(string(portprofile->name), portlist[i], this);
        if (RTCCommonUtil::compareIgnoreCase(portType, "CorbaPort")) {
            rtsPort->isServicePort = true;
            rtsPort->isInPort = false;
            outPorts.push_back(rtsPort);
        } else {
            rtsPort->isServicePort = false;
            if (RTCCommonUtil::compareIgnoreCase(portType, "DataInPort")) {
                inPorts.push_back(rtsPort);
            } else {
                rtsPort->isInPort = false;
                outPorts.push_back(rtsPort);
            }
        }
    }

    bool isUpdated = rtsItem_->isConsistentWithFile();
    list<RTSConnection*> rtsConnectionList;
    auto rtsImpl = static_cast<RTSystemItemEx*>(rtsItem_)->impl;
    rtsImpl->RTSCompToConnectionList(this, rtsConnectionList, 0);
    for (auto it = rtsConnectionList.begin(); it != rtsConnectionList.end(); ++it) {
        RTSConnectionPtr connection = (*it);
        rtsImpl->removeConnection(*it);
        RTSPort* sourcePort = connection->srcRTC->nameToRTSPort(connection->sourcePortName);
        RTSPort* targetPort = connection->targetRTC->nameToRTSPort(connection->targetPortName);
        if (sourcePort && targetPort) {
            connection->sourcePort = sourcePort;
            connection->targetPort = targetPort;
            rtsImpl->rtsConnections[RTSystemItemEx::RTSPortPair(sourcePort, targetPort)] = connection;
        }
    }
    rtsItem_->setConsistentWithFile(isUpdated);

    connectionCheck();
    isAlive_ = true;
    DDEBUG("RTSComp::setRtc End");
}

bool RTSComp::connectionCheck()
{
    //DDEBUG("RTSComp::connectionCheck");

    bool updated = false;

    for (auto it = inPorts.begin(); it != inPorts.end(); ++it) {
        if (connectionCheckSub(*it)) {
            updated = true;
        }
        bool isCon = (*it)->checkConnected();
        if( (*it)->isConnected_ != isCon) {
            updated = true;
            (*it)->isConnected_ = isCon;
        }
    }
    for (auto it = outPorts.begin(); it != outPorts.end(); it++) {
        if (connectionCheckSub(*it)) {
            updated = true;
        }
        bool isCon = (*it)->checkConnected();
        if( (*it)->isConnected_ != isCon) {
            updated = true;
            (*it)->isConnected_ = isCon;
        }
    }

    return updated;
}


bool RTSComp::connectionCheckSub(RTSPort* rtsPort)
{
    bool updated = false;

    if (isObjectAlive(rtsPort->port) == false) return updated;

    auto rtsImpl = static_cast<RTSystemItemEx*>(rtsItem_)->impl;

    /**
       The get_port_profile() function should not be used here because
       it takes much time when the port has large data and its owner RTC is in a remote host.
       The get_connector_profiles() function does not seem to cause such a problem.
    */
    ConnectorProfileList_var connectorProfiles = rtsPort->port->get_connector_profiles();

    for (CORBA::ULong i = 0; i < connectorProfiles->length(); ++i) {
        ConnectorProfile& connectorProfile = connectorProfiles[i];
        PortServiceList& connectedPorts = connectorProfile.ports;
        for (CORBA::ULong j = 0; j < connectedPorts.length(); ++j) {
            PortService_var connectedPortRef = connectedPorts[j];
            if(CORBA::is_nil(connectedPortRef)){
                continue;
            }
            PortProfile_var connectedPortProfile;
            try {
                connectedPortProfile = connectedPortRef->get_port_profile();
            }
            catch (CORBA::SystemException& ex) {
                MessageView::instance()->putln(
                    format(_("CORBA {0} ({1}), {2} in RTSComp::connectionCheckSub()"),
                           ex._name(), ex._rep_id(), ex.NP_minorString()),
                    MessageView::WARNING);
                continue;
            }
            string portName = string(connectedPortProfile->name);
            vector<string> target;
            RTCCommonUtil::splitPortName(portName, target);
            if (target[0] == name) continue;

            string rtcPath;
            if(!getComponentPath(connectedPortRef, rtcPath)){
                continue;
            }
            RTSComp* targetRTC = rtsImpl->nameToRTSComp("/" + rtcPath);
            if(!targetRTC){
                continue;
            }

            //DDEBUG("targetRTC Found");
            RTSPort* targetPort = targetRTC->nameToRTSPort(portName);
            if (targetPort) {
                auto itr = rtsImpl->rtsConnections.find(RTSystemItemEx::RTSPortPair(rtsPort, targetPort));
                if (itr != rtsImpl->rtsConnections.end()) {
                    continue;
                }
                DDEBUG("RTSComp::connectionCheckSub Add Con");
                RTSConnectionPtr rtsConnection = new RTSConnection(
                    string(connectorProfile.connector_id), string(connectorProfile.name),
                    name, rtsPort->name,
                    target[0], portName);
                coil::Properties properties = NVUtil::toProperties(connectorProfile.properties);
                vector<NamedValuePtr> propList;
                NamedValuePtr dataType(new NamedValue("dataport.dataflow_type", properties["dataport.dataflow_type"]));
                propList.push_back(dataType);
                NamedValuePtr subscription(new NamedValue("dataport.subscription_type", properties["dataport.subscription_type"]));
                rtsConnection->propList = propList;
                
                rtsConnection->srcRTC = this;
                rtsConnection->sourcePort = nameToRTSPort(rtsConnection->sourcePortName);
                rtsConnection->targetRTC = targetRTC;
                rtsConnection->targetPort = targetRTC->nameToRTSPort(rtsConnection->targetPortName);
                rtsConnection->isAlive_ = true;
                rtsImpl->rtsConnections[RTSystemItemEx::RTSPortPair(rtsPort, targetPort)] = rtsConnection;
                
                rtsItem_->suggestFileUpdate();
                
                updated = true;
            }
        }
    }

    return updated;
}


// Ext2
bool RTSComp::connectionCheckForChecking()
{
    bool updated = false;

    for (auto it = inPorts.begin(); it != inPorts.end(); ++it) {
        if (connectionCheckSubForChecking(*it)) {
            updated = true;
        }
        bool isCon = (*it)->checkConnected();
        if( (*it)->isConnected_ != isCon) {
            updated = true;
            (*it)->isConnected_ = isCon;
        }
    }
    for (auto it = outPorts.begin(); it != outPorts.end(); it++) {
        if (connectionCheckSubForChecking(*it)) {
            updated = true;
        }
        bool isCon = (*it)->checkConnected();
        if( (*it)->isConnected_ != isCon) {
            updated = true;
            (*it)->isConnected_ = isCon;
        }
    }

    //DDEBUG_V("RTSComp::connectionCheckForChecking End %d", updated);
    return updated;
}

bool RTSComp::connectionCheckSubForChecking(RTSPort* rtsPort)
{
    bool updated = false;

    if (isObjectAlive(rtsPort->port) == false) return updated;

    auto rtsImpl = static_cast<RTSystemItemEx*>(rtsItem_)->impl;

    /**
       The get_port_profile() function should not be used here because
       it takes much time when the port has large data and its owner RTC is in a remote host.
       The get_connector_profiles() function does not seem to cause such a problem.
    */
    ConnectorProfileList_var connectorProfiles = rtsPort->port->get_connector_profiles();

    for (CORBA::ULong i = 0; i < connectorProfiles->length(); ++i) {
        ConnectorProfile& connectorProfile = connectorProfiles[i];
        PortServiceList& connectedPorts = connectorProfile.ports;
        for (CORBA::ULong j = 0; j < connectedPorts.length(); ++j) {
            PortService_var connectedPortRef = connectedPorts[j];
            if(CORBA::is_nil(connectedPortRef)){
                continue;
            }
            PortProfile_var connectedPortProfile;
            try {
                connectedPortProfile = connectedPortRef->get_port_profile();
            }
            catch (CORBA::SystemException& ex) {
                MessageView::instance()->putln(
                    format(_("CORBA {0} ({1}), {2} in RTSComp::connectionCheckSub()"),
                           ex._name(), ex._rep_id(), ex.NP_minorString()),
                    MessageView::WARNING);
                continue;
            }
            string portName = string(connectedPortProfile->name);
            vector<string> target;
            RTCCommonUtil::splitPortName(portName, target);
            if (target[0] == name) continue;

            string rtcPath;
            if(!getComponentPath(connectedPortRef, rtcPath)){
                continue;
            }
            RTSComp* targetRTC = rtsImpl->nameToRTSCompForChecking("/" + rtcPath);
            if(!targetRTC){
                continue;
            }

            //DDEBUG("targetRTC Found");
            RTSPort* targetPort = targetRTC->orgComp_->nameToRTSPort(portName);
            if (targetPort) {
                auto itr = rtsImpl->rtsConnectionsCheck.find(RTSystemItemEx::RTSPortPair(rtsPort->orgPort_, targetPort));
                if (itr != rtsImpl->rtsConnectionsCheck.end()) {
                    continue;
                }
                DDEBUG("RTSComp::connectionCheckSub Add Con");
                RTSConnectionPtr rtsConnection = new RTSConnection(
                    string(connectorProfile.connector_id), string(connectorProfile.name),
                    name, rtsPort->name,
                    target[0], portName);
                coil::Properties properties = NVUtil::toProperties(connectorProfile.properties);
                vector<NamedValuePtr> propList;
                NamedValuePtr dataType(new NamedValue("dataport.dataflow_type", properties["dataport.dataflow_type"]));
                propList.push_back(dataType);
                NamedValuePtr subscription(new NamedValue("dataport.subscription_type", properties["dataport.subscription_type"]));
                rtsConnection->propList = propList;
                
                rtsConnection->srcRTC = this->orgComp_;
                rtsConnection->sourcePort = this->orgComp_->nameToRTSPort(rtsConnection->sourcePortName);
                rtsConnection->targetRTC = targetRTC->orgComp_;
                rtsConnection->targetPort = targetRTC->orgComp_->nameToRTSPort(rtsConnection->targetPortName);
                rtsConnection->isAlive_ = true;
                rtsImpl->rtsConnectionsAdded[RTSystemItemEx::RTSPortPair(rtsPort->orgPort_, targetPort)] = rtsConnection;
                
                rtsItem_->suggestFileUpdate();
                
                updated = true;
            }
        }
    }

    DDEBUG_V("RTSComp::connectionCheckSubForChecking End %d", updated);
    return updated;
}

bool RTSComp::getComponentPath(RTC::PortService_ptr source, std::string& out_path)
{
    PortProfile_var portprofile = source->get_port_profile();
    if(!CORBA::is_nil(portprofile->owner)){
        ComponentProfile_var cprofile;
        try {
            cprofile = portprofile->owner->get_component_profile();
        }
        catch (CORBA::SystemException& ex) {
            MessageView::instance()->putln(
                format(_("CORBA {0} ({1}), {2} in RTSComp::getComponentPath()"),
                       ex._name(), ex._rep_id(), ex.NP_minorString()),
                MessageView::WARNING);
            return false;
        }
        NVList properties = cprofile->properties;
        for(int index = 0; index < properties.length(); ++index){
            string strName(properties[index].name._ptr);
            if(strName == "naming.names"){
                const char* nvValue;
                properties[index].value >>= nvValue;
                out_path = string(nvValue);
                return true;
            }
        }
    }
    return false;
}

void RTSComp::moveToRelative(const QPointF& p)
{
    QPointF newPos = pos_ + p;
    if (newPos != pos_) {
        pos_ = newPos;
        rtsItem_->suggestFileUpdate();
    }
}
//////////
void RTSystemItemEx::initializeClass(ExtensionManager* ext)
{
    DDEBUG("RTSystemItem::initializeClass");
    ItemManager& im = ext->itemManager();
    im.registerClass<RTSystemItemEx>(N_("RTSystemItem"));
    im.addCreationPanel<RTSystemItemEx>();
    im.addLoaderAndSaver<RTSystemItemEx>(
        _("RT-System"), "RTS-PROFILE-XML", "xml",
        [](RTSystemItemEx* item, const std::string& filename, std::ostream&, Item*) {
        return item->loadRtsProfile(filename);
    },
        [](RTSystemItemEx* item, const std::string& filename, std::ostream&, Item*) {
        return item->saveRtsProfile(filename);
    });
}


RTSystemItemEx::RTSystemItemEx()
{
    DDEBUG("RTSystemItem::RTSystemItem");
    impl = new RTSystemItemExImpl(this);
}


RTSystemItemExImpl::RTSystemItemExImpl(RTSystemItemEx* self)
    : self(self),
      stateCheckMode(RTSystemItemEx::N_STATE_CHECK_MODES, CNOID_GETTEXT_DOMAIN_NAME),
      pollingThreadType(N_POLLING_THREAD_TYPES, CNOID_GETTEXT_DOMAIN_NAME)
{
    DDEBUG_V("RTSystemItemImpl::initialize cycle:%d", pollingCycle);

    autoConnection = true;

    connectionNo = 0;

    Mapping* config = AppConfig::archive()->openMapping("OpenRTM");
    vendorName = config->get("defaultVendor", "AIST");
    version = config->get("defaultVersion", "1.0.0");

    pollingCycle = 1000;
    
    stateCheckMode.setSymbol(RTSystemItemEx::POLLING_MODE, N_("Polling"));
    stateCheckMode.setSymbol(RTSystemItemEx::MANUAL_MODE, N_("Manual"));
    checkAtLoading = true;
    
#if defined(OPENRTM_VERSION12)
    stateCheckMode.setSymbol(RTSystemItemEx::OBSERVER_MODE, "Observer");
    heartBeatPeriod = config->get("heartBeatPeriod", 500);
#endif

    stateCheckMode.select(RTSystemItemEx::POLLING_MODE);

    pollingThreadType.setSymbol(FOREGROUND, N_("Foreground"));
    pollingThreadType.setSymbol(BACKGROUND, N_("Background"));
    pollingThreadType.select(BACKGROUND);

    isCheckingStatus = false;
    doStatusChecking = false;

    timer.setInterval(pollingCycle);
    timer.setSingleShot(false);
    timeOutConnection.reset(timer.sigTimeout().connect([&](){ checkStatus(); }));
}


RTSystemItemEx::RTSystemItemEx(const RTSystemItemEx& org)
    : Item(org)
{
    impl = new RTSystemItemExImpl(this, *org.impl);
}


RTSystemItemExImpl::RTSystemItemExImpl(RTSystemItemEx* self, const RTSystemItemExImpl& org)
    : RTSystemItemExImpl(self)
{
    autoConnection = org.autoConnection;
    vendorName = org.vendorName;
    version = org.version;
    pollingCycle = org.pollingCycle;
    stateCheckMode = org.stateCheckMode;
    pollingThreadType = org.pollingThreadType;
}


RTSystemItemEx::~RTSystemItemEx()
{
    delete impl;
}

RTSystemItemExImpl::~RTSystemItemExImpl()
{
    locationChangedConnection.disconnect();
}


Item* RTSystemItemEx::doDuplicate() const
{
    return new RTSystemItemEx(*this);
}

void RTSystemItemExImpl::onLocationChanged(string host, int port)
{
    NameServerManager::instance()->getNCHelper()->setLocation(host, port);
}

void RTSystemItemEx::onActivated()
{
    impl->onActivated();
}

void RTSystemItemExImpl::onActivated()
{
    DDEBUG_V("RTSystemItemExImpl::onActivated %d, %d", stateCheckMode.selectedIndex());
    if(stateCheckMode.is(RTSystemItemEx::POLLING_MODE)){
        DDEBUG("RTSystemItemExImpl::onActivated timer.start");
        timer.start();
    }
}

RTSComp* RTSystemItemEx::nameToRTSComp(const string& name)
{
    return impl->nameToRTSComp(name);
}

RTSComp* RTSystemItemExImpl::nameToRTSComp(const string& name)
{
    //DDEBUG_V("RTSystemItemImpl::nameToRTSComp:%s", name.c_str());
    map<string, RTSCompPtr>::iterator it = rtsComps.find(name);
    if (it == rtsComps.end())
        return 0;
    else
        return it->second.get();
}


RTSComp* RTSystemItemExImpl::nameToRTSCompForChecking(const string& name)
{
    map<string, RTSCompPtr>::iterator it = rtsCompsCheck.find(name);
    if (it == rtsCompsCheck.end())
        return 0;
    else
        return it->second.get();
}


RTSComp* RTSystemItemEx::addRTSComp(const string& name, const QPointF& pos)
{
    return impl->addRTSComp(name, pos);
}


RTSComp* RTSystemItemExImpl::addRTSComp(const string& name, const QPointF& pos)
{
    DDEBUG_V("RTSystemItemImpl::addRTSComp:%s", name.c_str());

    if (!nameToRTSComp("/" + name + ".rtc")) {
        std::vector<NamingContextHelper::ObjectPath> pathList;
        NamingContextHelper::ObjectPath path(name, "rtc");
        pathList.push_back(path);
        NamingContextHelper* ncHelper = NameServerManager::instance()->getNCHelper();
        RTC::RTObject_ptr rtc = ncHelper->findObject<RTC::RTObject>(pathList);
        DDEBUG_V("ncHelper host:%s, port:%d", ncHelper->host().c_str(), ncHelper->port());
        if (rtc == RTC::RTObject::_nil()) {
            DDEBUG("RTSystemItemImpl::addRTSComp Failed");
            return nullptr;
        }

        string fullPath = "/" + name + ".rtc";
        RTSCompPtr rtsComp = new RTSComp(name, fullPath, rtc, self, pos, ncHelper->host().c_str(), ncHelper->port(), false);
        rtsComps[fullPath] = rtsComp;

        self->suggestFileUpdate();

        return rtsComp;
    }

    return nullptr;
}


RTSComp* RTSystemItemEx::addRTSComp(const  NamingContextHelper::ObjectInfo& info, const QPointF& pos)
{
    return impl->addRTSComp(info, pos);
}


RTSComp* RTSystemItemExImpl::addRTSComp(const NamingContextHelper::ObjectInfo& info, const QPointF& pos)
{
    DDEBUG("RTSystemItemImpl::addRTSComp");
    timeOutConnection.block();

    string fullPath = info.getFullPath();

    if (!nameToRTSComp(fullPath)) {
        std::vector<NamingContextHelper::ObjectPath> target = info.fullPath_;
        auto ncHelper = NameServerManager::instance()->getNCHelper();
        if(info.isRegisteredInRtmDefaultNameServer_) {
            NameServerInfo ns = RTCCommonUtil::getManagerAddress();
            ncHelper->setLocation(ns.hostAddress, ns.portNo);
        } else {
            ncHelper->setLocation(info.hostAddress_, info.portNo_);
        }
        RTC::RTObject_ptr rtc = ncHelper->findObject<RTC::RTObject>(target);
        if (!isObjectAlive(rtc)) {
            CORBA::release(rtc);
            rtc = nullptr;
        }

        RTSCompPtr rtsComp = new RTSComp(info.id_, fullPath, rtc, self, pos, info.hostAddress_, info.portNo_, info.isRegisteredInRtmDefaultNameServer_);
        rtsComps[fullPath] = rtsComp;

        self->suggestFileUpdate();

        return rtsComp.get();
    }
    timeOutConnection.unblock();

    return nullptr;
}

void RTSystemItemEx::deleteRTSComp(const string& name)
{
    impl->deleteRTSComp(name);
}


void RTSystemItemExImpl::deleteRTSComp(const string& name)
{
    timeOutConnection.block();

    if (rtsComps.erase(name) > 0) {
        self->suggestFileUpdate();
    }

    timeOutConnection.unblock();
}


bool RTSystemItemExImpl::compIsAlive(RTSComp* rtsComp)
{
    //DDEBUG("RTSystemItemImpl::compIsAlive");
    if (rtsComp->isAlive_ && rtsComp->rtc_ && rtsComp->rtc_ != nullptr) {
        if (isObjectAlive(rtsComp->rtc_)) {
            return true;
        } else {
            rtsComp->setRtc(nullptr);
            return false;
        }
    } else {
        //DDEBUG_V("Full Path = %s", rtsComp->fullPath.c_str());
        QStringList nameList = QString::fromStdString(rtsComp->fullPath).split("/");
        std::vector<NamingContextHelper::ObjectPath> pathList;
        for (int index = 0; index < nameList.count(); index++) {
            QString elem = nameList[index];
            if (elem.length() == 0) continue;
            QStringList elemList = elem.split(".");
            if (elemList.size() != 2) return false;
            NamingContextHelper::ObjectPath path(elemList[0].toStdString(), elemList[1].toStdString());
            pathList.push_back(path);
        }

        std::string host;
        int port;
        if(rtsComp->isDefaultNS) {
            NameServerInfo ns = RTCCommonUtil::getManagerAddress();
            host = ns.hostAddress;
            port = ns.portNo;
        } else {
            host = rtsComp->hostAddress;
            port = rtsComp->portNo;
        }
        DDEBUG_V("host:%s, port:%d, default:%d", host.c_str(), port, rtsComp->isDefaultNS);
        NameServerManager::instance()->getNCHelper()->setLocation(host, port);
        RTC::RTObject_ptr rtc = NameServerManager::instance()->getNCHelper()->findObject<RTC::RTObject>(pathList);
        if (!isObjectAlive(rtc)) {
            //DDEBUG("RTSystemItemImpl::compIsAlive NOT Alive");
            return false;
        } else {
            DDEBUG("RTSystemItemImpl::compIsAlive Alive");
            rtsComp->setRtc(rtc);
            if (autoConnection) {
                list<RTSConnection*> rtsConnectionList;
                RTSCompToConnectionList(rtsComp, rtsConnectionList, 0);
                for (auto it = rtsConnectionList.begin(); it != rtsConnectionList.end(); it++) {
                    auto connection = *it;
                    connection->connect();
                }
                DDEBUG("autoConnection End");
            }
            return true;
        }
    }
}


bool RTSystemItemExImpl::compIsAliveForChecking(RTSComp* rtsComp)
{
    DDEBUG("RTSystemItemImpl::compIsAliveForChecking");
    rtsComp->isSetRtc_ = false;
    rtsComp->rtcCheck_ = 0;
    if (rtsComp->isAlive_ && rtsComp->rtc_ && rtsComp->rtc_ != nullptr) {
        if (isObjectAlive(rtsComp->rtc_)) {
            rtsComp->isAlive_ = true;
            return true;
        } else {
            rtsComp->isSetRtc_ = true;
            rtsComp->rtcCheck_ = nullptr;
            rtsComp->isAlive_ = false;
            return false;
        }
    } else {
        DDEBUG_V("Full Path = %s", rtsComp->fullPath.c_str());
        QStringList nameList = QString::fromStdString(rtsComp->fullPath).split("/");
        std::vector<NamingContextHelper::ObjectPath> pathList;
        for (int index = 0; index < nameList.count(); index++) {
            QString elem = nameList[index];
            if (elem.length() == 0) continue;
            QStringList elemList = elem.split(".");
            if (elemList.size() != 2) return false;
            NamingContextHelper::ObjectPath path(elemList[0].toStdString(), elemList[1].toStdString());
            pathList.push_back(path);
        }

        std::string host;
        int port;
        if(rtsComp->isDefaultNS) {
            NameServerInfo ns = RTCCommonUtil::getManagerAddress();
            host = ns.hostAddress;
            port = ns.portNo;
        } else {
            host = rtsComp->hostAddress;
            port = rtsComp->portNo;
        }
        DDEBUG_V("host:%s, port:%d, default:%d", host.c_str(), port, rtsComp->isDefaultNS);
        NameServerManager::instance()->getNCHelper()->setLocation(host, port);
        RTC::RTObject_ptr rtc = NameServerManager::instance()->getNCHelper()->findObject<RTC::RTObject>(pathList);
        if (!isObjectAlive(rtc)) {
            rtsComp->isAlive_ = false;
            return false;
        } else {
            rtsComp->isSetRtc_ = true;
            rtsComp->rtcCheck_ = rtc;
            rtsComp->isAlive_ = true;
            return true;
        }
    }
}

string RTSystemItemExImpl::getConnectionNumber()
{
    stringstream ss;
    ss << connectionNo;
    connectionNo++;
    return ss.str();
}


RTSConnection* RTSystemItemEx::addRTSConnection
(const std::string& id, const std::string& name, RTSPort* sourcePort, RTSPort* targetPort, const std::vector<NamedValuePtr>& propList, const Vector2 pos[])
{
    bool setPos = true;
    if (!pos) setPos = false;
    return impl->addRTSConnection(id, name, sourcePort, targetPort, propList, setPos, pos);
}


RTSConnection* RTSystemItemExImpl::addRTSConnection
(const string& id, const string& name,
 RTSPort* sourcePort, RTSPort* targetPort, const std::vector<NamedValuePtr>& propList,
 const bool setPos, const Vector2 pos[])
{
    DDEBUG("RTSystemItemImpl::addRTSConnection");
    timeOutConnection.block();

    bool updated = false;

    RTSConnection* rtsConnection_;
    auto it = rtsConnections.find(RTSystemItemEx::RTSPortPair(sourcePort, targetPort));

    if (it != rtsConnections.end()) {
        rtsConnection_ = it->second;;

    } else {
        RTSConnectionPtr rtsConnection =
            new RTSConnection(
                id, name,
                sourcePort->rtsComp->name, sourcePort->name,
                targetPort->rtsComp->name, targetPort->name);

        rtsConnection->srcRTC = sourcePort->rtsComp;
        rtsConnection->sourcePort = sourcePort;
        rtsConnection->targetRTC = targetPort->rtsComp;
        rtsConnection->targetPort = targetPort;
        rtsConnection->propList = propList;

        if (setPos) {
            rtsConnection->setPosition(pos);
        }

        rtsConnections[RTSystemItemEx::RTSPortPair(sourcePort, targetPort)] = rtsConnection;
        rtsConnection_ = rtsConnection;

        updated = true;
    }

    if (!CORBA::is_nil(sourcePort->port) && !sourcePort->port->_non_existent() &&
       !CORBA::is_nil(targetPort->port) && !targetPort->port->_non_existent()) {
        if (rtsConnection_->connect()) {
            updated = true;
        }
    }

    if (rtsConnection_->id.empty()) {
        rtsConnection_->id = "NoConnection_" + getConnectionNumber();
        updated = true;
    }

    if (updated) {
        self->suggestFileUpdate();
    }

    timeOutConnection.unblock();
    return rtsConnection_;
}


RTSConnection* RTSystemItemExImpl::addRTSConnectionName
(const string& id, const string& name,
 const string& sourceCompName, const string& sourcePortName,
 const string& targetCompName, const string& targetPortName,
 const string& dataflow, const string& subscription,
 const bool setPos, const Vector2 pos[])
{
    string sourceId = "/" + sourceCompName + ".rtc";
    RTSPort* sourcePort = 0;
    RTSComp* sourceRtc = nameToRTSComp(sourceId);
    if (sourceRtc) {
        sourcePort = sourceRtc->nameToRTSPort(sourcePortName);
    }

    string targetId = "/" + targetCompName + ".rtc";
    RTSPort* targetPort = 0;
    RTSComp* targetRtc = nameToRTSComp(targetId);
    if (targetRtc) {
        targetPort = targetRtc->nameToRTSPort(targetPortName);
    }
    if (sourcePort && targetPort) {
        vector<NamedValuePtr> propList;
        NamedValuePtr paramDataFlow(new NamedValue("dataport.dataflow_type", dataflow));
        propList.push_back(paramDataFlow);
        NamedValuePtr paramSubscription(new NamedValue("dataport.subscription_type", subscription));
        propList.push_back(paramSubscription);
        NamedValuePtr sinterfaceProp(new NamedValue("dataport.interface_type", "corba_cdr"));
        propList.push_back(sinterfaceProp);

        return addRTSConnection(id, name, sourcePort, targetPort, propList, setPos, pos);
    }

    return nullptr;
}

// Ext1
bool RTSystemItemExImpl::connectionCheck()
{
    bool updated = false;

    for (auto it = rtsConnections.begin(); it != rtsConnections.end(); it++) {
        const RTSystemItemEx::RTSPortPair& ports = it->first;
        if (ports(0)->isConnectedWith(ports(1))) {
            if (!it->second->isAlive_) {
                it->second->isAlive_ = true;
                updated = true;
            }
        } else {
            if (it->second->isAlive_) {
                it->second->isAlive_ = false;
                updated = true;
            }
        }
    }

    for (auto it = rtsComps.begin(); it != rtsComps.end(); it++) {
        if (it->second->connectionCheck()) {
            updated = true;
        }
    }

    return updated;
}


void RTSystemItemEx::RTSCompToConnectionList
(const RTSComp* rtsComp, list<RTSConnection*>& rtsConnectionList, int mode)
{
    impl->RTSCompToConnectionList(rtsComp, rtsConnectionList, mode);
}


void RTSystemItemExImpl::RTSCompToConnectionList
(const RTSComp* rtsComp, list<RTSConnection*>& rtsConnectionList, int mode)
{
    for (RTSystemItemEx::RTSConnectionMap::iterator it = rtsConnections.begin();
            it != rtsConnections.end(); it++) {
        switch (mode) {
            case 0:
            default:
                if (it->second->sourceRtcName == rtsComp->name || it->second->targetRtcName == rtsComp->name)
                    rtsConnectionList.push_back(it->second);
                break;
            case 1:
                if (it->second->sourceRtcName == rtsComp->name)
                    rtsConnectionList.push_back(it->second);
                break;
            case 2:
                if (it->second->targetRtcName == rtsComp->name)
                    rtsConnectionList.push_back(it->second);
                break;
        }
    }
}


map<string, RTSCompPtr>& RTSystemItemEx::rtsComps()
{
    return impl->rtsComps;
}


RTSystemItemEx::RTSConnectionMap& RTSystemItemEx::rtsConnections()
{
    return impl->rtsConnections;
}


void RTSystemItemEx::disconnectAndRemoveConnection(RTSConnection* connection)
{
    connection->disconnect();
    impl->removeConnection(connection);
}


void RTSystemItemExImpl::removeConnection(RTSConnection* connection)
{
    RTSystemItemEx::RTSPortPair pair(connection->sourcePort, connection->targetPort);
    if (rtsConnections.erase(pair) > 0) {
        self->suggestFileUpdate();
    }
}


void RTSystemItemEx::doPutProperties(PutPropertyFunction& putProperty)
{
    impl->doPutProperties(putProperty);
}


void RTSystemItemExImpl::doPutProperties(PutPropertyFunction& putProperty)
{
    DDEBUG("RTSystemItemImpl::doPutProperties");
    putProperty(_("Auto connection"), autoConnection, changeProperty(autoConnection));
    putProperty(_("Vendor name"), vendorName, changeProperty(vendorName));
    putProperty(_("Version"), version, changeProperty(version));
    putProperty(_("State check mode"), stateCheckMode,
                [&](int mode) { setStateCheckMode(mode); return true; });
    putProperty(_("Polling cycle"), pollingCycle,
                [&](int mode) { changePollingPeriod(mode); return true; });
    putProperty(_("Polling thread"), pollingThreadType, changeProperty(pollingThreadType));
    putProperty(_("Check at loading"), checkAtLoading, changeProperty(checkAtLoading));

#if defined(OPENRTM_VERSION12)
    putProperty(_("Heart-beat period"), heartBeatPeriod, changeProperty(heartBeatPeriod));
#endif
}

void RTSystemItemExImpl::changePollingPeriod(int value)
{
    DDEBUG_V("RTSystemItemImpl::changePollingPeriod=%d", value);
    if (pollingCycle != value) {
        pollingCycle = value;

        bool isStarted = timer.isActive();
        if( isStarted ) timer.stop();
        timer.setInterval(value);
        if( isStarted ) timer.start();
    }
}

void RTSystemItemExImpl::setStateCheckMode(int mode)
{
    DDEBUG_V("RTSystemItemImpl::setStateCheckMode=%d", mode);
    stateCheckMode.selectIndex(mode);
    changeStateCheckMode();
}

void RTSystemItemExImpl::setStateCheckModeByString(const string& mode)
{
    DDEBUG_V("RTSystemItemImpl::setStateCheckModeByString=%s", value.c_str());
    stateCheckMode.select(mode);
    DDEBUG_V("RTSystemItemImpl::setStateCheckModeByString=%d", stateCheckMode.selectedIndex());
    changeStateCheckMode();
}

void RTSystemItemExImpl::changeStateCheckMode()
{
    int state = stateCheckMode.selectedIndex();
    DDEBUG_V("RTSystemItemImpl::changeStateCheckMode=%d", state);
    switch (state) {
        case RTSystemItemEx::MANUAL_MODE:
            timer.stop();
            break;
#if defined(OPENRTM_VERSION12)
        case RTSystemItemEx::OBSERVER_MODE:
            break;
#endif
        default:
            timer.start();
            break;
    }
}

bool RTSystemItemEx::loadRtsProfile(const string& filename)
{
    DDEBUG_V("RTSystemItem::loadRtsProfile=%s", filename.c_str());
    ProfileHandler::getRtsProfileInfo(filename, impl->vendorName, impl->version);
    if (ProfileHandler::restoreRtsProfile(filename, this)) {
        notifyUpdate();
        return true;
    }
    return false;
}


bool RTSystemItemEx::saveRtsProfile(const string& filename)
{
    if (isConsistentWithFile()) return true;
    return impl->saveRtsProfile(filename);
}


bool RTSystemItemExImpl::saveRtsProfile(const string& filename)
{
    if (vendorName.empty()) {
        vendorName = "Choreonoid";
    }
    if (version.empty()) {
        version = "1.0.0";
    }
    string systemId = "RTSystem:" + vendorName + ":" + self->name() + ":" + version;
    ProfileHandler::saveRtsProfile(filename, systemId, rtsComps, rtsConnections, MessageView::mainInstance()->cout());

    return true;
}

void RTSystemItemEx::setVendorName(const std::string& name)
{
    impl->vendorName = name;
}


void RTSystemItemEx::setVersion(const std::string& version)
{
    impl->version = version;
}

int RTSystemItemEx::stateCheckMode() const
{
    return impl->stateCheckMode.selectedIndex();
}

void RTSystemItemEx::checkStatus()
{
    impl->checkStatus();
}

void RTSystemItemExImpl::checkStatus()
{
    if(pollingThreadType.is(FOREGROUND)){
        checkStatusForeground();
    } else {
        checkStatusBackground();
    }
}

void RTSystemItemExImpl::checkStatusForeground()
{
    DDEBUG("RTSystemItemExImpl::checkStatusForeground");
    if( sigStatusUpdate.empty() ) {
        timer.stop();
    }

    if(isCheckingStatus){
        // Background polling is not finished
        return;
    }

    bool modified = false;

    for (auto it = rtsComps.begin(); it != rtsComps.end(); it++) {
        if (compIsAlive(it->second)) {
            if (!it->second->isAlive_) {
                modified = true;
            }
            it->second->isAlive_ = true;
            RTC_STATUS status = it->second->getRTCState();
            DDEBUG_V("RTC State : %d, %d", it->second->rtc_status_, status);
            if( status != it->second->rtc_status_ ) {
                modified = true;
                it->second->rtc_status_ = status;
            }
        } else {
            if (it->second->isAlive_) {
                modified = true;
            }
            it->second->isAlive_ = false;
        }
    }
    //
    if (connectionCheck()) {
        modified = true;
    }
    DDEBUG_V("RTSystemItemExImpl::checkStatusForeground End : %d", modified);
    sigStatusUpdate(modified);
}
    
void RTSystemItemExImpl::checkStatusBackground()
{
    DDEBUG("RTSystemItemExImpl::checkStatusBackground");
    if( sigStatusUpdate.empty() ) {
        DDEBUG("RTSystemItemExImpl::checkStatusBackground timer.stop");
        timer.stop();
    }
    //
    if (isCheckingStatus) {
        DDEBUG("RTSystemItemExImpl::checkStatusBackground isCheckingStatus");
        doStatusChecking = true;
        return;
    }
    isCheckingStatus = true;
    copyForStatusChecking();
    pollingThread = std::thread([&](){ statusChecking(); });
}

void RTSystemItemExImpl::statusChecking()
{
    statusModified = false;

    DDEBUG("RTSystemItemExImpl::statusChecking start");
    for (auto it = rtsCompsCheck.begin(); it != rtsCompsCheck.end(); it++) {
        if (compIsAliveForChecking(it->second)) {
            if (!it->second->isAlive_) {
              DDEBUG("RTSystemItemExImpl::statusChecking 1");
              statusModified = true;
            }
            RTC_STATUS status;
            if (it->second->isSetRtc_) {
                status = RTC_UNKNOWN;
            } else {
                status = it->second->getRTCState();
            }
            DDEBUG_V("RTC State : %d, %d", it->second->rtc_status_, status);
            if (status != it->second->rtc_status_) {
              DDEBUG("RTSystemItemExImpl::statusChecking 2");
              statusModified = true;
              it->second->rtc_status_ = status;
            }
        } else {
            if (it->second->isAlive_) {
                DDEBUG("RTSystemItemExImpl::statusChecking 3");
                statusModified = true;
            }
        }
    }
    ///
    DDEBUG("RTSystemItemExImpl::statusChecking rtsConnectionsCheck");
    for (auto it = rtsConnectionsCheck.begin(); it != rtsConnectionsCheck.end(); it++) {
        const RTSystemItemEx::RTSPortPair& ports = it->first;

        if (ports(0)->isConnectedWith(ports(1))) {
            if (!it->second->isAlive_) {
                it->second->isAlive_ = true;
                DDEBUG("RTSystemItemExImpl::statusChecking 4");
                statusModified = true;
            }
        } else {
            if (it->second->isAlive_) {
                it->second->isAlive_ = false;
                DDEBUG("RTSystemItemExImpl::statusChecking 5");
                statusModified = true;
            }
        }
    }

    DDEBUG("RTSystemItemExImpl::statusChecking rtsCompsCheck");
    for (auto it = rtsCompsCheck.begin(); it != rtsCompsCheck.end(); it++) {
        if (it->second->connectionCheckForChecking()) {
              DDEBUG("RTSystemItemExImpl::statusChecking 6");
            statusModified = true;
        }
    }
    //////////
    DDEBUG("RTSystemItemExImpl::statusChecking finish");
    callLater([&](){ restoreForStatusChecking(); });
}

void RTSystemItemExImpl::copyForStatusChecking()
{
    DDEBUG("RTSystemItemExImpl::copyForStatusChecking");
    rtsCompsCheck.clear();
    for (auto it = rtsComps.begin(); it != rtsComps.end(); it++) {
        if(it->second) rtsCompsCheck[it->first] = it->second->copyForChecking();
    }
    rtsConnectionsCheck.clear();
    for (auto it = rtsConnections.begin(); it != rtsConnections.end(); it++) {
        if(it->second) rtsConnectionsCheck[it->first] = it->second->copyForChecking();
    }
    rtsConnectionsAdded.clear();
    DDEBUG("RTSystemItemExImpl::copyForStatusChecking End");
}

void RTSystemItemExImpl::restoreForStatusChecking()
{
    DDEBUG("RTSystemItemExImpl::restoreForStatusChecking");
    for (auto it = rtsCompsCheck.begin(); it != rtsCompsCheck.end(); it++) {
        RTSCompPtr targetComp = rtsComps[it->first];
        if (targetComp) {
            RTSCompPtr sourceComp = rtsCompsCheck[it->first];
            targetComp->rtc_status_ = sourceComp->rtc_status_;
            if (sourceComp->isSetRtc_) {
                targetComp->setRtc(sourceComp->rtcCheck_);
                if (autoConnection) {
                    list<RTSConnection*> rtsConnectionList;
                    RTSCompToConnectionList(targetComp, rtsConnectionList, 0);
                    for (auto it = rtsConnectionList.begin(); it != rtsConnectionList.end(); it++) {
                        auto connection = *it;
                        connection->connect();
                    }
                    DDEBUG("autoConnection End");
                }
            }
            //
            for (auto itTarget = targetComp->inPorts.begin(); itTarget != targetComp->inPorts.end(); itTarget++) {
                for (auto itSource = sourceComp->inPorts.begin(); itSource != sourceComp->inPorts.end(); itSource++) {
                    if ((*itSource)->name == (*itTarget)->name) {
                        (*itTarget)->isConnected_ = (*itSource)->checkConnected();
                        DDEBUG_V("InPort %s,%d", (*itTarget)->name.c_str(), (*itTarget)->isConnected_);
                        break;
                    }
                }
            }
            for (auto itTarget = targetComp->outPorts.begin(); itTarget != targetComp->outPorts.end(); itTarget++) {
                for (auto itSource = sourceComp->outPorts.begin(); itSource != sourceComp->outPorts.end(); itSource++) {
                    if ((*itSource)->name == (*itTarget)->name) {
                        (*itTarget)->isConnected_ = (*itSource)->checkConnected();
                        DDEBUG_V("OutPort %s,%d", (*itTarget)->name.c_str(), (*itTarget)->isConnected_);
                        break;
                    }
                }
            }
        }
    }
    //
    for (auto it = rtsConnectionsCheck.begin(); it != rtsConnectionsCheck.end(); it++) {
        if (0 < rtsConnections.count(it->first)) {
            RTSConnectionPtr targetConn = rtsConnections[it->first];
            RTSConnectionPtr sourceConn = rtsConnectionsCheck[it->first];
            targetConn->isAlive_ = sourceConn->isAlive_;
        }
    }
    for (auto it = rtsConnectionsAdded.begin(); it != rtsConnectionsAdded.end(); it++) {
        rtsConnections[it->first] = it->second;
    }

    sigStatusUpdate(statusModified);

    isCheckingStatus = false;
    pollingThread.detach();

    const bool doContinuousCheck = false;
    if(doContinuousCheck){
        if(doStatusChecking){
            doStatusChecking = false;
            checkStatus();
        }
    }
}

bool RTSystemItemEx::isCheckAtLoading()
{
    return impl->checkAtLoading;
}
///////////
bool RTSystemItemEx::store(Archive& archive)
{
    if (overwrite()) {
        archive.writeRelocatablePath("filename", filePath());
        archive.write("format", fileFormat());

        archive.write("autoConnection", impl->autoConnection);
        archive.write("pollingCycle", impl->pollingCycle);
        archive.write("stateCheckMode", impl->stateCheckMode.selectedSymbol());
        archive.write("pollingThread", impl->pollingThreadType.selectedSymbol());
        archive.write("checkAtLoading", impl->checkAtLoading);

#if defined(OPENRTM_VERSION12)
        archive.write("HeartBeatPeriod", impl->heartBeatPeriod);
#endif
        return true;
    }

    return true;
}


bool RTSystemItemEx::restore(const Archive& archive)
{
    DDEBUG("RTSystemItemEx::restore");

    if(!archive.read("autoConnection", impl->autoConnection)){
        archive.read("AutoConnection", impl->autoConnection);
    }

    int pollingCycle = 1000;
    if(!archive.read("pollingCycle", pollingCycle)){
        archive.read("PollingCycle", pollingCycle);
    }
    if(!archive.read("checkAtLoading", impl->checkAtLoading)){
        archive.read("CheckAtLoading", impl->checkAtLoading);
    }

    impl->changePollingPeriod(pollingCycle);

#if defined(OPENRTM_VERSION12)
    if(!archive.read("HeartBeatPeriod", impl->heartBeatPeriod)){
        archive.read("heartBeatPeriod", impl->heartBeatPeriod);
    }
#endif

    /**
       The contents of RTSystemItem must be loaded after all the items are restored
       so that the states of the RTCs created by other items can be loaded.
    */
    std::string filename, formatId;
    if (archive.readRelocatablePath("filename", filename)) {
        if (archive.read("format", formatId)) {
            archive.addPostProcess(
                [this, filename, formatId]() { load(filename, formatId); });
        }
    } else {
        // old format data contained in a project file
        archive.addPostProcess([&]() { impl->restoreRTSystem(archive); });
    }

    string symbol;
    if(archive.read("stateCheckMode", symbol) || archive.read("stateCheck", symbol)){
        DDEBUG_V("StateCheckMode:%s", symbol.c_str());
        impl->setStateCheckModeByString(symbol);
        archive.addPostProcess([&]() { impl->changeStateCheckMode(); });
    }

    if(archive.read("pollingThread", symbol)){
        impl->pollingThreadType.select(symbol);
    }

    return true;
}


void RTSystemItemExImpl::restoreRTSystem(const Archive& archive)
{
    DDEBUG("RTSystemItemImpl::restoreRTSystem");

    const Listing& compListing = *archive.findListing("RTSComps");
    if (compListing.isValid()) {
        for (int i = 0; i < compListing.size(); i++) {
            const Mapping& compMap = *compListing[i].toMapping();
            string name;
            Vector2 pos;
            compMap.read("name", name);
            read(compMap, "pos", pos);

            vector<pair<string, bool>> inPorts;
            vector<pair<string, bool>> outPorts;
            const Listing& inportListing = *compMap.findListing("InPorts");
            if (inportListing.isValid()) {
                for (int i = 0; i < inportListing.size(); i++) {
                    const Mapping& inMap = *inportListing[i].toMapping();
                    string portName;
                    bool isServicePort;
                    inMap.read("name", portName);
                    inMap.read("isServicePort", isServicePort);
                    inPorts.push_back(make_pair(portName, isServicePort));
                }
            }
            const Listing& outportListing = *compMap.findListing("OutPorts");
            if (outportListing.isValid()) {
                for (int i = 0; i < outportListing.size(); i++) {
                    const Mapping& outMap = *outportListing[i].toMapping();
                    string portName;
                    bool isServicePort;
                    outMap.read("name", portName);
                    outMap.read("isServicePort", isServicePort);
                    outPorts.push_back(make_pair(portName, isServicePort));
                }
            }
            restoreRTSComp(name, pos, inPorts, outPorts);
        }
    }

    if (autoConnection) {
        const Listing& connectionListing = *archive.findListing("RTSConnections");
        if (connectionListing.isValid()) {
            for (int i = 0; i < connectionListing.size(); i++) {
                const Mapping& connectMap = *connectionListing[i].toMapping();
                string name, sR, sP, tR, tP, dataflow, subscription;
                connectMap.read("name", name);
                connectMap.read("sourceRtcName", sR);
                connectMap.read("sourcePortName", sP);
                connectMap.read("targetRtcName", tR);
                connectMap.read("targetPortName", tP);
                connectMap.read("dataflow", dataflow);
                connectMap.read("subscription", subscription);
                VectorXd p(12);
                bool readPos = false;
                Vector2 pos[6];
                if (read(connectMap, "position", p)) {
                    readPos = true;
                    for (int i = 0; i < 6; i++) {
                        pos[i] << p(2 * i), p(2 * i + 1);
                    }
                }

                addRTSConnectionName("", name, sR, sP, tR, tP, dataflow, subscription, readPos, pos);
            }
        }
    }

    if (checkAtLoading) {
        checkStatus();
    }

    self->notifyUpdate();
    DDEBUG("RTSystemItemImpl::restoreRTSystem End");
}


void RTSystemItemExImpl::restoreRTSComp(const string& name, const Vector2& pos,
        const vector<pair<string, bool>>& inPorts, const vector<pair<string, bool>>& outPorts)
{
    DDEBUG("RTSystemItemImpl::restoreRTSComp");

    RTSComp* comp = addRTSComp(name, QPointF(pos(0), pos(1)));
    if (comp == 0) return;
    if (!comp->rtc_) {
        comp->inPorts.clear();
        comp->outPorts.clear();
        for (int i = 0; i < inPorts.size(); i++) {
            RTSPortPtr rtsPort = new RTSPort(inPorts[i].first, 0, comp);
            rtsPort->isInPort = true;
            rtsPort->isServicePort = inPorts[i].second;
            comp->inPorts.push_back(rtsPort);
        }
        for (int i = 0; i < outPorts.size(); i++) {
            RTSPortPtr rtsPort = new RTSPort(outPorts[i].first, 0, comp);
            rtsPort->isInPort = false;
            rtsPort->isServicePort = outPorts[i].second;
            comp->outPorts.push_back(rtsPort);
        }
    }
    DDEBUG("RTSystemItemImpl::restoreRTSComp End");
}

SignalProxy<void(bool)> RTSystemItemEx::sigStatusUpdate()
{
    return impl->sigStatusUpdate;
}
