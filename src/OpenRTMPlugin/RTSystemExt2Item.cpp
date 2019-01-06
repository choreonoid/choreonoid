#include "RTSystemExt2Item.h"
#include "RTSCommonUtil.h"
#include "RTSTypeUtilEXT2.h"
#include "ProfileHandlerEXT2.h"
#include <cnoid/MessageView>
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/EigenArchive>
#include <cnoid/AppConfig>
#include <cnoid/Timer>
#include <cnoid/LazyCaller>
#include <boost/format.hpp>
#include "LoggerUtil.h"
#include <mutex>

#include "gettext.h"

using namespace cnoid;
using namespace std;
using namespace std::placeholders;
using namespace RTC;
using boost::format;

namespace cnoid {

struct RTSPortComparator
{
    std::string name_;

    RTSPortComparator(std::string value)
    {
        name_ = value;
    }
    bool operator()(const RTSPortExt2Ptr elem) const
    {
        return (name_ == elem->name);
    }
};
//////////
class RTSystemExt2ItemImpl
{
public:
    map<string, RTSCompExt2Ptr> rtsComps;
    RTSystemExt2Item::RTSConnectionMap rtsConnections;
    bool autoConnection;

    map<string, RTSCompExt2Ptr> rtsCompsCheck;
    RTSystemExt2Item::RTSConnectionMap rtsConnectionsCheck;
    bool isStatusChecking;
    RTSystemExt2Item::RTSConnectionMap rtsConnectionsAdded;

    std::string vendorName;
    std::string version;
    int pollingCycle;
    bool checkAtLoading;

#if defined(OPENRTM_VERSION12)
    int heartBeatPeriod;
#endif

    enum State_Detection
    {
        POLLING_CHECK = 0,
        MANUAL_CHECK,
#if defined(OPENRTM_VERSION12)
        OBSERVER_CHECK,
#endif
        N_STATE_DETECTION
    };
    Selection stateCheck;

    RTSystemExt2ItemImpl(RTSystemExt2Item* self);
    RTSystemExt2ItemImpl(RTSystemExt2Item* self, const RTSystemExt2ItemImpl& org);
    ~RTSystemExt2ItemImpl();

    void initialize();
    void onLocationChanged(std::string host, int port);
    RTSCompExt2* addRTSComp(const string& name, const QPointF& pos);
    RTSCompExt2* addRTSComp(const NamingContextHelper::ObjectInfo& info, const QPointF& pos);
    void deleteRTSComp(const string& name);
    RTSCompExt2* nameToRTSComp(const string& name);
    RTSCompExt2* nameToRTSCompForChecking(const string& name);
    bool compIsAliveForChecking(RTSCompExt2* rtsComp);
    RTSConnectionExt2* addRTSConnection(
        const string& id, const string& name,
        RTSPortExt2* sourcePort, RTSPortExt2* targetPort, const std::vector<NamedValueExt2Ptr>& propList,
        const bool setPos, const Vector2 pos[]);
    RTSConnectionExt2* addRTSConnectionName(
        const string& id, const string& name,
        const string& sourceComp, const string& sourcePort,
        const string& targetComp, const string& targetPort,
        const string& dataflow, const string& subscription,
        const bool setPos, const Vector2 pos[]);
    void RTSCompToConnectionList(const RTSCompExt2* rtsComp, list<RTSConnectionExt2*>& rtsConnectionList, int mode);
    void removeConnection(RTSConnectionExt2* connection);
    void doPutProperties(PutPropertyFunction& putProperty);
    bool saveRtsProfile(const string& filename);
    void restoreRTSystem(const Archive& archive);
    void restoreRTSComp(const string& name, const Vector2& pos,
            const vector<pair<string, bool>>& inPorts, const vector<pair<string, bool>>& outPorts);
    string getConnectionNumber();
    void setStateCheckMethodByString(const string& value);
    void checkStatus();
    void statusChecking();

    Signal<void(bool)> sigStatusUpdate;

    void onActivated();
    void changePollingPeriod(int value);
    void changeStateCheck();

private:
    RTSystemExt2Item* self;
    Connection locationChangedConnection;
    int connectionNo;

    Timer timer;
    ScopedConnection timeOutConnection;
    bool modifed_;
    cnoid::LazyCaller updateStateLater;

    void setStateCheckMethod(int value);
    void copyForStatusChecking();
    void restoreyForStatusChecking();
};

}
//////////
RTSPortExt2::RTSPortExt2(const string& name_, PortService_var port_, RTSCompExt2* parent)
    : rtsComp(parent), isConnected_(false)
{
    DDEBUG("RTSPortExt2::RTSPortExt2");
    isInPort = true;
    name = name_;
    port = port_;
    //
    if (port) {
        DDEBUG("RTSPortExt2::RTSPortExt2 Port ALIVE");
        RTC::PortProfile_var profile = port->get_port_profile();
        RTC::PortInterfaceProfileList interfaceList = profile->interfaces;
        for (CORBA::ULong index = 0; index < interfaceList.length(); ++index) {
            RTC::PortInterfaceProfile ifProfile = interfaceList[index];
            PortInterfaceExt2Ptr portIf(new PortInterfaceExt2());
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
        DDEBUG_V("RTSPortExt2::RTSPortExt2 Port isConnected_ %s,%d", name.c_str(), isConnected_);
    }
}


RTSPortExt2Ptr RTSPortExt2::copyForChecking() {
  DDEBUG("RTSPortExt2::copyForChecking");
  RTSPortExt2Ptr result = new RTSPortExt2(this->name, this->port, this->rtsComp);
  result->isServicePort = this->isServicePort;
  result->isInPort = this->isInPort;
  result->isConnected_ = this->isConnected_;
  result->orgPort_ = this;
  return result;
}


bool RTSPortExt2::checkConnected()
{
    if (!port || !isObjectAlive(port)) {
        return false;
    }
    ConnectorProfileList_var connectorProfiles = port->get_connector_profiles();
    return connectorProfiles->length() != 0;
}


bool RTSPortExt2::isConnectedWith(RTSPortExt2* target)
{
    if (!port || !target->port ||
       CORBA::is_nil(port) || port->_non_existent() ||
       CORBA::is_nil(target->port) || target->port->_non_existent()) {
        return false;
    }

    ConnectorProfileList_var connectorProfiles = port->get_connector_profiles();

    for (CORBA::ULong i = 0; i < connectorProfiles->length(); ++i) {
        ConnectorProfile& connectorProfile = connectorProfiles[i];
        PortServiceList& connectedPorts = connectorProfile.ports;
        for (CORBA::ULong j = 0; j < connectedPorts.length(); ++j) {
            PortService_ptr connectedPortRef = connectedPorts[j];
            if (connectedPortRef->_is_equivalent(target->port)) {
                return true;
            }
        }
    }

    return false;
}


bool RTSPortExt2::checkConnectablePort(RTSPortExt2* target)
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
        vector<string> dataTypes = RTSTypeUtilExt2::getAllowDataTypes(this, target);
        vector<string> ifTypes = RTSTypeUtilExt2::getAllowInterfaceTypes(this, target);
        vector<string> subTypes = RTSTypeUtilExt2::getAllowSubscriptionTypes(this, target);
        if (dataTypes.size() == 0 || ifTypes.size() == 0 || subTypes.size() == 0) {
            return false;
        }
    }

    return true;
}


vector<string> RTSPortExt2::getDataTypes()
{
    return getProperty("dataport.data_type");
}


vector<string> RTSPortExt2::getInterfaceTypes()
{
    return getProperty("dataport.interface_type");
}


vector<string> RTSPortExt2::getDataflowTypes()
{
    return getProperty("dataport.dataflow_type");
}


vector<string> RTSPortExt2::getSubscriptionTypes()
{
    return getProperty("dataport.subscription_type");
}


vector<string> RTSPortExt2::getProperty(const string& key)
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

RTSPortExt2* RTSCompExt2::nameToRTSPort(const string& name)
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
RTSConnectionExt2::RTSConnectionExt2(const string& id, const string& name,
        const string& sourceRtcName, const string& sourcePortName,
        const string& targetRtcName, const string& targetPortName)
    : id(id), name(name), sourceRtcName(sourceRtcName), sourcePortName(sourcePortName),
    targetRtcName(targetRtcName), targetPortName(targetPortName), setPos(false)
{
    isAlive_ = false;
}

RTSConnectionExt2Ptr RTSConnectionExt2::copyForChecking() {
  RTSConnectionExt2Ptr result = new RTSConnectionExt2(
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

bool RTSConnectionExt2::connect()
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
        NamedValueExt2Ptr param = propList[index];
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


bool RTSConnectionExt2::disconnect()
{
    isAlive_ = false;

    if (CORBA::is_nil(sourcePort->port) || sourcePort->port->_non_existent()) {
        return false;
    }

    return (sourcePort->port->disconnect(id.c_str()) == RTC_OK);
}


void RTSConnectionExt2::setPosition(const Vector2 pos[])
{
    for (int i = 0; i < 6; ++i) {
        position[i] = pos[i];
    }
    setPos = true;
    srcRTC->rts()->suggestFileUpdate();
}
//////////
RTSCompExt2::RTSCompExt2(const string& name, const std::string& fullPath, RTC::RTObject_ptr rtc, RTSystemExt2Item* rts, const QPointF& pos, const string& host, int port, bool isDefault)
    : rts_(rts), pos_(pos), name(name), fullPath(fullPath), hostAddress(host), portNo(port), isDefaultNS(isDefault)
{
    setRtc(rtc);
}

RTSCompExt2Ptr RTSCompExt2::copyForChecking() {
  DDEBUG("RTSCompExt2::copyForChecking");
  RTSCompExt2Ptr result= new RTSCompExt2(
                          this->name, this->fullPath, NULL, 
                          this->rts_, this->pos_,
                          this->hostAddress, this->portNo, this->isDefaultNS);
  result->rtc_ = this->rtc_;
  result->orgComp_ = this;

  if (!isObjectAlive(this->rtc_)) {
    DDEBUG("RTSCompExt2::copyForChecking RTC NOT ALIVE");
    result->participatingExeContList = 0;

    result->inPorts.clear();
    for (auto it = this->inPorts.begin(); it != this->inPorts.end(); it++) {
      (*it)->port = 0;
      RTSPortExt2* port = (*it)->copyForChecking();
      result->inPorts.push_back(port);
    }
    result->outPorts.clear();
    for (auto it = this->outPorts.begin(); it != this->outPorts.end(); it++) {
      (*it)->port = 0;
      RTSPortExt2* port = (*it)->copyForChecking();
      result->outPorts.push_back(port);
    }

  } else {
    DDEBUG("RTSCompExt2::copyForChecking RTC ALIVE");
    result->ownedExeContList_ = this->ownedExeContList_;
    result->activeIndex_ = this->activeIndex_;
    result->rtc_status_ = this->rtc_status_;
    result->isAlive_ = this->isAlive_;

    result->inPorts.clear();
    for (auto it = this->inPorts.begin(); it != this->inPorts.end(); it++) {
      result->inPorts.push_back((*it)->copyForChecking());
    }
    result->outPorts.clear();
    for (auto it = this->outPorts.begin(); it != this->outPorts.end(); it++) {
      result->outPorts.push_back((*it)->copyForChecking());
    }
  }
  return result;
}


void RTSCompExt2::setRtc(RTObject_ptr rtc)
{
    DDEBUG("RTSComp::setRtc");
    rtc_ = 0;

    rts_->suggestFileUpdate();

    setRTObject(rtc);

    if (!isObjectAlive(rtc)) {
        participatingExeContList = 0;
        for (auto it = inPorts.begin(); it != inPorts.end(); ++it) {
            RTSPortExt2* port = *it;
            port->port = 0;
        }
        for (auto it = outPorts.begin(); it != outPorts.end(); ++it) {
            RTSPortExt2* port = *it;
            port->port = 0;
        }
        DDEBUG("RTSComp::setRtc Failed");
        return;
    }

    //ComponentProfile_var cprofile = rtc_->get_component_profile();
    participatingExeContList = rtc_->get_participating_contexts();
    rtc_status_ = getRTCState();

    inPorts.clear();
    outPorts.clear();

    PortServiceList_var portlist = rtc_->get_ports();
    for (CORBA::ULong i = 0; i < portlist->length(); ++i) {
        PortProfile_var portprofile = portlist[i]->get_port_profile();
        coil::Properties pproperties = NVUtil::toProperties(portprofile->properties);
        string portType = pproperties["port.port_type"];
        RTSPortExt2Ptr rtsPort = new RTSPortExt2(string(portprofile->name), portlist[i], this);
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

    list<RTSConnectionExt2*> rtsConnectionList;
    rts_->impl->RTSCompToConnectionList(this, rtsConnectionList, 0);
    for (auto it = rtsConnectionList.begin(); it != rtsConnectionList.end(); ++it) {
      RTSConnectionExt2Ptr connection = (*it);
      rts_->impl->removeConnection(*it);
      RTSPortExt2* sourcePort = connection->srcRTC->nameToRTSPort(connection->sourcePortName);
      RTSPortExt2* targetPort = connection->targetRTC->nameToRTSPort(connection->targetPortName);
      if (sourcePort && targetPort) {
        connection->sourcePort = sourcePort;
        connection->targetPort = targetPort;
        rts_->impl->rtsConnections[RTSystemExt2Item::RTSPortPair(sourcePort, targetPort)] = connection;
      }
    }

    connectionCheck();
    DDEBUG("RTSComp::setRtc End");
}

bool RTSCompExt2::connectionCheck()
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


bool RTSCompExt2::connectionCheckSub(RTSPortExt2* rtsPort)
{
    bool updated = false;

    if (isObjectAlive(rtsPort->port) == false) return updated;

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
                    MessageView::WARNING, format(_("CORBA %1% (%2%), %3% in RTSComp::connectionCheckSub()"))
                    % ex._name() % ex._rep_id() % ex.NP_minorString());
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
            RTSCompExt2* targetRTC = rts_->impl->nameToRTSComp("/" + rtcPath);
            if(!targetRTC){
                continue;
            }

            //DDEBUG("targetRTC Found");
            RTSPortExt2* targetPort = targetRTC->nameToRTSPort(portName);
            if (targetPort) {
                auto itr = rts_->impl->rtsConnections.find(RTSystemExt2Item::RTSPortPair(rtsPort, targetPort));
                if (itr != rts_->impl->rtsConnections.end()) {
                    continue;
                }
                DDEBUG("RTSCompExt2::connectionCheckSub Add Con");
                RTSConnectionExt2Ptr rtsConnection = new RTSConnectionExt2(
                    string(connectorProfile.connector_id), string(connectorProfile.name),
                    name, rtsPort->name,
                    target[0], portName);
                coil::Properties properties = NVUtil::toProperties(connectorProfile.properties);
                vector<NamedValueExt2Ptr> propList;
                NamedValueExt2Ptr dataType(new NamedValueExt2("dataport.dataflow_type", properties["dataport.dataflow_type"]));
                propList.push_back(dataType);
                NamedValueExt2Ptr subscription(new NamedValueExt2("dataport.subscription_type", properties["dataport.subscription_type"]));
                rtsConnection->propList = propList;
                
                rtsConnection->srcRTC = this;
                rtsConnection->sourcePort = nameToRTSPort(rtsConnection->sourcePortName);
                rtsConnection->targetRTC = targetRTC;
                rtsConnection->targetPort = targetRTC->nameToRTSPort(rtsConnection->targetPortName);
                rtsConnection->isAlive_ = true;
                rts_->impl->rtsConnections[RTSystemExt2Item::RTSPortPair(rtsPort, targetPort)] = rtsConnection;
                
                rts_->suggestFileUpdate();
                
                updated = true;
            }
        }
    }

    return updated;
}


bool RTSCompExt2::connectionCheckForChecking()
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

    //DDEBUG_V("RTSCompExt2::connectionCheckForChecking End %d", updated);
    return updated;
}

bool RTSCompExt2::connectionCheckSubForChecking(RTSPortExt2* rtsPort)
{
    bool updated = false;

    if (isObjectAlive(rtsPort->port) == false) return updated;

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
                    MessageView::WARNING, format(_("CORBA %1% (%2%), %3% in RTSComp::connectionCheckSub()"))
                    % ex._name() % ex._rep_id() % ex.NP_minorString());
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
            RTSCompExt2* targetRTC = rts_->impl->nameToRTSCompForChecking("/" + rtcPath);
            if(!targetRTC){
                continue;
            }

            //DDEBUG("targetRTC Found");
            RTSPortExt2* targetPort = targetRTC->orgComp_->nameToRTSPort(portName);
            if (targetPort) {
                auto itr = rts_->impl->rtsConnectionsCheck.find(RTSystemExt2Item::RTSPortPair(rtsPort->orgPort_, targetPort));
                if (itr != rts_->impl->rtsConnectionsCheck.end()) {
                    continue;
                }
                DDEBUG("RTSCompExt2::connectionCheckSub Add Con");
                RTSConnectionExt2Ptr rtsConnection = new RTSConnectionExt2(
                    string(connectorProfile.connector_id), string(connectorProfile.name),
                    name, rtsPort->name,
                    target[0], portName);
                coil::Properties properties = NVUtil::toProperties(connectorProfile.properties);
                vector<NamedValueExt2Ptr> propList;
                NamedValueExt2Ptr dataType(new NamedValueExt2("dataport.dataflow_type", properties["dataport.dataflow_type"]));
                propList.push_back(dataType);
                NamedValueExt2Ptr subscription(new NamedValueExt2("dataport.subscription_type", properties["dataport.subscription_type"]));
                rtsConnection->propList = propList;
                
                rtsConnection->srcRTC = this->orgComp_;
                rtsConnection->sourcePort = this->orgComp_->nameToRTSPort(rtsConnection->sourcePortName);
                rtsConnection->targetRTC = targetRTC->orgComp_;
                rtsConnection->targetPort = targetRTC->orgComp_->nameToRTSPort(rtsConnection->targetPortName);
                rtsConnection->isAlive_ = true;
                rts_->impl->rtsConnectionsAdded[RTSystemExt2Item::RTSPortPair(rtsPort->orgPort_, targetPort)] = rtsConnection;
                
                rts_->suggestFileUpdate();
                
                updated = true;
            }
        }
    }

    DDEBUG_V("RTSCompExt2::connectionCheckSubForChecking End %d", updated);
    return updated;
}

bool RTSCompExt2::getComponentPath(RTC::PortService_ptr source, std::string& out_path)
{
    PortProfile_var portprofile = source->get_port_profile();
    if(!CORBA::is_nil(portprofile->owner)){
        ComponentProfile_var cprofile;
        try {
            cprofile = portprofile->owner->get_component_profile();
        }
        catch (CORBA::SystemException& ex) {
            MessageView::instance()->putln(
                MessageView::WARNING, format(_("CORBA %1% (%2%), %3% in RTSComp::getComponentPath()"))
                % ex._name() % ex._rep_id() % ex.NP_minorString());
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

void RTSCompExt2::moveToRelative(const QPointF& p)
{
    QPointF newPos = pos_ + p;
    if (newPos != pos_) {
        pos_ = newPos;
        rts_->suggestFileUpdate();
    }
}
//////////
void RTSystemExt2Item::initializeClass(ExtensionManager* ext)
{
    DDEBUG("RTSystemItem::initializeClass");
    ItemManager& im = ext->itemManager();
    im.registerClass<RTSystemExt2Item>(N_("RTSystemItem"));
    im.addCreationPanel<RTSystemExt2Item>();
    im.addLoaderAndSaver<RTSystemExt2Item>(
        _("RT-System"), "RTS-PROFILE-XML", "xml",
        [](RTSystemExt2Item* item, const std::string& filename, std::ostream&, Item*) {
        return item->loadRtsProfile(filename);
    },
        [](RTSystemExt2Item* item, const std::string& filename, std::ostream&, Item*) {
        return item->saveRtsProfile(filename);
    });
}


RTSystemExt2Item::RTSystemExt2Item()
{
    DDEBUG("RTSystemItem::RTSystemItem");
    impl = new RTSystemExt2ItemImpl(this);
}


RTSystemExt2ItemImpl::RTSystemExt2ItemImpl(RTSystemExt2Item* self)
    : self(self), pollingCycle(1000), isStatusChecking(false)
{
    initialize();
    autoConnection = true;
}


RTSystemExt2Item::RTSystemExt2Item(const RTSystemExt2Item& org)
    : Item(org)
{
    impl = new RTSystemExt2ItemImpl(this, *org.impl);
}


RTSystemExt2ItemImpl::RTSystemExt2ItemImpl(RTSystemExt2Item* self, const RTSystemExt2ItemImpl& org)
    : self(self), pollingCycle(1000)
{
    initialize();
    autoConnection = org.autoConnection;
}

void RTSystemExt2ItemImpl::initialize()
{
    DDEBUG_V("RTSystemItemImpl::initialize cycle:%d", pollingCycle);
    connectionNo = 0;

    Mapping* config = AppConfig::archive()->openMapping("OpenRTM");
    vendorName = config->get("defaultVendor", "AIST");
    version = config->get("defaultVersion", "1.0.0");
    stateCheck.setSymbol(POLLING_CHECK, "Polling");
    stateCheck.setSymbol(MANUAL_CHECK, "Manual");
    checkAtLoading = true;
#if defined(OPENRTM_VERSION12)
    stateCheck.setSymbol(OBSERVER_CHECK, "Observer");
    heartBeatPeriod = config->get("heartBeatPeriod", 500);
#endif

    timer.setInterval(pollingCycle);
    timer.setSingleShot(false);
    timeOutConnection.reset(
        timer.sigTimeout().connect(
            std::bind(&RTSystemExt2ItemImpl::checkStatus, this)));
}

RTSystemExt2Item::~RTSystemExt2Item()
{
    delete impl;
}

RTSystemExt2ItemImpl::~RTSystemExt2ItemImpl()
{
    locationChangedConnection.disconnect();
}


Item* RTSystemExt2Item::doDuplicate() const
{
    return new RTSystemExt2Item(*this);
}

void RTSystemExt2ItemImpl::onLocationChanged(string host, int port)
{
    NameServerManager::instance()->getNCHelper()->setLocation(host, port);
}

void RTSystemExt2Item::onActivated()
{
    impl->onActivated();
}

void RTSystemExt2ItemImpl::onActivated()
{
    DDEBUG("RTSystemExtItemImpl::onActivated");
    if( sigStatusUpdate.empty() == false && stateCheck.selectedIndex()==POLLING_CHECK ) {
        timer.start();
    }
}

RTSCompExt2* RTSystemExt2Item::nameToRTSComp(const string& name)
{
    return impl->nameToRTSComp(name);
}

RTSCompExt2* RTSystemExt2ItemImpl::nameToRTSComp(const string& name)
{
    //DDEBUG_V("RTSystemItemImpl::nameToRTSComp:%s", name.c_str());
    map<string, RTSCompExt2Ptr>::iterator it = rtsComps.find(name);
    if (it == rtsComps.end())
        return 0;
    else
        return it->second.get();
}


RTSCompExt2* RTSystemExt2ItemImpl::nameToRTSCompForChecking(const string& name)
{
    map<string, RTSCompExt2Ptr>::iterator it = rtsCompsCheck.find(name);
    if (it == rtsCompsCheck.end())
        return 0;
    else
        return it->second.get();
}


RTSCompExt2* RTSystemExt2Item::addRTSComp(const string& name, const QPointF& pos)
{
    return impl->addRTSComp(name, pos);
}


RTSCompExt2* RTSystemExt2ItemImpl::addRTSComp(const string& name, const QPointF& pos)
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
        RTSCompExt2Ptr rtsComp = new RTSCompExt2(name, fullPath, rtc, self, pos, ncHelper->host().c_str(), ncHelper->port(), false);
        rtsComps[fullPath] = rtsComp;

        self->suggestFileUpdate();

        return rtsComp;
    }

    return nullptr;
}


RTSCompExt2* RTSystemExt2Item::addRTSComp(const  NamingContextHelper::ObjectInfo& info, const QPointF& pos)
{
    return impl->addRTSComp(info, pos);
}


RTSCompExt2* RTSystemExt2ItemImpl::addRTSComp(const NamingContextHelper::ObjectInfo& info, const QPointF& pos)
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

        RTSCompExt2Ptr rtsComp = new RTSCompExt2(info.id_, fullPath, rtc, self, pos, info.hostAddress_, info.portNo_, info.isRegisteredInRtmDefaultNameServer_);
        rtsComps[fullPath] = rtsComp;

        self->suggestFileUpdate();

        return rtsComp.get();
    }
    timeOutConnection.unblock();

    return nullptr;
}

void RTSystemExt2Item::deleteRTSComp(const string& name)
{
    impl->deleteRTSComp(name);
}


void RTSystemExt2ItemImpl::deleteRTSComp(const string& name)
{
    timeOutConnection.block();

    if (rtsComps.erase(name) > 0) {
        self->suggestFileUpdate();
    }

    timeOutConnection.unblock();
}


bool RTSystemExt2ItemImpl::compIsAliveForChecking(RTSCompExt2* rtsComp)
{
    DDEBUG("RTSystemItemImpl::compIsAliveForChecking");
    if (rtsComp->isAlive_ && rtsComp->rtc_ && rtsComp->rtc_ != nullptr) {
        if (isObjectAlive(rtsComp->rtc_)) {
            return true;
        } else {
            rtsComp->setRtc(nullptr);
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
            return false;
        } else {
            return true;
        }
    }
}

string RTSystemExt2ItemImpl::getConnectionNumber()
{
    stringstream ss;
    ss << connectionNo;
    connectionNo++;
    return ss.str();
}


RTSConnectionExt2* RTSystemExt2Item::addRTSConnection
(const std::string& id, const std::string& name, RTSPortExt2* sourcePort, RTSPortExt2* targetPort, const std::vector<NamedValueExt2Ptr>& propList, const Vector2 pos[])
{
    bool setPos = true;
    if (!pos) setPos = false;
    return impl->addRTSConnection(id, name, sourcePort, targetPort, propList, setPos, pos);
}


RTSConnectionExt2* RTSystemExt2ItemImpl::addRTSConnection
(const string& id, const string& name,
 RTSPortExt2* sourcePort, RTSPortExt2* targetPort, const std::vector<NamedValueExt2Ptr>& propList,
 const bool setPos, const Vector2 pos[])
{
    DDEBUG("RTSystemItemImpl::addRTSConnection");
    timeOutConnection.block();

    bool updated = false;

    RTSConnectionExt2* rtsConnection_;
    auto it = rtsConnections.find(RTSystemExt2Item::RTSPortPair(sourcePort, targetPort));

    if (it != rtsConnections.end()) {
        rtsConnection_ = it->second;;

    } else {
        RTSConnectionExt2Ptr rtsConnection =
            new RTSConnectionExt2(
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

        rtsConnections[RTSystemExt2Item::RTSPortPair(sourcePort, targetPort)] = rtsConnection;
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


RTSConnectionExt2* RTSystemExt2ItemImpl::addRTSConnectionName
(const string& id, const string& name,
 const string& sourceCompName, const string& sourcePortName,
 const string& targetCompName, const string& targetPortName,
 const string& dataflow, const string& subscription,
 const bool setPos, const Vector2 pos[])
{
    string sourceId = "/" + sourceCompName + ".rtc";
    RTSPortExt2* sourcePort = 0;
    RTSCompExt2* sourceRtc = nameToRTSComp(sourceId);
    if (sourceRtc) {
        sourcePort = sourceRtc->nameToRTSPort(sourcePortName);
    }

    string targetId = "/" + targetCompName + ".rtc";
    RTSPortExt2* targetPort = 0;
    RTSCompExt2* targetRtc = nameToRTSComp(targetId);
    if (targetRtc) {
        targetPort = targetRtc->nameToRTSPort(targetPortName);
    }
    if (sourcePort && targetPort) {
        vector<NamedValueExt2Ptr> propList;
        NamedValueExt2Ptr paramDataFlow(new NamedValueExt2("dataport.dataflow_type", dataflow));
        propList.push_back(paramDataFlow);
        NamedValueExt2Ptr paramSubscription(new NamedValueExt2("dataport.subscription_type", subscription));
        propList.push_back(paramSubscription);
        NamedValueExt2Ptr sinterfaceProp(new NamedValueExt2("dataport.interface_type", "corba_cdr"));
        propList.push_back(sinterfaceProp);

        return addRTSConnection(id, name, sourcePort, targetPort, propList, setPos, pos);
    }

    return nullptr;
}


void RTSystemExt2Item::RTSCompToConnectionList
(const RTSCompExt2* rtsComp, list<RTSConnectionExt2*>& rtsConnectionList, int mode)
{
    impl->RTSCompToConnectionList(rtsComp, rtsConnectionList, mode);
}


void RTSystemExt2ItemImpl::RTSCompToConnectionList
(const RTSCompExt2* rtsComp, list<RTSConnectionExt2*>& rtsConnectionList, int mode)
{
    for (RTSystemExt2Item::RTSConnectionMap::iterator it = rtsConnections.begin();
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


map<string, RTSCompExt2Ptr>& RTSystemExt2Item::rtsComps()
{
    return impl->rtsComps;
}


RTSystemExt2Item::RTSConnectionMap& RTSystemExt2Item::rtsConnections()
{
    return impl->rtsConnections;
}


void RTSystemExt2Item::disconnectAndRemoveConnection(RTSConnectionExt2* connection)
{
    connection->disconnect();
    impl->removeConnection(connection);
}


void RTSystemExt2ItemImpl::removeConnection(RTSConnectionExt2* connection)
{
    RTSystemExt2Item::RTSPortPair pair(connection->sourcePort, connection->targetPort);
    if (rtsConnections.erase(pair) > 0) {
        self->suggestFileUpdate();
    }
}


void RTSystemExt2Item::doPutProperties(PutPropertyFunction& putProperty)
{
    impl->doPutProperties(putProperty);
}


void RTSystemExt2ItemImpl::doPutProperties(PutPropertyFunction& putProperty)
{
    DDEBUG("RTSystemItemImpl::doPutProperties");
    putProperty(_("Auto Connection"), autoConnection, changeProperty(autoConnection));
    putProperty(_("Vendor Name"), vendorName, changeProperty(vendorName));
    putProperty(_("Version"), version, changeProperty(version));
    putProperty(_("State Check"), stateCheck,
                [&](int value) { setStateCheckMethod(value); return true; });
    putProperty(_("Polling Cycle"), pollingCycle,
                [&](int value) { changePollingPeriod(value); return true; });
    putProperty(_("CheckAtLoading"), checkAtLoading, changeProperty(checkAtLoading));

#if defined(OPENRTM_VERSION12)
    putProperty(_("HeartBeat Period"), heartBeatPeriod, changeProperty(heartBeatPeriod));
#endif
}

void RTSystemExt2ItemImpl::changePollingPeriod(int value)
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

void RTSystemExt2ItemImpl::setStateCheckMethod(int value)
{
    DDEBUG_V("RTSystemItemImpl::setStateCheckMethod=%d", value);
    stateCheck.selectIndex(value);
    changeStateCheck();
}

void RTSystemExt2ItemImpl::setStateCheckMethodByString(const string& value)
{
    DDEBUG_V("RTSystemItemImpl::setStateCheckMethodByString=%s", value.c_str());
    stateCheck.select(value);
    DDEBUG_V("RTSystemItemImpl::setStateCheckMethodByString=%d", stateCheck.selectedIndex());
    changeStateCheck();
}

void RTSystemExt2ItemImpl::changeStateCheck()
{
    int state = stateCheck.selectedIndex();
    DDEBUG_V("RTSystemItemImpl::changeStateCheck=%d", state);
    switch (state) {
        case MANUAL_CHECK:
            timer.stop();
            break;
#if defined(OPENRTM_VERSION12)
        case OBSERVER_CHECK:
            break;
#endif
        default:
            timer.start();
            break;
    }
}

bool RTSystemExt2Item::loadRtsProfile(const string& filename)
{
    DDEBUG_V("RTSystemItem::loadRtsProfile=%s", filename.c_str());
    ProfileHandlerExt2::getRtsProfileInfo(filename, impl->vendorName, impl->version);
    if (ProfileHandlerExt2::restoreRtsProfile(filename, this)) {
        notifyUpdate();
        return true;
    }
    return false;
}


bool RTSystemExt2Item::saveRtsProfile(const string& filename)
{
    return impl->saveRtsProfile(filename);
}


bool RTSystemExt2ItemImpl::saveRtsProfile(const string& filename)
{
    if (vendorName.empty()) {
        vendorName = "Choreonoid";
    }
    if (version.empty()) {
        version = "1.0.0";
    }
    string systemId = "RTSystem:" + vendorName + ":" + self->name() + ":" + version;
    ProfileHandlerExt2::saveRtsProfile(filename, systemId, rtsComps, rtsConnections, MessageView::mainInstance()->cout());

    return true;
}

void RTSystemExt2Item::setVendorName(const std::string& name)
{
    impl->vendorName = name;
}


void RTSystemExt2Item::setVersion(const std::string& version)
{
    impl->version = version;
}

int RTSystemExt2Item::stateCheck() const
{
    return impl->stateCheck.selectedIndex();
}

void RTSystemExt2Item::checkStatus()
{
    impl->checkStatus();
}

void RTSystemExt2ItemImpl::checkStatus()
{
    DDEBUG("RTSystemExt2ItemImpl::checkStatus");
    if( sigStatusUpdate.empty() ) {
        timer.stop();
    }
    //statusChecking();
    std::thread checkThread = std::thread([&](){ statusChecking(); });
    checkThread.join();
}

void RTSystemExt2ItemImpl::statusChecking()
{
    if (isStatusChecking) return;
    isStatusChecking = true;

    modifed_ = false;

    copyForStatusChecking();
    //////////
    DDEBUG("RTSystemExt2ItemImpl::statusChecking start");
    for (auto it = rtsCompsCheck.begin(); it != rtsCompsCheck.end(); it++) {
        RTSCompExt2* targetComp = it->second;
        if (compIsAliveForChecking(it->second)) {
            if (!it->second->isAlive_) {
              modifed_ = true;
            }
            it->second->isAlive_ = true;
            RTC_STATUS status = it->second->getRTCState();
            DDEBUG_V("RTC State : %d, %d", it->second->rtc_status_, status);
            if (status != it->second->rtc_status_) {
              modifed_ = true;
              it->second->rtc_status_ = status;
            }
        } else {
            if (it->second->isAlive_) {
                modifed_ = true;
            }
            it->second->isAlive_ = false;
        }
    }
    ///
    for (auto it = rtsConnectionsCheck.begin(); it != rtsConnectionsCheck.end(); it++) {
        const RTSystemExt2Item::RTSPortPair& ports = it->first;
        if (ports(0)->isConnectedWith(ports(1))) {
            if (!it->second->isAlive_) {
                it->second->isAlive_ = true;
                modifed_ = true;
            }
        } else {
            if (it->second->isAlive_) {
                it->second->isAlive_ = false;
                modifed_ = true;
            }
        }
    }

    for (auto it = rtsCompsCheck.begin(); it != rtsCompsCheck.end(); it++) {
        if (it->second->connectionCheckForChecking()) {
            modifed_ = true;
        }
    }
    //////////
    DDEBUG("RTSystemExt2ItemImpl::statusChecking finish");
    callLater([&](){ restoreyForStatusChecking(); });
}

void RTSystemExt2ItemImpl::copyForStatusChecking() {
    DDEBUG("RTSystemExt2ItemImpl::copyForStatusChecking");
    std::mutex rtcStateMutex;

    std::lock_guard<std::mutex> guard(rtcStateMutex);
    rtsCompsCheck.clear();
    for (auto it = rtsComps.begin(); it != rtsComps.end(); it++) {
      rtsCompsCheck[it->first] = it->second->copyForChecking();
    }
    rtsConnectionsCheck.clear();
    for (auto it = rtsConnections.begin(); it != rtsConnections.end(); it++) {
      rtsConnectionsCheck[it->first] = it->second->copyForChecking();
    }
    rtsConnectionsAdded.clear();
}

void RTSystemExt2ItemImpl::restoreyForStatusChecking() {
    std::mutex rtcStateMutex;

    std::lock_guard<std::mutex> guard(rtcStateMutex);
    for (auto it = rtsCompsCheck.begin(); it != rtsCompsCheck.end(); it++) {
      RTSCompExt2Ptr targetComp = rtsComps[it->first];
      if (targetComp) {
        RTSCompExt2Ptr sourceComp = rtsCompsCheck[it->first];
        targetComp->rtc_status_ = sourceComp->rtc_status_;
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
      RTSConnectionExt2Ptr targetConn = rtsConnections[it->first];
      if (targetConn) {
        RTSConnectionExt2Ptr sourceConn = rtsConnectionsCheck[it->first];
        targetConn->isAlive_ = sourceConn->isAlive_;
      }
    }
    for (auto it = rtsConnectionsAdded.begin(); it != rtsConnectionsAdded.end(); it++) {
      rtsConnections[it->first] = it->second;
    }

    DDEBUG_V("RTSystemExt2ItemImpl::checkStatus End : %d", modifed_);
    sigStatusUpdate(modifed_);
    isStatusChecking = false;
}

bool RTSystemExt2Item::isCheckAtLoading()
{
    return impl->checkAtLoading;
}
///////////
bool RTSystemExt2Item::store(Archive& archive)
{
    if (overwrite()) {
        archive.writeRelocatablePath("filename", filePath());
        archive.write("format", fileFormat());

        archive.write("autoConnection", impl->autoConnection);
        archive.write("pollingCycle", impl->pollingCycle);
        archive.write("stateCheck", impl->stateCheck.selectedSymbol());
        archive.write("checkAtLoading", impl->checkAtLoading);

#if defined(OPENRTM_VERSION12)
        archive.write("HeartBeatPeriod", impl->heartBeatPeriod);
#endif
        return true;
    }

    return true;
}


bool RTSystemExt2Item::restore(const Archive& archive)
{
    DDEBUG("RTSystemExt2Item::restore");

    if (archive.read("autoConnection", impl->autoConnection) == false) {
        archive.read("AutoConnection", impl->autoConnection);
    }

    int pollingCycle = 1000;
    if( archive.read("pollingCycle", pollingCycle)==false) {
        archive.read("PollingCycle", pollingCycle);
    }
    if( archive.read("checkAtLoading", impl->checkAtLoading)==false) {
        archive.read("CheckAtLoading", impl->checkAtLoading);
    }

    impl->changePollingPeriod(pollingCycle);

#if defined(OPENRTM_VERSION12)
    if(archive.read("HeartBeatPeriod", impl->heartBeatPeriod) == false) {
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

    string stateCheck;
    if (archive.read("stateCheck", stateCheck) == false) {
        archive.read("StateCheck", stateCheck);
    }
    if(stateCheck.empty()==false) {
        DDEBUG_V("StateCheck:%s", stateCheck.c_str());
        impl->setStateCheckMethodByString(stateCheck);
        archive.addPostProcess([&]() { impl->changeStateCheck(); });
    }

    return true;
}


void RTSystemExt2ItemImpl::restoreRTSystem(const Archive& archive)
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


void RTSystemExt2ItemImpl::restoreRTSComp(const string& name, const Vector2& pos,
        const vector<pair<string, bool>>& inPorts, const vector<pair<string, bool>>& outPorts)
{
    DDEBUG("RTSystemItemImpl::restoreRTSComp");

    RTSCompExt2* comp = addRTSComp(name, QPointF(pos(0), pos(1)));
    if (comp == 0) return;
    if (!comp->rtc_) {
        comp->inPorts.clear();
        comp->outPorts.clear();
        for (int i = 0; i < inPorts.size(); i++) {
            RTSPortExt2Ptr rtsPort = new RTSPortExt2(inPorts[i].first, 0, comp);
            rtsPort->isInPort = true;
            rtsPort->isServicePort = inPorts[i].second;
            comp->inPorts.push_back(rtsPort);
        }
        for (int i = 0; i < outPorts.size(); i++) {
            RTSPortExt2Ptr rtsPort = new RTSPortExt2(outPorts[i].first, 0, comp);
            rtsPort->isInPort = false;
            rtsPort->isServicePort = outPorts[i].second;
            comp->outPorts.push_back(rtsPort);
        }
    }
    DDEBUG("RTSystemItemImpl::restoreRTSComp End");
}

SignalProxy<void(bool)> RTSystemExt2Item::sigStatusUpdate()
{
    return impl->sigStatusUpdate;
}
