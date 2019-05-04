#include "RTSystemItem.h"
#include "RTSTypeUtil.h"
#include <cnoid/MessageView>
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/EigenArchive>
#include <cnoid/AppConfig>
#include <rtm/CORBA_SeqUtil.h>
#include <fmt/format.h>
#include "LoggerUtil.h"
#include "gettext.h"

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

class RTSystemItemImpl
{
public:
    RTSystemItem* self;
    Connection locationChangedConnection;
    map<string, RTSCompPtr> rtsComps;
    RTSystemItem::RTSConnectionMap rtsConnections;
    int connectionNo;
    bool autoConnection;

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

    RTSystemItemImpl(RTSystemItem* self);
    RTSystemItemImpl(RTSystemItem* self, const RTSystemItemImpl& org);
    ~RTSystemItemImpl();

    void initialize();
    void onLocationChanged(std::string host, int port);
    RTSComp* addRTSComp(const string& name, const QPointF& pos);
    RTSComp* addRTSComp(const NamingContextHelper::ObjectInfo& info, const QPointF& pos);
    void deleteRTSComp(const string& name);
    RTSComp* nameToRTSComp(const string& name);
    bool compIsAlive(RTSComp* rtsComp);
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
    bool connectionCheck();
    void RTSCompToConnectionList(const RTSComp* rtsComp, list<RTSConnection*>& rtsConnectionList, int mode);
    void removeConnection(RTSConnection* connection);
    void doPutProperties(PutPropertyFunction& putProperty);
    bool saveRtsProfile(const string& filename);
    void restoreRTSystem(const Archive& archive);
    void restoreRTSComp(const string& name, const Vector2& pos,
            const vector<pair<string, bool>>& inPorts, const vector<pair<string, bool>>& outPorts);
    string getConnectionNumber();
    void setStateCheckMethodByString(const string& value);
    bool checkStatus();

    Signal<void(int)> sigTimerPeriodChanged;
    Signal<void(bool)> sigTimerChanged;

    void changeStateCheck();
    void changePollingPeriod(int value);

private:
    void setStateCheckMethod(int value);
};

}


//////////
RTSPort::RTSPort(const string& name_, PortService_var port_, RTSComp* parent)
    : rtsComp(parent)
{
    isInPort = true;
    name = name_;
    port = port_;
    //
    if (port) {
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
    }
}


bool RTSPort::isConnected()
{
    if (!port || !isObjectAlive(port)) {
        return false;
    }
    ConnectorProfileList_var connectorProfiles = port->get_connector_profiles();
    return connectorProfiles->length() != 0;
}


bool RTSPort::isConnectedWith(RTSPort* target)
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


RTSConnection::RTSConnection(const string& id, const string& name,
        const string& sourceRtcName, const string& sourcePortName,
        const string& targetRtcName, const string& targetPortName)
    : id(id), name(name), sourceRtcName(sourceRtcName), sourcePortName(sourcePortName),
    targetRtcName(targetRtcName), targetPortName(targetPortName), setPos(false)
{
    isAlive_ = false;
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
    DDEBUG("suggestFileUpdate RTSConnection::setPosition");
}


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
        DDEBUG("RTSComp::setRtc Failed");
        return;
    }

    ComponentProfile_var cprofile = rtc_->get_component_profile();
    participatingExeContList = rtc_->get_participating_contexts();

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
    auto rtsImpl = static_cast<RTSystemItem*>(rtsItem_)->impl;
    rtsImpl->RTSCompToConnectionList(this, rtsConnectionList, 0);
    for (auto it = rtsConnectionList.begin(); it != rtsConnectionList.end(); ++it) {
        RTSConnectionPtr connection = (*it);
        rtsImpl->removeConnection(*it);
        RTSPort* sourcePort = connection->srcRTC->nameToRTSPort(connection->sourcePortName);
        RTSPort* targetPort = connection->targetRTC->nameToRTSPort(connection->targetPortName);
        if (sourcePort && targetPort) {
            connection->sourcePort = sourcePort;
            connection->targetPort = targetPort;
            rtsImpl->rtsConnections[RTSystemItem::RTSPortPair(sourcePort, targetPort)] = connection;
        }
    }
    rtsItem_->setConsistentWithFile(isUpdated);

    connectionCheck();
    DDEBUG("RTSComp::setRtc End");
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


bool RTSComp::connectionCheck()
{
    //DDEBUG("RTSComp::connectionCheck");

    bool updated = false;

    for (auto it = inPorts.begin(); it != inPorts.end(); ++it) {
        if (connectionCheckSub(*it)) {
            updated = true;
        }
    }
    for (auto it = outPorts.begin(); it != outPorts.end(); it++) {
        if (connectionCheckSub(*it)) {
            updated = true;
        }
    }

    return updated;
}


bool RTSComp::connectionCheckSub(RTSPort* rtsPort)
{
    bool updated = false;

    if (!isObjectAlive(rtsPort->port)) return updated;

    auto rtsImpl = static_cast<RTSystemItem*>(rtsItem_)->impl;
    
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
                auto itr = rtsImpl->rtsConnections.find(RTSystemItem::RTSPortPair(rtsPort, targetPort));
                if (itr != rtsImpl->rtsConnections.end()) {
                    continue;
                }
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
                rtsImpl->rtsConnections[RTSystemItem::RTSPortPair(rtsPort, targetPort)] = rtsConnection;
                
                rtsItem_->suggestFileUpdate();
                DDEBUG("suggestFileUpdate RTSComp::connectionCheckSub");
                
                updated = true;
            }
        }
    }

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


void RTSComp::setPos(const QPointF& p)
{
    if (p != pos_) {
        pos_ = p;
        rtsItem_->suggestFileUpdate();
        DDEBUG("suggestFileUpdate RTSComp::setPos");
    }
}


void RTSystemItem::initializeClass(ExtensionManager* ext)
{
    DDEBUG("RTSystemItem::initializeClass");
    ItemManager& im = ext->itemManager();
    im.registerClass<RTSystemItem>(N_("RTSystemItem"));
    im.addCreationPanel<RTSystemItem>();
    im.addLoaderAndSaver<RTSystemItem>(
        _("RT-System"), "RTS-PROFILE-XML", "xml",
        [](RTSystemItem* item, const std::string& filename, std::ostream&, Item*){
            return item->loadRtsProfile(filename);
        },
        [](RTSystemItem* item, const std::string& filename, std::ostream&, Item*){
            return item->saveRtsProfile(filename);
        });
}


RTSystemItem::RTSystemItem()
{
    DDEBUG("RTSystemItem::RTSystemItem");
    impl = new RTSystemItemImpl(this);
}


RTSystemItemImpl::RTSystemItemImpl(RTSystemItem* self)
    : self(self)
{
    initialize();
    autoConnection = true;
}


RTSystemItem::RTSystemItem(const RTSystemItem& org)
    : Item(org)
{
    impl = new RTSystemItemImpl(this, *org.impl);
}


RTSystemItemImpl::RTSystemItemImpl(RTSystemItem* self, const RTSystemItemImpl& org)
    : self(self), pollingCycle(1000)
{
    initialize();
    autoConnection = org.autoConnection;
}

void RTSystemItemImpl::initialize()
{
    DDEBUG("RTSystemItemImpl::initialize");
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
}

RTSystemItem::~RTSystemItem()
{
    delete impl;
}

RTSystemItemImpl::~RTSystemItemImpl()
{
    locationChangedConnection.disconnect();
}


Item* RTSystemItem::doDuplicate() const
{
    return new RTSystemItem(*this);
}

void RTSystemItemImpl::onLocationChanged(string host, int port)
{
    NameServerManager::instance()->getNCHelper()->setLocation(host, port);
}

RTSComp* RTSystemItem::nameToRTSComp(const string& name)
{
    return impl->nameToRTSComp(name);
}

RTSComp* RTSystemItemImpl::nameToRTSComp(const string& name)
{
    //DDEBUG_V("RTSystemItemImpl::nameToRTSComp:%s", name.c_str());
    map<string, RTSCompPtr>::iterator it = rtsComps.find(name);
    if (it == rtsComps.end())
        return 0;
    else
        return it->second.get();
}


RTSComp* RTSystemItem::addRTSComp(const string& name, const QPointF& pos)
{
    return impl->addRTSComp(name, pos);
}


RTSComp* RTSystemItemImpl::addRTSComp(const string& name, const QPointF& pos)
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
        DDEBUG("suggestFileUpdate RTSystemItemImpl::addRTSComp");

        return rtsComp;
    }

    return nullptr;
}


RTSComp* RTSystemItem::addRTSComp(const  NamingContextHelper::ObjectInfo& info, const QPointF& pos)
{
    return impl->addRTSComp(info, pos);
}


RTSComp* RTSystemItemImpl::addRTSComp(const NamingContextHelper::ObjectInfo& info, const QPointF& pos)
{
    DDEBUG("RTSystemItemImpl::addRTSComp");

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
        DDEBUG("suggestFileUpdate RTSystemItemImpl::addRTSComp");

        return rtsComp.get();
    }

    return nullptr;
}

void RTSystemItem::deleteRTSComp(const string& name)
{
    impl->deleteRTSComp(name);
}


void RTSystemItemImpl::deleteRTSComp(const string& name)
{
    if (rtsComps.erase(name) > 0) {
        self->suggestFileUpdate();
        DDEBUG("suggestFileUpdate RTSystemItemImpl::deleteRTSComp");
    }
}


bool RTSystemItem::compIsAlive(RTSComp* rtsComp)
{
    return impl->compIsAlive(rtsComp);
}


bool RTSystemItemImpl::compIsAlive(RTSComp* rtsComp)
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


string RTSystemItemImpl::getConnectionNumber()
{
    stringstream ss;
    ss << connectionNo;
    connectionNo++;
    return ss.str();
}


RTSConnection* RTSystemItem::addRTSConnection
(const std::string& id, const std::string& name, RTSPort* sourcePort, RTSPort* targetPort, const std::vector<NamedValuePtr>& propList, const Vector2 pos[])
{
    bool setPos = true;
    if (!pos) setPos = false;
    return impl->addRTSConnection(id, name, sourcePort, targetPort, propList, setPos, pos);
}


RTSConnection* RTSystemItemImpl::addRTSConnection
(const string& id, const string& name,
 RTSPort* sourcePort, RTSPort* targetPort, const std::vector<NamedValuePtr>& propList,
 const bool setPos, const Vector2 pos[])
{
    DDEBUG("RTSystemItemImpl::addRTSConnection");

    bool updated = false;

    RTSConnection* rtsConnection_;
    auto it = rtsConnections.find(RTSystemItem::RTSPortPair(sourcePort, targetPort));

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

        rtsConnections[RTSystemItem::RTSPortPair(sourcePort, targetPort)] = rtsConnection;
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
        DDEBUG("suggestFileUpdate RTSystemItemImpl::addRTSConnection");
    }

    return rtsConnection_;
}


RTSConnection* RTSystemItemImpl::addRTSConnectionName
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


bool RTSystemItem::connectionCheck()
{
    return impl->connectionCheck();
}


bool RTSystemItemImpl::connectionCheck()
{
    //DDEBUG("RTSystemItemImpl::connectionCheck");
    bool updated = false;

    for (auto it = rtsConnections.begin(); it != rtsConnections.end(); it++) {
        const RTSystemItem::RTSPortPair& ports = it->first;
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

    //DDEBUG("RTSystemItemImpl::connectionCheck End");
    return updated;
}


void RTSystemItem::RTSCompToConnectionList
(const RTSComp* rtsComp, list<RTSConnection*>& rtsConnectionList, int mode)
{
    impl->RTSCompToConnectionList(rtsComp, rtsConnectionList, mode);
}


void RTSystemItemImpl::RTSCompToConnectionList
(const RTSComp* rtsComp, list<RTSConnection*>& rtsConnectionList, int mode)
{
    for (RTSystemItem::RTSConnectionMap::iterator it = rtsConnections.begin();
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


map<string, RTSCompPtr>& RTSystemItem::rtsComps()
{
    return impl->rtsComps;
}


RTSystemItem::RTSConnectionMap& RTSystemItem::rtsConnections()
{
    return impl->rtsConnections;
}


void RTSystemItem::disconnectAndRemoveConnection(RTSConnection* connection)
{
    connection->disconnect();
    impl->removeConnection(connection);
}


void RTSystemItemImpl::removeConnection(RTSConnection* connection)
{
    RTSystemItem::RTSPortPair pair(connection->sourcePort, connection->targetPort);
    if (rtsConnections.erase(pair) > 0) {
        self->suggestFileUpdate();
        DDEBUG("suggestFileUpdate RTSystemItemImpl::removeConnection");
    }
}


void RTSystemItem::doPutProperties(PutPropertyFunction& putProperty)
{
    impl->doPutProperties(putProperty);
}


void RTSystemItemImpl::doPutProperties(PutPropertyFunction& putProperty)
{
    DDEBUG("RTSystemItemImpl::doPutProperties");
    putProperty(_("Auto connection"), autoConnection, changeProperty(autoConnection));
    putProperty(_("Vendor name"), vendorName, changeProperty(vendorName));
    putProperty(_("Version"), version, changeProperty(version));
    putProperty(_("State check"), stateCheck,
                [&](int value) { setStateCheckMethod(value); return true; });
    putProperty(_("Polling cycle"), pollingCycle,
                [&](int value) { changePollingPeriod(value); return true; });
    putProperty(_("Check at loading"), checkAtLoading, changeProperty(checkAtLoading));

#if defined(OPENRTM_VERSION12)
    putProperty(_("Heart-beat period"), heartBeatPeriod, changeProperty(heartBeatPeriod));
#endif
}

void RTSystemItemImpl::changePollingPeriod(int value)
{
    DDEBUG_V("RTSystemItemImpl::changePollingPeriod=%d", value);
    if (pollingCycle != value) {
        pollingCycle = value;
        sigTimerPeriodChanged(value);
    }
}

void RTSystemItemImpl::setStateCheckMethod(int value)
{
    DDEBUG_V("RTSystemItemImpl::setStateCheckMethod=%d", value);
    stateCheck.selectIndex(value);
    changeStateCheck();
}

void RTSystemItemImpl::setStateCheckMethodByString(const string& value)
{
    DDEBUG_V("RTSystemItemImpl::setStateCheckMethodByString=%s", value.c_str());
    stateCheck.select(value);
    DDEBUG_V("RTSystemItemImpl::setStateCheckMethodByString=%d", stateCheck.selectedIndex());
    changeStateCheck();
}

void RTSystemItemImpl::changeStateCheck()
{
    int state = stateCheck.selectedIndex();
    DDEBUG_V("RTSystemItemImpl::changeStateCheck=%d", state);
    switch (state) {
        case MANUAL_CHECK:
            sigTimerChanged(false);
            break;
#if defined(OPENRTM_VERSION12)
        case OBSERVER_CHECK:
            break;
#endif
        default:
            sigTimerChanged(true);
            break;
    }
}

bool RTSystemItem::loadRtsProfile(const string& filename)
{
    DDEBUG_V("RTSystemItem::loadRtsProfile=%s", filename.c_str());
    ProfileHandler::getRtsProfileInfo(filename, impl->vendorName, impl->version);
    if (ProfileHandler::restoreRtsProfile(filename, this)) {
        notifyUpdate();
        return true;
    }
    return false;
}


bool RTSystemItem::saveRtsProfile(const string& filename)
{
    if (isConsistentWithFile()) return true;
    return impl->saveRtsProfile(filename);
}


bool RTSystemItemImpl::saveRtsProfile(const string& filename)
{
    DDEBUG_V("RTSystemItem::saveRtsProfile=%s", filename.c_str());
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


int RTSystemItem::pollingCycle() const
{
    return impl->pollingCycle;
}


void RTSystemItem::setVendorName(const std::string& name)
{
    impl->vendorName = name;
}


void RTSystemItem::setVersion(const std::string& version)
{
    impl->version = version;
}

int RTSystemItem::stateCheck() const
{
    return impl->stateCheck.selectedIndex();
}

bool RTSystemItem::checkStatus()
{
    return impl->checkStatus();
}

bool RTSystemItemImpl::checkStatus()
{
    bool modified = false;

    for (auto it = rtsComps.begin(); it != rtsComps.end(); it++) {
        if (compIsAlive(it->second)) {
            if (!it->second->isAlive_) {
                modified = true;
            }
            it->second->isAlive_ = true;
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

    return modified;
}

bool RTSystemItem::isCheckAtLoading()
{
    return impl->checkAtLoading;
}
///////////
bool RTSystemItem::store(Archive& archive)
{
    if (overwrite()) {
        archive.writeRelocatablePath("filename", filePath());
        archive.write("format", fileFormat());

        archive.write("autoConnection", impl->autoConnection);
        archive.write("pollingCycle", impl->pollingCycle);
        archive.write("stateCheckMode", impl->stateCheck.selectedSymbol());
        archive.write("checkAtLoading", impl->checkAtLoading);

#if defined(OPENRTM_VERSION12)
        archive.write("heartBeatPeriod", impl->heartBeatPeriod);
#endif
        return true;
    }

    return true;
}


bool RTSystemItem::restore(const Archive& archive)
{
    DDEBUG("RTSystemItemImpl::restore");

    if (!archive.read("autoConnection", impl->autoConnection)){
        archive.read("AutoConnection", impl->autoConnection);
    }
    if(!archive.read("checkAtLoading", impl->checkAtLoading)){
        archive.read("CheckAtLoading", impl->checkAtLoading);
    }
    int pollingCycle = 1000;
    if(!archive.read("pollingCycle", pollingCycle)){
        archive.read("PollingCycle", pollingCycle);
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

    string stateCheck;
    if(!archive.read("stateCheckMode", stateCheck)){
        if(!archive.read("stateCheck", stateCheck)){
            archive.read("StateCheck", stateCheck);
        }
    }
    if(!stateCheck.empty()) {
        DDEBUG_V("StateCheck:%s", stateCheck.c_str());
        impl->setStateCheckMethodByString(stateCheck);
        archive.addPostProcess([&]() { impl->changeStateCheck(); });
    }

    return true;
}


void RTSystemItemImpl::restoreRTSystem(const Archive& archive)
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


void RTSystemItemImpl::restoreRTSComp(const string& name, const Vector2& pos,
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

SignalProxy<void(int)> RTSystemItem::sigTimerPeriodChanged()
{
    return impl->sigTimerPeriodChanged;
}

SignalProxy<void(bool)> RTSystemItem::sigTimerChanged()
{
    return impl->sigTimerChanged;
}
