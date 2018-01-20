/*!
 * @brief  This is a definition of RTSystemEditorPlugin.

 * @author Hisashi Ikari 
 * @file
 */
#include "RTSItem.h"
#include "RTSNameServerView.h"
#include "RTSCommonUtil.h"
#include "RTSDiagramView.h"
#include <cnoid/MessageView>
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/CorbaUtil>
#include <cnoid/EigenArchive>
#include <cnoid/AppConfig>

//#include <rtm/idl/SDOPackage.hh>
//#include "corba/Observer/ComponentObserver.hh"
#include "LoggerUtil.h"

#include "gettext.h"

using namespace cnoid;
using namespace std;
using namespace std::placeholders;
using namespace RTC;

namespace cnoid { 

RTSPort::RTSPort(const string& name_, PortService_var port_, RTSComp* parent)
	: rtsComp(parent) {

	isInPort = true;
	name = name_;
	port = port_;
	//
	RTC::PortProfile* profile = port->get_port_profile();
	if (profile) {
		RTC::PortInterfaceProfileList interfaceList = profile->interfaces;
		for (CORBA::ULong index = 0; index < interfaceList.length(); index++) {
			RTC::PortInterfaceProfile ifProfile = interfaceList[index];
			PortInterfacePtr portIf(new PortInterface());
			if (parent) {
				portIf->rtc_name = parent->name;
			}
			const char* port_name;
			port_name = (const char*)profile->name;
			portIf->port_name = port_name;
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

bool RTSPort::connected() {
	if (!port || !NamingContextHelper::isObjectAlive(port))
		return false;

	ConnectorProfileList_var connectorProfiles = port->get_connector_profiles();
	return connectorProfiles->length() != 0;
}


bool RTSPort::connectedWith(RTSPort* target) {
	if (!port || !target->port ||
		CORBA::is_nil(port) || port->_non_existent() ||
		CORBA::is_nil(target->port) || target->port->_non_existent())
		return false;

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


bool RTSPort::checkConnectablePort(RTSPort* target) {
	if (rtsComp == target->rtsComp) return false;
	if (!port || !target->port) return false;

	if ((isInPort && target->isInPort) ||
		(isInPort && target->isServicePort) ||
		(!isInPort && !isServicePort && !target->isInPort) ||
		(isServicePort && !target->isServicePort))
		return false;

	//In case of connection between data ports
	if (!isServicePort && !target->isServicePort) {
		vector<string> dataTypes = RTCCommonUtil::getAllowDataTypes(this, target);
		vector<string> ifTypes = RTCCommonUtil::getAllowInterfaceTypes(this, target);
		vector<string> subTypes = RTCCommonUtil::getAllowSubscriptionTypes(this, target);
		if (dataTypes.size() == 0 || ifTypes.size() == 0 || subTypes.size() == 0) return false;
	}
	return true;
}


RTSPort* RTSComp::nameToRTSPort(const string& name) {
	DDEBUG_V("RTSComp::nameToRTSPort %s", name.c_str());
	vector<RTSPortPtr>::iterator it = find_if( inPorts.begin(), inPorts.end(), RTSPortComparator(name));
	if (it != inPorts.end())
		return (*it);
	it = find_if(outPorts.begin(), outPorts.end(), RTSPortComparator(name));
	if (it != outPorts.end())
		return (*it);
	return 0;
}

vector<string> RTSPort::getDataTypes() {
	return getProperty("dataport.data_type");
}

vector<string> RTSPort::getInterfaceTypes() {
	return getProperty("dataport.interface_type");
}

vector<string> RTSPort::getDataflowTypes() {
	return getProperty("dataport.dataflow_type");
}

vector<string> RTSPort::getSubscriptionTypes() {
	return getProperty("dataport.subscription_type");
}

vector<string> RTSPort::getProperty(const string& key) {
	vector<string> result;

	NVList properties = port->get_port_profile()->properties;
	for (int index = 0; index < properties.length(); index++) {
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
//////////
//class ComponentObserverImpl : OpenRTM::ComponentObserver {
//public:
//	bool attachComponent(RTC::RTObject_ptr component);
//	void update_status(::OpenRTM::StatusKind status_kind, const char* hint);
//
//private:
//	RTC::RTObject_ptr rtc;
//};
//
//void ComponentObserverImpl::update_status(::OpenRTM::StatusKind status_kind, const char* hint) {
//	DDEBUG("ComponentObserverImpl::update_status");
//
//}
//
//bool ComponentObserverImpl::attachComponent(RTC::RTObject_ptr component) {
//	DDEBUG("ComponentObserverImpl::attachComponent");
//
//	this->rtc = component;
//
//	SDOPackage::ServiceProfile profile;
//	profile.id = "1";
//	profile.interface_type = "IDL:OpenRTM/ComponentObserver:1.0";
//	CORBA_SeqUtil::push_back(profile.properties, NVUtil::newNV("observed_status", "ALL"));
//
//	SDOPackage::Configuration_ptr conf = component->get_configuration();
//	conf->add_service_profile(profile);
//
//	return true;
//}
/////
class RTSystemItemImpl {
public:
    RTSystemItem* self;
    NamingContextHelper ncHelper;
    Connection locationChangedConnection;
    map<string, RTSCompPtr> rtsComps;

    RTSystemItem::RTSConnectionMap rtsConnections;
    int connectionNo;

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
		RTSConnection* addRTSConnection(const string& id, const string& name,
						RTSPort* sourcePort, RTSPort* targetPort, const std::vector<NamedValuePtr>& propList,
						const bool setPos, const Vector2 pos[]);
		RTSConnection* addRTSConnectionName(const string& id, const string& name,
            const string& sourceComp, const string& sourcePort,
            const string& targetComp, const string& targetPort,
            const string& dataflow, const string& subscription,
            const bool setPos, const Vector2 pos[] );
    void deleteRTSConnection(const RTSConnection* connection);
    bool connectionCheck();
    void RTSCompToConnectionList(const RTSComp* rtsComp,
            list<RTSConnection*>& rtsConnectionList, int mode);
    void restoreRTSystem( const Archive& archive);
		void diagramViewUpdate();
    void restoreRTSComp(const string& name, const Vector2& pos,
            const vector<pair<string, bool>>& inPorts, const vector<pair<string, bool>>& outPorts);
    string getConnectionNumber();
private:
//	ComponentObserverImpl* observer_;
};

}
//////////
RTSConnection::RTSConnection(const string& id, const string& name,
        const string& sourceRtcName,const string& sourcePortName,
        const string& targetRtcName, const string& targetPortName)
    : id(id), name(name), sourceRtcName(sourceRtcName), sourcePortName(sourcePortName),
      targetRtcName(targetRtcName), targetPortName(targetPortName), setPos(false)
{
  isAlive_ = false;
}


bool RTSConnection::connect() {
	DDEBUG("RTSConnection::connect");

    if(!sourcePort || !targetPort )
        return false;

		if( !sourcePort->port || !targetPort->port ||
        CORBA::is_nil(sourcePort->port) || sourcePort->port->_non_existent() ||
        CORBA::is_nil(targetPort->port) || targetPort->port->_non_existent())
            return false;

		if( sourcePort->connectedWith(targetPort) )
        return true;

		ConnectorProfile cprof;
    cprof.connector_id = CORBA::string_dup(id.c_str());
    cprof.name = CORBA::string_dup(name.c_str());
    cprof.ports.length(2);
    cprof.ports[0] = PortService::_duplicate(sourcePort->port);
    cprof.ports[1] = PortService::_duplicate(targetPort->port);

		for (int index = 0; index < propList.size(); index++) {
			NamedValuePtr param = propList[index];
			CORBA_SeqUtil::push_back(cprof.properties,
			          NVUtil::newNV(param->name_.c_str(), param->value_.c_str()));
		}

    RTC::ReturnCode_t result = sourcePort->port->connect(cprof);
		if(result == RTC::RTC_OK){
        PortProfile_var portprofile = sourcePort->port->get_port_profile();
        ConnectorProfileList connections = portprofile->connector_profiles;
        for(CORBA::ULong i = 0; i < connections.length(); i++){
            ConnectorProfile& connector = connections[i];
            PortServiceList& connectedPorts = connector.ports;
            for(CORBA::ULong j=0; j < connectedPorts.length(); ++j){
                PortService_ptr connectedPortRef = connectedPorts[j];
                if(connectedPortRef->_is_equivalent(targetPort->port)){
                    id = string(connector.connector_id);
                    isAlive_ = true;
										DDEBUG("RTSConnection::connect End");
										return true;
                }
            }
        }
				return false;
    }else{
			DDEBUG("connect Error");
			return false;
    }
}


bool RTSConnection::disConnect()
{
    isAlive_ = false;

    if(CORBA::is_nil(sourcePort->port) || sourcePort->port->_non_existent())
        return false;

    ReturnCode_t result = sourcePort->port->disconnect(id.c_str());
    if(result == RTC_OK)
        return true;
    else
        return false;
}


RTSComp::RTSComp(const string& name, RTC::RTObject_ptr rtc, RTSystemItemImpl* impl, const QPointF& pos) :
	impl(impl), pos(pos), name(name), fullPath(name) {
	setRtc(rtc);
}

RTSComp::RTSComp(const string& name, const std::string& fullPath, RTC::RTObject_ptr rtc, RTSystemItemImpl* impl, const QPointF& pos) :
	impl(impl), pos(pos), name(name), fullPath(fullPath) {
	setRtc(rtc);
}

bool RTSComp::connectionCheckSub(RTSPort* rtsPort) {
    bool add=false;

    if(!rtsPort->port || CORBA::is_nil(rtsPort->port) || rtsPort->port->_non_existent())
        return add;

    PortProfile_var portprofile = rtsPort->port->get_port_profile();
    ConnectorProfileList connectorProfiles = portprofile->connector_profiles;
    for(CORBA::ULong i = 0; i < connectorProfiles.length(); i++){
        ConnectorProfile& connectorProfile = connectorProfiles[i];
        PortServiceList& connectedPorts = connectorProfile.ports;
        for(CORBA::ULong j=0; j < connectedPorts.length(); ++j){
            PortService_ptr connectedPortRef = connectedPorts[j];
            PortProfile_var connectedPortProfile = connectedPortRef->get_port_profile();
            string portName = string(connectedPortProfile->name);
            vector<string> target;
            RTCCommonUtil::splitPortName(portName, target);
            if(target[0] == name)
                continue;
						RTSComp* targetRTC = impl->nameToRTSComp(target[0] + "|rtc::");
            if(targetRTC){
                RTSPort* targetPort = targetRTC->nameToRTSPort(portName);
                if(targetPort){
                    RTSystemItem::RTSConnectionMap::iterator itr =
                        impl->rtsConnections.find(RTSystemItem::RTSPortPair(rtsPort,targetPort));
                    if(itr!=impl->rtsConnections.end())
                        continue;
                    RTSConnectionPtr rtsConnection = new RTSConnection(
                        string(connectorProfile.connector_id), string(connectorProfile.name),
                        name, string(portprofile->name),
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
                    impl->rtsConnections[RTSystemItem::RTSPortPair(rtsPort,targetPort)] = rtsConnection;
                    add = true;
                }
            }
        }
    }

    return add;
}


bool RTSComp::connectionCheck()
{
    bool modified = false;
    for(vector<RTSPortPtr>::iterator it = inPorts.begin(); it != inPorts.end(); it++)
        modified |= connectionCheckSub(*it);
    for(vector<RTSPortPtr>::iterator it = outPorts.begin(); it != outPorts.end(); it++)
        modified |= connectionCheckSub(*it);

    return modified;
}

void RTSComp::setRtc(RTObject_ptr rtc) {
	DDEBUG("RTSComp::setRtc");
	setRTObject(rtc);

  if(!NamingContextHelper::isObjectAlive(rtc)){
    participatingExeContList = 0;
    for(vector<RTSPortPtr>::iterator it = inPorts.begin(); it != inPorts.end(); it++){
      RTSPort* port = *it;
      port->port = 0;
    }
    for(vector<RTSPortPtr>::iterator it = outPorts.begin(); it != outPorts.end(); it++){
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
    for (CORBA::ULong i = 0; i < portlist->length(); i++) {
        PortProfile_var portprofile = portlist[i]->get_port_profile();
        coil::Properties pproperties = NVUtil::toProperties(portprofile->properties);
        string portType = pproperties["port.port_type"];
        RTSPortPtr rtsPort = new RTSPort(string(portprofile->name), portlist[i], this);
				if (RTCCommonUtil::matchIgnore(portType, "CorbaPort")) {
            rtsPort->isServicePort = true;
            rtsPort->isInPort = false;
            outPorts.push_back(rtsPort);
        }else{
            rtsPort->isServicePort = false;
						if (RTCCommonUtil::matchIgnore(portType, "DataInPort"))
                inPorts.push_back(rtsPort);
            else{
                rtsPort->isInPort = false;
                outPorts.push_back(rtsPort);
            }
        }
    }

    list<RTSConnection*> rtsConnectionList;
    impl->RTSCompToConnectionList(this, rtsConnectionList, 0);
    for(list<RTSConnection*>::iterator it = rtsConnectionList.begin();
            it != rtsConnectionList.end(); it++){
        RTSConnectionPtr connection = (*it);
        impl->deleteRTSConnection(*it);
        RTSPort* sourcePort = connection->srcRTC->nameToRTSPort(connection->sourcePortName);
        RTSPort* targetPort = connection->targetRTC->nameToRTSPort(connection->targetPortName);
        if(sourcePort && targetPort){
            connection->sourcePort = sourcePort;
            connection->targetPort = targetPort;
            impl->rtsConnections[RTSystemItem::RTSPortPair(sourcePort, targetPort)] = connection;
        }
    }

    connectionCheck();
		DDEBUG("RTSComp::setRtc End");
}
/////////////////////////////////////////////////////////////////
void RTSystemItem::initialize(ExtensionManager* ext) {
	ext->itemManager().registerClass<RTSystemItem>(N_("RTSystemItem"));
	ext->itemManager().addCreationPanel<RTSystemItem>();
}


RTSystemItem::RTSystemItem() {
	impl = new RTSystemItemImpl(this);
	autoConnection = true;
  systemName = "";
}


RTSystemItemImpl::RTSystemItemImpl(RTSystemItem* self)
	: self(self) {
	initialize();
}


RTSystemItem::RTSystemItem(const RTSystemItem& org)
	: Item(org) {
	impl = new RTSystemItemImpl(this, *org.impl);
	autoConnection = org.autoConnection;
}


RTSystemItemImpl::RTSystemItemImpl(RTSystemItem* self, const RTSystemItemImpl& org)
	: self(self) {
	initialize();
}

RTSystemItem::~RTSystemItem() {
	delete impl;
}

RTSystemItemImpl::~RTSystemItemImpl() {
	locationChangedConnection.disconnect();
}

Item* RTSystemItem::doDuplicate() const {
	return new RTSystemItem(*this);
}

void RTSystemItemImpl::initialize() {
	RTSNameServerView* nsView = RTSNameServerView::instance();
	if (nsView) {
		if (!locationChangedConnection.connected()) {
			locationChangedConnection = nsView->sigLocationChanged().connect(
				std::bind(&RTSystemItemImpl::onLocationChanged, this, _1, _2));
			ncHelper.setLocation(nsView->getHost(), nsView->getPort());
		}
	}
	connectionNo = 0;
}

void RTSystemItemImpl::onLocationChanged(string host, int port) {
	ncHelper.setLocation(host, port);
}

RTSComp* RTSystemItem::nameToRTSComp(const string& name) {
    return impl->nameToRTSComp(name);
}

RTSComp* RTSystemItemImpl::nameToRTSComp(const string& name) {
	//DDEBUG_V("RTSystemItemImpl::nameToRTSComp:%s", name.c_str());
    map<string, RTSCompPtr>::iterator it = rtsComps.find(name);
    if(it==rtsComps.end())
        return 0;
    else
        return it->second.get();
}


RTSComp* RTSystemItem::addRTSComp(const string& name, const QPointF& pos) {
	return impl->addRTSComp(name, pos);
}


RTSComp* RTSystemItemImpl::addRTSComp(const string& name, const QPointF& pos) {
	DDEBUG_V("RTSystemItemImpl::addRTSComp:%s", name.c_str());

	if (!nameToRTSComp("/" + name + ".rtc")) {
		RTC::RTObject_ptr rtc = ncHelper.findObject<RTC::RTObject>(name, "rtc");
		if (rtc == RTC::RTObject::_nil()) {
			DDEBUG("RTSystemItemImpl::addRTSComp Failed");
			return 0;
		}

		string fullPath = "/" + name + ".rtc";
		RTSCompPtr rtsComp = new RTSComp(name, fullPath, rtc, this, pos);
		rtsComps[fullPath] = rtsComp;

//		ComponentObserverImpl* obs = new ComponentObserverImpl();
//		obs->attachComponent(rtsComp);

		return rtsComp.get();
	}
	return 0;
}

RTSComp* RTSystemItem::addRTSComp(const  NamingContextHelper::ObjectInfo& info, const QPointF& pos) {
	return impl->addRTSComp(info, pos);
}

RTSComp* RTSystemItemImpl::addRTSComp(const NamingContextHelper::ObjectInfo& info, const QPointF& pos) {
	DDEBUG("RTSystemItemImpl::addRTSComp");
	string fullPath = info.getFullPath();
	if (!nameToRTSComp(fullPath)) {
		std::vector<NamingContextHelper::ObjectPath> target = info.fullPath;
		RTC::RTObject_ptr rtc = ncHelper.findObject<RTC::RTObject>(target);
		if (rtc == RTC::RTObject::_nil()) return 0;
		if (ncHelper.isObjectAlive(rtc) == false) return 0;

		RTSCompPtr rtsComp = new RTSComp(info.id, fullPath, rtc, this, pos);
		rtsComps[fullPath] = rtsComp;

		//		ComponentObserverImpl* obs = new ComponentObserverImpl();
		//		obs->attachComponent(rtsComp);

		return rtsComp.get();
	}
	return 0;
}

void RTSystemItem::deleteRTSComp(const string& name)
{
    impl->deleteRTSComp(name);
}


void RTSystemItemImpl::deleteRTSComp(const string& name)
{
    rtsComps.erase(name);
}

bool RTSystemItem::compIsAlive(RTSComp* rtsComp)
{
    return impl->compIsAlive(rtsComp);
}

bool RTSystemItemImpl::compIsAlive(RTSComp* rtsComp)
{
    if(!rtsComp->rtc_){
        RTC::RTObject_ptr rtc = ncHelper.findObject<RTC::RTObject>(rtsComp->name, "rtc");
        if(ncHelper.isObjectAlive(rtc)){
            rtsComp->setRtc(rtc);
            list<RTSConnection*> rtsConnectionList;
            RTSCompToConnectionList(rtsComp, rtsConnectionList, 0);
            for(list<RTSConnection*>::iterator it = rtsConnectionList.begin();
                    it != rtsConnectionList.end(); it++){
                RTSConnection& connection = *(*it);
                if(self->autoConnection){
                    connection.connect();
                }
            }
            return true;
        }else
            return false;
    }else{
        if(ncHelper.isObjectAlive(rtsComp->rtc_))
            return true;
        else{
            rtsComp->setRtc(0);
            return false;
        }
    }

}

RTSConnection* RTSystemItem::addRTSConnection(const std::string& id, const std::string& name,
						RTSPort* sourcePort, RTSPort* targetPort, const std::vector<NamedValuePtr>& propList) {
	return impl->addRTSConnection(id, name, sourcePort, targetPort, propList, false, 0);
}

RTSConnection* RTSystemItemImpl::addRTSConnectionName(const string& id, const string& name,
	const string& sourceCompName, const string& sourcePortName,
	const string& targetCompName, const string& targetPortName,
	const string& dataflow, const string& subscription,
	const bool setPos, const Vector2 pos[]) {

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
	return 0;
}

string RTSystemItemImpl::getConnectionNumber()
{
    stringstream ss;
    ss << connectionNo;
    connectionNo++;
    return ss.str();
}

RTSConnection* RTSystemItemImpl::addRTSConnection(const string& id, const string& name,
			RTSPort* sourcePort, RTSPort* targetPort, const std::vector<NamedValuePtr>& propList,
			const bool setPos, const Vector2 pos[]) {
	DDEBUG("RTSystemItemImpl::addRTSConnection");

	RTSConnection* rtsConnection_;
	RTSystemItem::RTSConnectionMap::iterator it = rtsConnections.find(
		RTSystemItem::RTSPortPair(sourcePort, targetPort));

	if (it == rtsConnections.end()) {
		RTSConnectionPtr rtsConnection = new RTSConnection(
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
		rtsConnections[RTSystemItem::RTSPortPair(sourcePort, targetPort)] =
			rtsConnection;
		rtsConnection_ = rtsConnection.get();
	} else {
		rtsConnection_ = it->second;;
	}

	if (!CORBA::is_nil(sourcePort->port) && !sourcePort->port->_non_existent() &&
		!CORBA::is_nil(targetPort->port) && !targetPort->port->_non_existent()) {
		rtsConnection_->connect();
	}

	if (rtsConnection_->id == "") {
		rtsConnection_->id = "NoConnection_" + getConnectionNumber();
	}

	return rtsConnection_;
}


bool RTSystemItem::connectionCheck()
{
    return impl->connectionCheck();
}


bool RTSystemItemImpl::connectionCheck()
{
    bool modified = false;
    for(RTSystemItem::RTSConnectionMap::iterator it= rtsConnections.begin();
            it != rtsConnections.end(); it++){
        const RTSystemItem::RTSPortPair& ports = it->first;
        if(ports(0)->connectedWith(ports(1))){
            if(!it->second->isAlive_){
                it->second->isAlive_ = true;
                modified = true;
            }
        }else{
            if(it->second->isAlive_){
                it->second->isAlive_ = false;
                modified = true;
            }
        }
    }

    for(map<string, RTSCompPtr>::iterator it = rtsComps.begin();
            it != rtsComps.end(); it++){
        modified |= it->second->connectionCheck();
    }

    return modified;
}


void RTSystemItem::RTSCompToConnectionList(const RTSComp* rtsComp,
        list<RTSConnection*>& rtsConnectionList, int mode)
{
    impl->RTSCompToConnectionList(rtsComp, rtsConnectionList, mode);
}


void RTSystemItemImpl::RTSCompToConnectionList(const RTSComp* rtsComp,
        list<RTSConnection*>& rtsConnectionList, int mode)
{
    for(RTSystemItem::RTSConnectionMap::iterator it = rtsConnections.begin();
            it != rtsConnections.end(); it++){
        switch (mode){
        case 0:
        default :
            if(it->second->sourceRtcName == rtsComp->name || it->second->targetRtcName == rtsComp->name)
                rtsConnectionList.push_back(it->second);
            break;
        case 1:
            if(it->second->sourceRtcName == rtsComp->name)
                rtsConnectionList.push_back(it->second);
            break;
        case 2:
            if(it->second->targetRtcName == rtsComp->name)
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


void RTSystemItem::deleteRtsConnection(const RTSConnection* connection)
{
    impl->deleteRTSConnection(connection);
}


void RTSystemItemImpl::deleteRTSConnection(const RTSConnection* connection)
{
    RTSystemItem::RTSPortPair pair(connection->sourcePort, connection->targetPort);
    rtsConnections.erase(pair);
}


void RTSystemItem::doPutProperties(PutPropertyFunction& putProperty) {
  putProperty(_("Auto Connection"), autoConnection, changeProperty(autoConnection));

  putProperty(_("System Name"), systemName, changeProperty(systemName));
  putProperty(_("Vendor Name"), vendorName, changeProperty(vendorName));
  putProperty(_("Version"), version, changeProperty(version));
  putProperty(_("Profile Path"), profileFileName, changeProperty(profileFileName));
}

struct ConnectorPropComparator {
	string target_;

	ConnectorPropComparator(string value) {
		target_ = value;
	}
	bool operator()(const NamedValuePtr elem) const {
		return (target_ == elem->name_);
	}
};
///////////
bool RTSystemItem::store(Archive& archive) {
  if (systemName.length() == 0) {
    MessageView::instance()->putln(MessageView::ERROR, _("System name is not set."));
    return false;
  }
  if (vendorName.length() == 0) {
    MessageView::instance()->putln(MessageView::ERROR, _("Vendor is not set."));
    return false;
  }
  if (version.length() == 0) {
    MessageView::instance()->putln(MessageView::ERROR, _("Version is not set."));
    return false;
  }
  if (profileFileName.length() == 0) {
    MessageView::instance()->putln(MessageView::ERROR, _("RtsProfile file name is not set."));
    return false;
  }

	archive.write("AutoConnection", autoConnection);

	string currentFolder;
	AppConfig::archive()->read("currentFileDialogDirectory", currentFolder);
	DDEBUG_V("path:%s", currentFolder.c_str());

	archive.write("file", profileFileName);
  string systemId = "RTSystem:" + vendorName + ":" + systemName + ":" + version;
	string hostName = impl->ncHelper.host();
	ProfileHandler::saveRtsProfile(currentFolder + "/" + profileFileName, systemId, hostName,
																	impl->rtsComps, impl->rtsConnections);

  return true;
}
/////

bool RTSystemItem::restore(const Archive& archive) {
	DDEBUG("RTSystemItemImpl::restore");

	archive.read("AutoConnection", autoConnection);
  archive.addPostProcess(
          std::bind(&RTSystemItemImpl::restoreRTSystem, impl, std::ref(archive)));

  return true;
}

void RTSystemItemImpl::restoreRTSystem(const Archive& archive) {
	DDEBUG("RTSystemItemImpl::restoreRTSystem");

	string targetFile;
	archive.read("file", targetFile);
	DDEBUG_V("targetFile:%s", targetFile.c_str());
	if (0 < targetFile.length()) {
    self->profileFileName = targetFile;
		string currentFolder;
		AppConfig::archive()->read("currentFileDialogDirectory", currentFolder);
		if (ProfileHandler::restoreRtsProfile(currentFolder + "/" + targetFile, self) == false) return;

	} else {
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

		if (self->autoConnection) {
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
	}

  diagramViewUpdate();
}

void RTSystemItemImpl::restoreRTSComp(const string& name, const Vector2& pos,
        const vector<pair<string, bool>>& inPorts, const vector<pair<string, bool>>& outPorts) {
	DDEBUG("RTSystemItemImpl::restoreRTSComp");

	RTSComp* comp = addRTSComp(name, QPointF(pos(0), pos(1)) );
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


void RTSystemItemImpl::diagramViewUpdate()
{
    RTSNameServerView* nsView = RTSNameServerView::instance();
    if(nsView){
        nsView->updateView();
    }
    RTSDiagramView* diagramView = RTSDiagramView::instance();
    if(diagramView){
        diagramView->updateView();
    }
}
