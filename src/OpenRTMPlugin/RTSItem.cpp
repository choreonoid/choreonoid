/*!
 * @brief  This is a definition of RTSystemEditorPlugin.

 * @author Hisashi Ikari 
 * @file
 */
#include "RTSItem.h"
#include "RTSNameServerView.h"
#include "RTSCommonUtil.h"
#include "RTSDiagramView.h"
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/CorbaUtil>
#include <cnoid/EigenArchive>
#include <boost/algorithm/string.hpp>

#include "gettext.h"

using namespace cnoid;
using namespace std;
using namespace std::placeholders;
using namespace RTC;

namespace cnoid 
{ 

class RTSystemItemImpl
{
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
    void deleteRTSComp(const string& name);
    RTSComp* nameToRTSComp(const string& name);
    bool compIsAlive(RTSComp* rtsComp);
    RTSConnection* addRTSConnection(const string& id, const string& name,
            RTSPort* sourcePort, RTSPort* targetPort, const string& dataflow, const string& subscription,
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
};

}

RTSPort::RTSPort(const string& name_, PortService_var port_, RTSComp* parent)
    : rtsComp(parent)
{
    isInPort = true;
    name = name_;
    port = port_;
}


bool RTSPort::connected()
{
    if(!port || !NamingContextHelper::isObjectAlive(port))
        return false;

    ConnectorProfileList_var connectorProfiles = port->get_connector_profiles();
    return connectorProfiles->length()!=0;
}


bool RTSPort::connectedWith(RTSPort* target)
{
    if( !port || !target->port ||
        CORBA::is_nil(port) || port->_non_existent() ||
        CORBA::is_nil(target->port) || target->port->_non_existent())
            return false;

    ConnectorProfileList_var connectorProfiles = port->get_connector_profiles();
    for(CORBA::ULong i=0; i < connectorProfiles->length(); ++i){
        ConnectorProfile& connectorProfile = connectorProfiles[i];
        PortServiceList& connectedPorts = connectorProfile.ports;

        for(CORBA::ULong j=0; j < connectedPorts.length(); ++j){
            PortService_ptr connectedPortRef = connectedPorts[j];
            if(connectedPortRef->_is_equivalent(target->port)){
                return true;
            }
        }
    }
    return false;
}


bool RTSPort::checkConnectablePort(RTSPort* target)
{
    if(rtsComp == target->rtsComp)
        return false;
    if( !port || !target->port )
        return false;
    if((isInPort && target->isInPort) ||
       (isInPort && target->isServicePort) ||
       (!isInPort && !isServicePort && !target->isInPort) ||
       (isServicePort && !target->isServicePort))
        return false;

    return true;
}


RTSPort* RTSComp::nameToRTSPort(const string& name)
{
    map<string, RTSPortPtr>::iterator it = inPorts.find(name);
    if(it!=inPorts.end())
        return it->second.get();
    it = outPorts.find(name);
    if(it!=outPorts.end())
        return it->second.get();
    return 0;
}

RTSConnection::RTSConnection(const string& id, const string& name,
        const string& sourceRtcName,const string& sourcePortName,
        const string& targetRtcName, const string& targetPortName)
    : id(id), name(name), sourceRtcName(sourceRtcName), sourcePortName(sourcePortName),
      targetRtcName(targetRtcName), targetPortName(targetPortName), setPos(false)
{
    pushPolicy = "all";
    pushRate = 1000.0;
    sinterface = "corba_cdr";
    isAlive_ = false;
}


bool RTSConnection::connect()
{
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

    CORBA_SeqUtil::push_back(cprof.properties,
            NVUtil::newNV("dataport.dataflow_type",
                    dataflow.c_str()));
    CORBA_SeqUtil::push_back(cprof.properties,
            NVUtil::newNV("dataport.interface_type",
                    sinterface.c_str()));
    CORBA_SeqUtil::push_back(cprof.properties,
            NVUtil::newNV("dataport.subscription_type",
                    subscription.c_str()));

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
                    return true;
                }
            }
        }
        return false;
    }else{
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
        impl(impl),
        name(name),
        pos(pos)
{
    setRtc(rtc);
}


bool RTSComp::connectionCheckSub(RTSPort* rtsPort)
{
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
            RTSComp* targetRTC = impl->nameToRTSComp(target[0]);
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
                    rtsConnection->dataflow = properties["dataport.dataflow_type"];
                    rtsConnection->subscription = properties["dataport.subscription_type"];
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
    for(map<string, RTSPortPtr>::iterator it = inPorts.begin(); it != inPorts.end(); it++)
        modified |= connectionCheckSub(it->second.get());
    for(map<string, RTSPortPtr>::iterator it = outPorts.begin(); it != outPorts.end(); it++)
        modified |= connectionCheckSub(it->second.get());

    return modified;
}


bool RTSComp::isActive()
{
    if(CORBA::is_nil(rtc_) || rtc_->_non_existent())
        return false;

    for(CORBA::ULong i=0; i<ownedExeContList->length(); i++){
        if(ownedExeContList[i]->get_component_state(rtc_) == RTC::ACTIVE_STATE)
            return true;
    }
    for(CORBA::ULong i=0; i<participatingExeContList->length(); i++){
        if(participatingExeContList[i]->get_component_state(rtc_) == RTC::ACTIVE_STATE)
            return true;
    }
    return false;
}

void RTSComp::setRtc(RTObject_ptr rtc)
{
    rtc_ = 0;

    if(!NamingContextHelper::isObjectAlive(rtc)){
        ownedExeContList = 0;
        participatingExeContList = 0;
        for(map<string, RTSPortPtr>::iterator it = inPorts.begin();
                it != inPorts.end(); it++){
            RTSPort* port = it->second.get();
            port->port = 0;
        }
        for(map<string, RTSPortPtr>::iterator it = outPorts.begin();
                it != outPorts.end(); it++){
            RTSPort* port = it->second.get();
            port->port = 0;
        }
        return;
    }

    rtc_ = rtc;
    ComponentProfile_var cprofile = rtc_->get_component_profile();

    ownedExeContList = rtc_->get_owned_contexts();
    participatingExeContList = rtc_->get_participating_contexts();

    inPorts.clear();
    outPorts.clear();

    PortServiceList_var portlist = rtc_->get_ports();
    for (CORBA::ULong i = 0; i < portlist->length(); i++) {
        PortProfile_var portprofile = portlist[i]->get_port_profile();
        coil::Properties pproperties = NVUtil::toProperties(portprofile->properties);
        string portType = pproperties["port.port_type"];
        RTSPortPtr rtsPort = new RTSPort(string(portprofile->name), portlist[i], this);
        if (boost::iequals(portType, "CorbaPort")){
            rtsPort->isServicePort = true;
            rtsPort->isInPort = false;
            outPorts.insert(pair<string, RTSPortPtr>(rtsPort->name, rtsPort));
        }else{
            rtsPort->isServicePort = false;
            if(boost::iequals(portType, "DataInPort"))
                inPorts.insert(pair<string, RTSPortPtr>(rtsPort->name, rtsPort));
            else{
                rtsPort->isInPort = false;
                outPorts.insert(pair<string, RTSPortPtr>(rtsPort->name, rtsPort));
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
}


/////////////////////////////////////////////////////////////////
void RTSystemItem::initialize(ExtensionManager* ext)
{
    ext->itemManager().registerClass<RTSystemItem>(N_("RTSystemItem"));
    ext->itemManager().addCreationPanel<RTSystemItem>();
}


RTSystemItem::RTSystemItem()
{
    impl = new RTSystemItemImpl(this);
    autoConnection = true;
}


RTSystemItemImpl::RTSystemItemImpl(RTSystemItem* self)
    : self(self)
{
    initialize();
}


RTSystemItem::RTSystemItem(const RTSystemItem& org)
    : Item(org)
{
    impl = new RTSystemItemImpl(this, *org.impl);
    autoConnection = org.autoConnection;
}


RTSystemItemImpl::RTSystemItemImpl(RTSystemItem* self, const RTSystemItemImpl& org)
    : self(self)
{
    initialize();
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


void RTSystemItemImpl::initialize()
{
    RTSNameServerView* nsView = RTSNameServerView::instance();
    if(nsView){
        if(!locationChangedConnection.connected()){
            locationChangedConnection = nsView->sigLocationChanged().connect(
                    std::bind(&RTSystemItemImpl::onLocationChanged, this, _1, _2));
            ncHelper.setLocation(nsView->getHost(), nsView->getPort());
        }
    }
    connectionNo = 0;
}


void RTSystemItemImpl::onLocationChanged(string host, int port)
{
    ncHelper.setLocation(host, port);
}


RTSComp* RTSystemItem::nameToRTSComp(const string& name)
{
    return impl->nameToRTSComp(name);
}


RTSComp* RTSystemItemImpl::nameToRTSComp(const string& name)
{
    map<string, RTSCompPtr>::iterator it = rtsComps.find(name);
    if(it==rtsComps.end())
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
    if(!nameToRTSComp(name)){
        RTC::RTObject_ptr rtc = ncHelper.findObject<RTC::RTObject>(name, "rtc");
        RTSCompPtr rtsComp = new RTSComp(name, rtc, this, pos);
        rtsComps[name] = rtsComp;
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

RTSConnection* RTSystemItem::addRTSConnection(const string& id, const string& name,
            RTSPort* sourcePort, RTSPort* targetPort, const string& dataflow, const string& subscription)
{
    return impl->addRTSConnection(id, name, sourcePort, targetPort, dataflow, subscription, false, 0 );
}


RTSConnection* RTSystemItemImpl::addRTSConnectionName(const string& id, const string& name,
        const string& sourceCompName, const string& sourcePortName,
        const string& targetCompName, const string& targetPortName,
        const string& dataflow, const string& subscription,
        const bool setPos, const Vector2 pos[] )
{
    RTSPort* sourcePort{};
    RTSPort* targetPort{};
    RTSComp* sourceRtc = nameToRTSComp(sourceCompName);
    if(sourceRtc){
        sourcePort = sourceRtc->nameToRTSPort(sourcePortName);
    }
    RTSComp* targetRtc = nameToRTSComp(targetCompName);
    if(targetRtc){
        targetPort = targetRtc->nameToRTSPort(targetPortName);
    }
    if(sourcePort && targetPort){
        return addRTSConnection(id, name, sourcePort, targetPort, dataflow, subscription, setPos, pos);
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
            RTSPort* sourcePort, RTSPort* targetPort, const string& dataflow, const string& subscription,
            const bool setPos, const Vector2 pos[] )
{
    RTSConnection* rtsConnection_;
    RTSystemItem::RTSConnectionMap::iterator it = rtsConnections.find(
            RTSystemItem::RTSPortPair(sourcePort, targetPort));

    if(it==rtsConnections.end()){
        RTSConnectionPtr rtsConnection = new RTSConnection(
            id, name,
            sourcePort->rtsComp->name, sourcePort->name,
            targetPort->rtsComp->name, targetPort->name);
        rtsConnection->srcRTC = sourcePort->rtsComp;
        rtsConnection->sourcePort = sourcePort;
        rtsConnection->targetRTC = targetPort->rtsComp;
        rtsConnection->targetPort = targetPort;
        rtsConnection->dataflow = dataflow;
        rtsConnection->subscription = subscription;
        if(setPos){
            rtsConnection->setPosition(pos);
        }
        rtsConnections[RTSystemItem::RTSPortPair(sourcePort, targetPort)] =
                    rtsConnection;
        rtsConnection_ = rtsConnection.get();
    }else{
        rtsConnection_ = it->second;;
    }

    if(!CORBA::is_nil(sourcePort->port) && !sourcePort->port->_non_existent() &&
       !CORBA::is_nil(targetPort->port) && !targetPort->port->_non_existent()){
        rtsConnection_->connect();
    }

    if(rtsConnection_->id==""){
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


void RTSystemItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Auto Connection"), autoConnection, changeProperty(autoConnection));
}


bool RTSystemItem::store(Archive& archive)
{
    archive.write("AutoConnection", autoConnection);

    map<string, RTSCompPtr>& comps = impl->rtsComps;

    ListingPtr compListing = new Listing();
    for(map<string, RTSCompPtr>::iterator it = comps.begin();
                it != comps.end(); it++){
        RTSComp* comp = it->second.get();
        Mapping* compMap = compListing->newMapping();
        compMap->write("name", comp->name);
        write( *compMap, "pos", Vector2(comp->pos.x(), comp->pos.y()) );

        ListingPtr inportListing = new Listing();
        for(map<string, RTSPortPtr>::iterator p0 = comp->inPorts.begin();
                p0 != comp->inPorts.end(); p0++){
            RTSPort* in = p0->second.get();
            Mapping* inMap = inportListing->newMapping();
            inMap->write("name", in->name);
            inMap->write("isServicePort", in->isServicePort);
        }
        if(!inportListing->empty()){
            compMap->insert("InPorts", inportListing);
        }

        ListingPtr outportListing = new Listing();
        for(map<string, RTSPortPtr>::iterator p0 = comp->outPorts.begin();
                p0 != comp->outPorts.end(); p0++){
            RTSPort* out = p0->second.get();
            Mapping* outMap = outportListing->newMapping();
            outMap->write("name", out->name);
            outMap->write("isServicePort", out->isServicePort);
        }
        if(!outportListing->empty()){
            compMap->insert("OutPorts", outportListing);
        }

    }
    if(!compListing->empty()){
        archive.insert("RTSComps", compListing);
    }

    RTSConnectionMap& connections = impl->rtsConnections;

    ListingPtr connectionListing = new Listing();
    for(RTSConnectionMap::iterator it = connections.begin();
                it != connections.end(); it++){
        RTSConnection* connect = it->second.get();
        Mapping* connectMap = connectionListing->newMapping();
        connectMap->write("name", connect->name);
        connectMap->write("sourceRtcName", connect->sourceRtcName);
        connectMap->write("sourcePortName", connect->sourcePortName);
        connectMap->write("targetRtcName", connect->targetRtcName);
        connectMap->write("targetPortName", connect->targetPortName);
        connectMap->write("dataflow", connect->dataflow);
        connectMap->write("subscription", connect->subscription);
        if(connect->setPos){
            VectorXd p(12);
            for(int i=0; i<6; i++){
                p(2*i) = connect->position[i](0);
                p(2*i+1) = connect->position[i](1);
            }
            write( *connectMap, "position", p );
        }
    }
    if(!connectionListing->empty()){
        archive.insert("RTSConnections", connectionListing);
    }

    return true;

}


bool RTSystemItem::restore(const Archive& archive)
{
    archive.read("AutoConnection", autoConnection);
    archive.addPostProcess(
            std::bind(&RTSystemItemImpl::restoreRTSystem, impl, std::ref(archive)));

    return true;
}

void RTSystemItemImpl::restoreRTSystem(const Archive& archive)
{
    const Listing& compListing = *archive.findListing("RTSComps");
    if(compListing.isValid()){
        for(int i=0; i < compListing.size(); i++){
            const Mapping& compMap = *compListing[i].toMapping();
            string name;
            Vector2 pos;
            compMap.read("name", name);
            read( compMap, "pos", pos);

            vector<pair<string, bool>> inPorts;
            vector<pair<string, bool>> outPorts;
            const Listing& inportListing = *compMap.findListing("InPorts");
            if(inportListing.isValid()){
                for(int i=0; i < inportListing.size(); i++){
                    const Mapping& inMap = *inportListing[i].toMapping();
                    string portName;
                    bool isServicePort;
                    inMap.read("name", portName);
                    inMap.read("isServicePort", isServicePort);
                    inPorts.push_back(make_pair(portName, isServicePort));
                }
            }
            const Listing& outportListing = *compMap.findListing("OutPorts");
            if(outportListing.isValid()){
                for(int i=0; i < outportListing.size(); i++){
                    const Mapping& outMap = *outportListing[i].toMapping();
                    string portName;
                    bool isServicePort;
                    outMap.read("name", portName);
                    outMap.read("isServicePort", isServicePort);
                    outPorts.push_back(make_pair(portName, isServicePort));
                }
            }
            restoreRTSComp( name, pos, inPorts, outPorts );
        }
    }

    if(self->autoConnection){
        const Listing& connectionListing = *archive.findListing("RTSConnections");
        if(connectionListing.isValid()){
            for(int i=0; i<connectionListing.size(); i++){
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
                bool readPos=false;
                Vector2 pos[6];
                if( read( connectMap, "position", p) ){
                    readPos=true;
                    for(int i=0; i<6; i++){
                        pos[i] << p(2*i) , p(2*i+1);
                    }
                }
                addRTSConnectionName( "", name,
                        sR, sP, tR, tP, dataflow, subscription, readPos, pos);
            }
        }
    }

    diagramViewUpdate();

}


void RTSystemItemImpl::restoreRTSComp(const string& name, const Vector2& pos,
        const vector<pair<string, bool>>& inPorts, const vector<pair<string, bool>>& outPorts)
{
    RTSComp* comp = addRTSComp(name, QPointF(pos(0), pos(1)) );
    if(!comp->rtc_){
        comp->inPorts.clear();
        comp->outPorts.clear();
        for(size_t i=0; i < inPorts.size(); i++){
            RTSPortPtr rtsPort = new RTSPort(inPorts[i].first, 0, comp);
            rtsPort->isInPort = true;
            rtsPort->isServicePort = inPorts[i].second;
            comp->inPorts.insert(pair<string, RTSPortPtr>(rtsPort->name, rtsPort));
        }
        for(size_t i=0; i < outPorts.size(); i++){
            RTSPortPtr rtsPort = new RTSPort(outPorts[i].first, 0, comp);
            rtsPort->isInPort = false;
            rtsPort->isServicePort = outPorts[i].second;
            comp->outPorts.insert(pair<string, RTSPortPtr>(rtsPort->name, rtsPort));
        }
    }
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
