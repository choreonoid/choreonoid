
#include "RTSystemExtItem.h"
#include "ProfileHandlerExt.h"
#include "RTSCommonUtil.h"
#include "LoggerUtil.h"
#include <rtm/idl/RTC.hh>
#include <QDateTime>
#include <QString>
#include <QStringList>

using namespace pugi;
using namespace std;
using namespace RTC;

namespace cnoid {

struct PropertyValueComparator
{
    QString target_;

    PropertyValueComparator(string value)
    {
        target_ = QString::fromStdString(value);
    }
    bool operator()(const Property elem) const
    {
        return (QString::fromStdString(elem.name).startsWith(target_));
    }
};

bool ProfileHandlerExt::getRtsProfileInfo(std::string targetFile, std::string& vendorName, std::string& version)
{
    RtsProfile profile;
    if (parseProfile(targetFile, profile) == false) return false;
    ///
    QString strId = QString::fromStdString(profile.id);
    QStringList elems = strId.split(":");
    if (elems.size() < 4) return false;
    vendorName = elems.at(1).toStdString();
    version = elems.at(3).toStdString();
    return true;
}

bool ProfileHandlerExt::restoreRtsProfile(std::string targetFile, RTSystemExtItem* rts)
{
    DDEBUG("ProfileHandler::restoreRtsProfile");
    RtsProfile profile;
    if (parseProfile(targetFile, profile) == false) return false;
    ///
    QString strId = QString::fromStdString(profile.id);
    QStringList elems = strId.split(":");
    if (elems.size() < 4) return false;
    rts->setVendorName(elems.at(1).toStdString());
    rts->setVersion(elems.at(3).toStdString());
    ///
    for (int index = 0; index < profile.compList.size(); index++) {
        Component compProf = profile.compList[index];

        QString name = QString::fromStdString(compProf.pathUri);
        QStringList nameList = name.split("/");
        QString hostName = nameList[0];
        nameList.removeAt(0);

        bool isSkip = false;
        NamingContextHelper::ObjectInfo info;
        for (int index = 0; index < nameList.count(); index++) {
            QString elem = nameList[index];
            QStringList elemList = elem.split(".");
            if (elemList.size() != 2) {
                isSkip = true;
                break;
            }
            NamingContextHelper::ObjectPath path(elemList[0].toStdString(), elemList[1].toStdString());
            info.fullPath_.push_back(path);
        }
        if (isSkip) continue;
        info.id_ = compProf.instanceName;

        if(compProf.isRegisteredInRtmDefaultNameServer) {
            NameServerInfo ns = RTCCommonUtil::getManagerAddress();
            info.hostAddress_ = ns.hostAddress;
            info.portNo_ = ns.portNo;
        } else {
            info.hostAddress_ = hostName.toStdString();
            info.portNo_ = 2809;
        }

        Vector2 pos;
        pos.x() = compProf.posX;
        pos.y() = compProf.posY;

        DDEBUG_V("addRTSComp: %s, %s, host=%s, port=%d", info.id_.c_str(), info.getFullPath().c_str(), info.hostAddress_.c_str(), info.portNo_);
        RTSCompExt* comp = rts->addRTSComp(info, QPointF(pos(0), pos(1)));
        if (comp == 0) continue;
        if (CORBA::is_nil(comp->rtc_)){
            comp->inPorts.clear();
            comp->outPorts.clear();
            for (int idxData = 0; idxData < compProf.dataPortList.size(); idxData++) {
                DataPort portProf = compProf.dataPortList[idxData];
                RTSPortExtPtr rtsPort = new RTSPortExt(portProf.name, 0, comp);
                rtsPort->isServicePort = false;
                if (portProf.direction == "DataOutPort") {
                    rtsPort->isInPort = false;
                    comp->outPorts.push_back(rtsPort);
                } else {
                    rtsPort->isInPort = true;
                    comp->inPorts.push_back(rtsPort);
                }
            }
            for (int idxService = 0; idxService < compProf.servicePortList.size(); idxService++) {
                ServicePort portProf = compProf.servicePortList[idxService];
                RTSPortExtPtr rtsPort = new RTSPortExt(portProf.name, 0, comp);
                rtsPort->isServicePort = true;
                rtsPort->isInPort = false;
                comp->outPorts.push_back(rtsPort);
            }
            comp->profile = compProf;
        }
        /////
        if (isObjectAlive(comp->rtc_)) {
            string activeConfig = compProf.activeConfigurationSet;
            DDEBUG_V("activeConfig:%s", activeConfig.c_str());
            vector<ConfigurationSet> configList = compProf.configList;
            SDOPackage::Configuration_ptr configuration = comp->rtc_->get_configuration();
            SDOPackage::ConfigurationSetList_var confSet = configuration->get_configuration_sets();
            for (int idxSet = 0; idxSet < configList.size(); idxSet++) {
                ConfigurationSet configSetProf = configList[idxSet];
                bool isExist = false;
                SDOPackage::ConfigurationSet confRaw;
                for (int idxConf = 0; idxConf < confSet->length(); idxConf++) {
                    string rawId = string(confSet[idxConf].id);
                    if (configSetProf.id == rawId) {
                        isExist = true;
                        confRaw = confSet[idxConf];
                        break;
                    }
                }
                if (isExist) {
                    DDEBUG("Cinfiguration UPDATE");
                    NVList configList;
                    for (int idxDetail = 0; idxDetail < configSetProf.dataList.size(); idxDetail++) {
                        string name = configSetProf.dataList[idxDetail].name;
                        string value = configSetProf.dataList[idxDetail].value;
                        CORBA_SeqUtil::push_back(configList, NVUtil::newNV(name.c_str(), value.c_str()));
                    }
                    confRaw.configuration_data = configList;
                    configuration->set_configuration_set_values(confRaw);

                } else {
                    DDEBUG("Cinfiguration INSERT");
                    SDOPackage::ConfigurationSet newSet;
                    newSet.id = CORBA::string_dup(configSetProf.id.c_str());
                    NVList configList;
                    for (int idxDetail = 0; idxDetail < configSetProf.dataList.size(); idxDetail++) {
                        CORBA_SeqUtil::push_back(configList,
                            NVUtil::newNV(configSetProf.dataList[idxDetail].name.c_str(), configSetProf.dataList[idxDetail].value.c_str()));
                    }
                    newSet.configuration_data = configList;
                    configuration->add_configuration_set(newSet);
                }
            }
            configuration->activate_configuration_set(activeConfig.c_str());
        }
    }
    /////
    for (int idxData = 0; idxData < profile.dataConnList.size(); idxData++) {
        DataPortConnector dataConProf = profile.dataConnList[idxData];
        RTSPortExt* sourcePort = getTargetPort(dataConProf.source.pathId, dataConProf.source.portName, rts);
        RTSPortExt* targetPort = getTargetPort(dataConProf.target.pathId, dataConProf.target.portName, rts);

        string id = "";
        string name = dataConProf.name;

        vector<NamedValueExtPtr> propList;
        for (int idxProp = 0; idxProp < dataConProf.propertyList.size(); idxProp++) {
            Property propPro = dataConProf.propertyList[idxProp];
            if (propPro.name == "dataport.corba_cdr.inport_ior") continue;
            NamedValueExtPtr param(new NamedValueExt(propPro.name, propPro.value));
            propList.push_back(param);
        }

        if (sourcePort && targetPort) {
            RTSConnectionExt* conn = rts->addRTSConnection(id, name, sourcePort, targetPort, propList, dataConProf.pos);
            conn->dataProfile = dataConProf;
        }
    }
    for (int idxService = 0; idxService < profile.serviceConnList.size(); idxService++) {
        ServicePortConnector serviceConProf = profile.serviceConnList[idxService];
        RTSPortExt* sourcePort = getTargetPort(serviceConProf.source.pathId, serviceConProf.source.portName, rts);
        RTSPortExt* targetPort = getTargetPort(serviceConProf.target.pathId, serviceConProf.target.portName, rts);

        string id = "";
        string name = serviceConProf.name;

        vector<NamedValueExtPtr> propList;
        for (int idxProp = 0; idxProp < serviceConProf.propertyList.size(); idxProp++) {
            Property propPro = serviceConProf.propertyList[idxProp];
            NamedValueExtPtr param(new NamedValueExt(propPro.name, propPro.value));
            propList.push_back(param);
        }

        if (sourcePort && targetPort) {
            RTSConnectionExt* conn = rts->addRTSConnection(id, name, sourcePort, targetPort, propList, serviceConProf.pos);
            conn->serviceProfile = serviceConProf;
        }
    }

    return true;
}

RTSPortExt* ProfileHandlerExt::getTargetPort(std::string& sourceRtc, std::string& sourcePort, RTSystemExtItem* rts)
{
    RTSPortExt* result = 0;

    QString sourcePath = QString::fromStdString(sourceRtc);
    QStringList sourcePathList = sourcePath.split("/");
    sourcePathList.removeAt(0);
    QString sourceId = "/" + sourcePathList.join("/");
    RTSCompExt* rtc = rts->nameToRTSComp(sourceId.toStdString());
    if (rtc) {
        result = rtc->nameToRTSPort(sourcePort);
    }
    return result;
}

bool ProfileHandlerExt::parseProfile(std::string targetFile, RtsProfile& profile)
{
    xml_document doc;
    xml_parse_result result = doc.load_file(targetFile.c_str());
    if (result == 0) return false;

    xml_node rtsProfile = doc.child("rts:RtsProfile");
    if (rtsProfile == 0) return false;
    profile.id = rtsProfile.attribute("rts:id").as_string();

    for (xml_node comp = rtsProfile.child("rts:Components"); comp; comp = comp.next_sibling("rts:Components")) {
        Component proComp;
        proComp.instanceName = comp.attribute("rts:instanceName").as_string();
        proComp.pathUri = comp.attribute("rts:pathUri").as_string();
        //Position
        xml_node pos = comp.child("rtsExt:Location");
        proComp.posX = pos.attribute("rtsExt:x").as_int();
        proComp.posY = pos.attribute("rtsExt:y").as_int();

        for (pugi::xml_node prop = comp.child("rtsExt:Properties"); prop; prop = prop.next_sibling("rtsExt:Properties")) {
            if (prop.attribute("rtsExt:name").as_string() == "OpenRTM_NS") {
                proComp.isRegisteredInRtmDefaultNameServer = prop.attribute("rtsExt:value").as_bool();
                break;
            }
        }

        proComp.activeConfigurationSet = comp.attribute("rts:activeConfigurationSet").as_string();
        //ConfigurationSet
        parseConfigurationSet(comp, proComp);
        //DataPort
        for (xml_node dataPort = comp.child("rts:DataPorts"); dataPort; dataPort = dataPort.next_sibling("rts:DataPorts")) {
            DataPort proPort;
            proPort.name = dataPort.attribute("rts:name").as_string();
            for (xml_node prop = dataPort.child("rtsExt:Properties"); prop; prop = prop.next_sibling("rtsExt:Properties")) {
                string propName = prop.attribute("rtsExt:name").as_string();
                if (propName == "port.port_type") {
                    proPort.direction = prop.attribute("rtsExt:value").as_string();
                }
            }
            proComp.dataPortList.push_back(proPort);
        }
        //ServicePort
        for (xml_node servicePort = comp.child("rts:ServicePorts"); servicePort; servicePort = servicePort.next_sibling("rts:ServicePorts")) {
            ServicePort proPort;
            proPort.name = servicePort.attribute("rts:name").as_string();
            proComp.servicePortList.push_back(proPort);
        }
        profile.compList.push_back(proComp);
    }
    //DataPortConnector
    for (xml_node dataConn = rtsProfile.child("rts:DataPortConnectors"); dataConn; dataConn = dataConn.next_sibling("rts:DataPortConnectors")) {
        DataPortConnector dataConProf;
        dataConProf.name = dataConn.attribute("rts:name").as_string();
        dataConProf.source = parseTargetPort(dataConn.child("rts:sourceDataPort"));
        dataConProf.target = parseTargetPort(dataConn.child("rts:targetDataPort"));;

        for (pugi::xml_node prop = dataConn.child("rtsExt:Properties"); prop; prop = prop.next_sibling("rtsExt:Properties")) {
            Property propPro;
            propPro.name = prop.attribute("rtsExt:name").as_string();
            propPro.value = prop.attribute("rtsExt:value").as_string();
            dataConProf.propertyList.push_back(propPro);
            if (propPro.name == "POSITION") {
                float posX[6], posY[6];
                sscanf(propPro.value.c_str(),
                  "{1:(%f,%f),2:(%f,%f),3:(%f,%f),4:(%f,%f),5:(%f,%f),6:(%f,%f)}",
                  &posX[0], &posY[0], &posX[1], &posY[1], &posX[2], &posY[2], &posX[3], &posY[3], &posX[4], &posY[4], &posX[5], &posY[5]);
                DDEBUG_V("Value: (%f, %f), (%f, %f), (%f, %f), (%f, %f), (%f, %f), (%f, %f) ",
                  posX[0], posY[0], posX[1], posY[1], posX[2], posY[2], posX[3], posY[3], posX[4], posY[4], posX[5], posY[5]);
                for (int idxPos = 0; idxPos < 6; idxPos++) {
                    dataConProf.pos[idxPos] << posX[idxPos], posY[idxPos];
                }
            }
        }
        profile.dataConnList.push_back(dataConProf);
    }
    //ServicePortConnector
    for (pugi::xml_node serviceConn = rtsProfile.child("rts:ServicePortConnectors"); serviceConn; serviceConn = serviceConn.next_sibling("rts:ServicePortConnectors")) {
        ServicePortConnector serviceConProf;
        serviceConProf.name = serviceConn.attribute("rts:name").as_string();
        serviceConProf.source = parseTargetPort(serviceConn.child("rts:sourceServicePort"));
        serviceConProf.target = parseTargetPort(serviceConn.child("rts:targetServicePort"));;

        for (pugi::xml_node prop = serviceConn.child("rtsExt:Properties"); prop; prop = prop.next_sibling("rtsExt:Properties")) {
            Property propPro;
            propPro.name = prop.attribute("rtsExt:name").as_string();
            propPro.value = prop.attribute("rtsExt:value").as_string();
            serviceConProf.propertyList.push_back(propPro);
            if (propPro.name == "POSITION") {
                float posX[6], posY[6];
                sscanf(propPro.value.c_str(),
                  "{1:(%f,%f),2:(%f,%f),3:(%f,%f),4:(%f,%f),5:(%f,%f),6:(%f,%f)}",
                  &posX[0], &posY[0], &posX[1], &posY[1], &posX[2], &posY[2], &posX[3], &posY[3], &posX[4], &posY[4], &posX[5], &posY[5]);
                DDEBUG_V("Value: (%f, %f), (%f, %f), (%f, %f), (%f, %f), (%f, %f), (%f, %f) ",
                  posX[0], posY[0], posX[1], posY[1], posX[2], posY[2], posX[3], posY[3], posX[4], posY[4], posX[5], posY[5]);
                for (int idxPos = 0; idxPos < 6; idxPos++) {
                    serviceConProf.pos[idxPos] << posX[idxPos], posY[idxPos];
                }
            }
        }
        profile.serviceConnList.push_back(serviceConProf);
    }

    return true;
}

void ProfileHandlerExt::parseConfigurationSet(xml_node& comp, Component& proComp)
{
    for (xml_node configSet = comp.child("rts:ConfigurationSets"); configSet; configSet = configSet.next_sibling("rts:ConfigurationSets")) {
        ConfigurationSet configSetProf;
        configSetProf.id = configSet.attribute("rts:id").as_string();
        for (xml_node config = configSet.child("rts:ConfigurationData"); config; config = configSet.next_sibling("rts:ConfigurationData")) {
            Property configProf;
            configProf.name = config.attribute("rts:name").as_string();
            configProf.value = config.attribute("rts:data").as_string();
            configSetProf.dataList.push_back(configProf);
        }
        proComp.configList.push_back(configSetProf);
    }
}

TargetPort ProfileHandlerExt::parseTargetPort(const pugi::xml_node& targetPort)
{
    TargetPort result;
    result.portName = targetPort.attribute("rts:portName").as_string();
    result.instanceName = targetPort.attribute("rts:instanceName").as_string();
    for (xml_node prop = targetPort.child("rtsExt:Properties"); prop; prop = prop.next_sibling("rtsExt:Properties")) {
        string propName = prop.attribute("rtsExt:name").as_string();
        if (propName == "COMPONENT_PATH_ID") {
            result.pathId = prop.attribute("rtsExt:value").as_string();
            break;
        }
    }
    return result;
}
//////////
void ProfileHandlerExt::saveRtsProfile
(const string& targetFile, string& systemId, map<string, RTSCompExtPtr>& comps, RTSConnectionExtMap& connections, std::ostream& os)
{
    RtsProfile profile;
    profile.id = systemId;

    int offsetX = 0;
    int offsetY = 0;
    for (map<string, RTSCompExtPtr>::iterator it = comps.begin(); it != comps.end(); it++) {
        RTSCompExt* comp = it->second.get();
        if (comp->pos().x() < offsetX) offsetX = comp->pos().x();
        if (comp->pos().y() < offsetY) offsetY = comp->pos().y();
    }
    for (RTSConnectionExtMap::iterator it = connections.begin(); it != connections.end(); it++) {
        RTSConnectionExt* connect = it->second.get();
        for (int idxPos = 0; idxPos < 6; idxPos++) {
            if (connect->position[idxPos](0) < offsetX) offsetX = connect->position[idxPos](0);
            if (connect->position[idxPos](1) < offsetY) offsetY = connect->position[idxPos](1);
        }
    }
    if (offsetX < 0) offsetX = -offsetX;
    if (offsetY < 0) offsetY = -offsetY;

    for (map<string, RTSCompExtPtr>::iterator it = comps.begin(); it != comps.end(); it++) {
        RTSCompExt* comp = it->second.get();

        if (!isObjectAlive(comp->rtc_)) {
            os << "\033[31mWarning: " << comp->name << "is NOT ALIVE.\033[0m" << endl;
            profile.compList.push_back(comp->profile);
            continue;
        }

        try {
            Component compProf;
            compProf.id = "RTC:" + comp->vendor_ + ":" + comp->category_ + ":" + comp->name + ":" + comp->version_;
            compProf.instanceName = comp->name;
            compProf.pathUri = comp->hostAddress + comp->fullPath;
            compProf.activeConfigurationSet = comp->rtc_->get_configuration()->get_active_configuration_set()->id;
            compProf.posX = comp->pos().x() + offsetX;
            compProf.posY = comp->pos().y() + offsetY;

            for (vector<RTSPortExtPtr>::iterator p0 = comp->inPorts.begin(); p0 != comp->inPorts.end(); p0++) {
                RTSPortExt* in = *p0;
                buildPortInfo(in, compProf, "DataInPort");
            }
            for (vector<RTSPortExtPtr>::iterator p0 = comp->outPorts.begin(); p0 != comp->outPorts.end(); p0++) {
                RTSPortExt* out = *p0;
                buildPortInfo(out, compProf, "DataOutPort");
            }
            //
            SDOPackage::Configuration_ptr config = comp->rtc_->get_configuration();
            SDOPackage::ConfigurationSetList_var confSet = config->get_configuration_sets();
            for (int index = 0; index < confSet->length(); index++) {
                SDOPackage::ConfigurationSet conf = confSet[index];
                ConfigurationSet configSetProf;
                configSetProf.id = conf.id;
                copyNVListToProperty(conf.configuration_data, configSetProf.dataList);
                compProf.configList.push_back(configSetProf);
            }
            //
            RTC::ExecutionContextList_var eclist = comp->ownedExeContList_;
            for (int index = 0; index < eclist->length(); ++index) {
                if (isObjectAlive(eclist[index])) {
                    RTC::ExecutionContextService_var ec = RTC::ExecutionContextService::_narrow(eclist[index]);
                    ExecutionContext ecProf;
                    ecProf.id = to_string(index);
                    RTC::ExecutionKind ecKind = ec->get_profile()->kind;
                    switch (ecKind) {
                        case PERIODIC:
                            ecProf.kind = "PERIODIC";
                            break;
                        case EVENT_DRIVEN:
                            ecProf.kind = "EVENT_DRIVEN";
                            break;
                        case OTHER:
                            ecProf.kind = "OTHER";
                            break;
                    }
                    ecProf.rate = ec->get_profile()->rate;
                    copyNVListToProperty(ec->get_profile()->properties, ecProf.propertyList);
                    compProf.ecList.push_back(ecProf);
                }
            }
            //
            compProf.isRegisteredInRtmDefaultNameServer = NameServerManager::instance()->isRtmDefaultNameServer(comp->hostAddress, comp->portNo);
            profile.compList.push_back(compProf);
        } catch (...) {
            os << "\033[31mWarning: " << "Failed to acquire [" << comp->name << "] information.\033[0m" << endl;
        }
    }
    //
    for (RTSConnectionExtMap::iterator it = connections.begin(); it != connections.end(); it++) {
        RTSConnectionExt* connect = it->second.get();

        if (!isObjectAlive(connect->sourcePort->port)) {
            os << "\033[31mWarning: " << "The connection between " << connect->sourcePort->name << " and " << connect->targetPort->name << " is NOT ALIVE.\033[0m" << endl;
            if (connect->sourcePort->isServicePort) {
                profile.serviceConnList.push_back(connect->serviceProfile);
            } else {
                profile.dataConnList.push_back(connect->dataProfile);
            }
            continue;
        }

        try {
            if (connect->sourcePort->isServicePort) {
                ServicePortConnector conProf;
                ConnectorProfileList_var connectorProfiles = connect->sourcePort->port->get_connector_profiles();
                if (0 < connectorProfiles->length()) {
                    conProf.connectorId = connect->id;
                    conProf.name = connect->name;

                    ConnectorProfile& connectorProfile = connectorProfiles[0];
                    copyNVListToProperty(connectorProfile.properties, conProf.propertyList);
                    conProf.source = buildTargetPortInfo(connect->sourcePort);
                    conProf.target = buildTargetPortInfo(connect->targetPort);

                    buildPosition(connect, offsetX, offsetY, conProf.propertyList);

                    profile.serviceConnList.push_back(conProf);
                }

            } else {
                DataPortConnector conProf;
                ConnectorProfileList_var connectorProfiles = connect->sourcePort->port->get_connector_profiles();
                if (0 < connectorProfiles->length()) {
                    conProf.connectorId = connect->id;
                    conProf.name = connect->name;

                    ConnectorProfile& connectorProfile = connectorProfiles[0];
                    conProf.dataType = NVUtil::toString(connectorProfile.properties, "dataport.data_type");
                    conProf.interfaceType = NVUtil::toString(connectorProfile.properties, "dataport.interface_type");
                    conProf.dataflowType = NVUtil::toString(connectorProfile.properties, "dataport.dataflow_type");
                    conProf.subscriptionType = NVUtil::toString(connectorProfile.properties, "dataport.subscription_type");
                    string pushInterval = NVUtil::toString(connectorProfile.properties, "dataport.push_interval");
                    if (0 < pushInterval.length()) {
                        conProf.pushInterval = std::stof(pushInterval);
                    }

                    copyNVListToProperty(connectorProfile.properties, conProf.propertyList);
                    conProf.source = buildTargetPortInfo(connect->sourcePort);
                    conProf.target = buildTargetPortInfo(connect->targetPort);

                    buildPosition(connect, offsetX, offsetY, conProf.propertyList);

                    profile.dataConnList.push_back(conProf);
                }
            }

        } catch (...) {
            os << "\033[31mWarning: " << "Failed to acquire connection information between " << connect->sourcePort->name << " and " << connect->targetPort->name << ".\033[0m" << endl;
        }
    }
    writeProfile(targetFile, profile, os);
}

void ProfileHandlerExt::buildPosition(const RTSConnectionExt* connect, int offsetX, int offsetY, std::vector<Property>& propList)
{
    QString position = "{";
    for (int idxPos = 0; idxPos < 6; idxPos++) {
        if (0 < idxPos) position.append(",");
        position.append(QString::number(idxPos + 1)).append(":(");
        position.append(QString::number(connect->position[idxPos](0) + offsetX)).append(",");
        position.append(QString::number(connect->position[idxPos](1) + offsetY)).append(")");
    }
    position.append("}");

    string positionName = "POSITION";
    string value = position.toStdString();
    appendStringValue(propList, positionName, value);

    //QString posX1 = QString::number( (connect->position[1](0) + connect->position[2](0))/2 + offsetX );
    //QString posY1 = QString::number( (connect->position[1](1) + connect->position[2](1))/2 + offsetY );
    //QString posX2 = QString::number( (connect->position[2](0) + connect->position[3](0))/2 + offsetX );
    //QString posY2 = QString::number( (connect->position[2](1) + connect->position[3](1))/2 + offsetY );
    //QString posX3 = QString::number( (connect->position[3](0) + connect->position[4](0))/2 + offsetX );
    //QString posY3 = QString::number( (connect->position[3](1) + connect->position[4](1))/2 + offsetY );

    //string bendPointName = "BEND_POINT";
    //string bendPoint = "{1:(" + posX1.toStdString() + "," + posY1.toStdString() + "),"
    //                  + "2:(" + posX2.toStdString() + "," + posY2.toStdString() + "),"
    //                  + "3:(" + posX3.toStdString() + "," + posY3.toStdString() + ")}";
    //appendStringValue(propList, bendPointName, bendPoint);
}

TargetPort ProfileHandlerExt::buildTargetPortInfo(RTSPortExt* sourcePort)
{
    TargetPort result;

    result.portName = sourcePort->name;
    result.componentId = "RTC:" + sourcePort->rtsComp->vendor_ + ":" + sourcePort->rtsComp->category_ + ":" + sourcePort->rtsComp->name + ":" + sourcePort->rtsComp->version_;
    result.instanceName = sourcePort->rtsComp->name;

    Property prop;
    prop.name = "COMPONENT_PATH_ID";
    prop.value = sourcePort->rtsComp->hostAddress + sourcePort->rtsComp->fullPath;
    result.propertyList.push_back(prop);

    return result;
}

void ProfileHandlerExt::buildPortInfo(RTSPortExt* port, Component& compProf, std::string direction)
{
    PortProfile* portRaw = port->port->get_port_profile();
    if (port->isServicePort) {
        ServicePort portProf;
        portProf.name = port->name;

        copyNVListToProperty(portRaw->properties, portProf.propertyList);
        compProf.servicePortList.push_back(portProf);

    } else {
        DataPort portProf;
        portProf.name = port->name;
        portProf.direction = direction;

        copyNVListToProperty(portRaw->properties, portProf.propertyList);
        compProf.dataPortList.push_back(portProf);
    }
}

void ProfileHandlerExt::copyNVListToProperty(NVList& source, vector<Property>& target)
{
    for (CORBA::ULong i(0), len(source.length()); i < len; ++i) {
        const char* value;
        if (source[i].value >>= value) {
            const char* name(source[i].name);
            Property prop;
            prop.name = name;
            prop.value = value;
            target.push_back(prop);
        };
    }
}

void ProfileHandlerExt::appendStringValue(std::vector<Property>& target, std::string& name, std::string& value)
{
    bool isExist = false;
    for (Property prop : target) {
        if (prop.name == name) {
            prop.value = value;
            isExist = true;
            break;
        }
    }
    if (isExist == false) {
        Property newProp;
        newProp.name = name;
        newProp.value = value;
        target.push_back(newProp);
    }
}

void ProfileHandlerExt::removePropertyByValue(std::vector<Property>& target, const std::string& name)
{
    target.erase(remove_if(target.begin(), target.end(), PropertyValueComparator(name)), target.end());
}

bool ProfileHandlerExt::writeProfile(const std::string& targetFile, RtsProfile& profile, std::ostream& os)
{
    xml_document doc;
    xml_node profileNode = doc.append_child("rts:RtsProfile");

    profileNode.append_attribute("rts:version") = "0.2";
    profileNode.append_attribute("rts:id") = profile.id.c_str();
    profileNode.append_attribute("xmlns:rts") = "http://www.openrtp.org/namespaces/rts";
    profileNode.append_attribute("xmlns:rtsExt") = "http://www.openrtp.org/namespaces/rts_ext";
    profileNode.append_attribute("xmlns:xsi") = "http://www.w3.org/2001/XMLSchema-instance";

    writeComponent(profile.compList, profileNode);
    writeDataConnector(profile.dataConnList, profileNode);
    writeServiceConnector(profile.serviceConnList, profileNode);

    bool ret = false;
    try {
        ret = doc.save_file(targetFile.c_str());
    } catch (...) {
        os << "\033[31mWarning: " << "Failed to save [" << targetFile << "].\033[0m" << endl;
    }
    return ret;
}

void ProfileHandlerExt::writeComponent(std::vector<Component>& compList, xml_node& parent)
{
    for (int idxComp = 0; idxComp < compList.size(); idxComp++) {
        Component target = compList[idxComp];
        xml_node compNode = parent.append_child("rts:Components");
        compNode.append_attribute("xsi:type") = "rtsExt:component_ext";
        compNode.append_attribute("rts:isRequired") = "false";
        compNode.append_attribute("rts:compositeType") = "None";
        compNode.append_attribute("rts:activeConfigurationSet") = target.activeConfigurationSet.c_str();
        compNode.append_attribute("rts:instanceName") = target.instanceName.c_str();
        compNode.append_attribute("rts:pathUri") = target.pathUri.c_str();
        compNode.append_attribute("rts:id") = target.id.c_str();

        xml_node propertyNode = compNode.append_child("rtsExt:Properties");
        propertyNode.append_attribute("rtsExt:value") = target.isRegisteredInRtmDefaultNameServer;
        propertyNode.append_attribute("rtsExt:name") = "OpenRTM_NS";

        //DataPort
        writeDataPort(target.dataPortList, compNode);
        //ServicePort
        writeServicePort(target.servicePortList, compNode);
        //ConfigurationSet
        writeConfigurationSet(target.configList, compNode);
        //ExecutionContext
        writeExecutionContext(target.ecList, compNode);
        //Location
        writeLocation(target, compNode);
    }
}

void ProfileHandlerExt::writeDataPort(std::vector<DataPort>& portList, xml_node& parent)
{
    for (int idxData = 0; idxData < portList.size(); idxData++) {
        DataPort dataport = portList[idxData];
        xml_node dataPortNode = parent.append_child("rts:DataPorts");
        dataPortNode.append_attribute("xsi:type") = "rtsExt:dataport_ext";
        dataPortNode.append_attribute("rts:name") = dataport.name.c_str();

        writeProperty(dataport.propertyList, dataPortNode);
    }
}

void ProfileHandlerExt::writeServicePort(std::vector<ServicePort>& portList, xml_node& parent)
{
    for (int idxService = 0; idxService < portList.size(); idxService++) {
        ServicePort serviceport = portList[idxService];
        xml_node servicePortNode = parent.append_child("rts:ServicePorts");
        servicePortNode.append_attribute("xsi:type") = "rtsExt:serviceport_ext";
        servicePortNode.append_attribute("rts:name") = serviceport.name.c_str();

        writeProperty(serviceport.propertyList, servicePortNode);
    }

}

void ProfileHandlerExt::writeConfigurationSet(std::vector<ConfigurationSet>& configList, xml_node& parent)
{
    for (int idxConfig = 0; idxConfig < configList.size(); idxConfig++) {
        ConfigurationSet config = configList[idxConfig];
        xml_node configNode = parent.append_child("rts:ConfigurationSets");
        configNode.append_attribute("rts:id") = config.id.c_str();
        for (int idxData = 0; idxData < config.dataList.size(); idxData++) {
            Property data = config.dataList[idxData];
            xml_node dataNode = configNode.append_child("rts:ConfigurationData");
            dataNode.append_attribute("rts:data") = data.value.c_str();
            dataNode.append_attribute("rts:name") = data.name.c_str();
        }
    }
}

void ProfileHandlerExt::writeExecutionContext(std::vector<ExecutionContext>& ecList, xml_node& parent)
{
    for (int idxEC = 0; idxEC < ecList.size(); idxEC++) {
        ExecutionContext ec = ecList[idxEC];
        xml_node ecNode = parent.append_child("rts:ExecutionContexts");
        ecNode.append_attribute("xsi:type") = "rtsExt:execution_context_ext";
        ecNode.append_attribute("rts:id") = ec.id.c_str();
        ecNode.append_attribute("rts:rate") = ec.rate;
        ecNode.append_attribute("rts:kind") = ec.kind.c_str();

        writeProperty(ec.propertyList, ecNode);
    }
}

void ProfileHandlerExt::writeLocation(Component& target, xml_node& parent)
{
    xml_node posNode = parent.append_child("rtsExt:Location");
    posNode.append_attribute("rtsExt:direction") = "RIGHT";
    posNode.append_attribute("rtsExt:width") = "-1";
    posNode.append_attribute("rtsExt:height") = "-1";
    posNode.append_attribute("rtsExt:x") = target.posX;
    posNode.append_attribute("rtsExt:y") = target.posY;
}

void ProfileHandlerExt::writeDataConnector(std::vector<DataPortConnector>& connList, xml_node& parent)
{
    for (int idxConn = 0; idxConn < connList.size(); idxConn++) {
        DataPortConnector conn = connList[idxConn];
        xml_node dataconn = parent.append_child("rts:DataPortConnectors");
        dataconn.append_attribute("xsi:type") = "rtsExt:dataport_connector_ext";
        dataconn.append_attribute("rts:connectorId") = conn.connectorId.c_str();
        dataconn.append_attribute("rts:name") = conn.name.c_str();
        if (0 < conn.dataType.length()) {
            dataconn.append_attribute("rts:dataType") = conn.dataType.c_str();
        }
        dataconn.append_attribute("rts:interfaceType") = conn.interfaceType.c_str();
        dataconn.append_attribute("rts:dataflowType") = conn.dataflowType.c_str();
        dataconn.append_attribute("rts:subscriptionType") = conn.subscriptionType.c_str();
        if (0.0 < conn.pushInterval) {
            dataconn.append_attribute("rts:pushInterval") = conn.pushInterval;
        }
        writeTargetPort(conn.source, "rts:sourceDataPort", dataconn);
        writeTargetPort(conn.target, "rts:targetDataPort", dataconn);

        writeProperty(conn.propertyList, dataconn);
    }
}

void ProfileHandlerExt::writeServiceConnector(std::vector<ServicePortConnector>& connList, xml_node& parent)
{
    for (int idxConn = 0; idxConn < connList.size(); idxConn++) {
        ServicePortConnector conn = connList[idxConn];
        xml_node srvconn = parent.append_child("rts:ServicePortConnectors");
        srvconn.append_attribute("xsi:type") = "rtsExt:serviceport_connector_ext";
        srvconn.append_attribute("rts:connectorId") = conn.connectorId.c_str();
        srvconn.append_attribute("rts:name") = conn.name.c_str();

        writeTargetPort(conn.source, "rts:sourceServicePort", srvconn);
        writeTargetPort(conn.target, "rts:targetServicePort", srvconn);
    }
}

void ProfileHandlerExt::writeTargetPort(TargetPort& target, std::string tag, xml_node& parent)
{
    xml_node targetPortNode = parent.append_child(tag.c_str());
    targetPortNode.append_attribute("xsi:type") = "rtsExt:target_port_ext";
    targetPortNode.append_attribute("rts:portName") = target.portName.c_str();
    targetPortNode.append_attribute("rts:instanceName") = target.instanceName.c_str();
    targetPortNode.append_attribute("rts:componentId") = target.componentId.c_str();

    writeProperty(target.propertyList, targetPortNode);
}

void ProfileHandlerExt::writeProperty(std::vector<Property>& propList, xml_node& parent)
{
    for (int idxProp = 0; idxProp < propList.size(); idxProp++) {
        Property prop = propList[idxProp];
        if (QString::fromStdString(prop.value).startsWith("IOR:")) continue;

        xml_node propertyNode = parent.append_child("rtsExt:Properties");
        propertyNode.append_attribute("rtsExt:value") = prop.value.c_str();
        propertyNode.append_attribute("rtsExt:name") = prop.name.c_str();
    }

}

}
