/**
   \file
   \author shizuko hattori
*/

#include"VirtualRobotRTC.h"
#include "BodyRTCItem.h"
#include <cnoid/Config>
#include <rtm/NVUtil.h>
#include <cnoid/MessageView>
#include <fmt/format.h>
#include <iostream>
#include "../gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {
const bool CONTROLLER_BRIDGE_DEBUG = false;
}


void VirtualRobotRTC::registerFactory(RTC::Manager* manager, const char* componentTypeName)
{
    static const char* spec[] = {
        "implementation_id", "VirtualRobot",
        "type_name",         "VirtualRobot",
        "description",       "This component enables controller components to"
        "access the I/O of a virtual robot in a Choreonoid simulation",
        "version",           CNOID_VERSION_STRING,
        "vendor",            "AIST",
        "category",          "Choreonoid",
        "activity_type",     "DataFlowComponent",
        "max_instance",      "100",
        "language",          "C++",
        "lang_type",         "compile",
        ""
    };

    if(CONTROLLER_BRIDGE_DEBUG){
        cout << "initVirtualRobotRTC()" << endl;
    }

    RTC::Properties profile(spec);
    profile.setDefault("type_name", componentTypeName);

    manager->registerFactory(profile,
                             RTC::Create<VirtualRobotRTC>,
                             RTC::Delete<VirtualRobotRTC>);
}


VirtualRobotRTC::VirtualRobotRTC(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager)
{
    if(CONTROLLER_BRIDGE_DEBUG){
        cout << "VirtualRobotRTC::VirtualRobotRTC" << endl;
    }
}

RTC::ReturnCode_t VirtualRobotRTC::onInitialize()
{
    return RTC::RTC_OK;
}


VirtualRobotRTC::~VirtualRobotRTC()
{
    if(CONTROLLER_BRIDGE_DEBUG){
        cout << "VirtualRobotRTC::~VirtualRobotRTC" << endl;
    }
}

void VirtualRobotRTC::createPorts( BridgeConf* bridgeConf)
{
    if(bridgeConf){
        PortInfoMap::iterator it;

        for(it = bridgeConf->outPortInfos.begin(); it != bridgeConf->outPortInfos.end(); ++it){
            if (!createOutPortHandler(it->second)){
                cerr << "createOutPortHandler(" << it->second.portName << ") failed" << std::endl;
            }
        }

        for(it = bridgeConf->inPortInfos.begin(); it != bridgeConf->inPortInfos.end(); ++it){
            if (!createInPortHandler(it->second)){
                cerr << "createInPortHandler(" << it->second.portName << ") failed" << std::endl;
            }
        }

        updatePortObjectRefs();
    }
}

bool VirtualRobotRTC::createOutPortHandler(PortInfo& portInfo)
{
    DataTypeId dataTypeId = portInfo.dataTypeId;
    bool ret = false;

    if(portInfo.dataOwnerNames.empty()){
        switch(dataTypeId) {

        case JOINT_VALUE:
        case JOINT_VELOCITY:
        case JOINT_ACCELERATION:
        case JOINT_TORQUE:
        case FORCE_SENSOR:
        case RATE_GYRO_SENSOR:
        case ACCELERATION_SENSOR:
            ret = registerOutPortHandler(new SensorStateOutPortHandler(portInfo));
            break;
        case CAMERA_RANGE:
            ret = registerOutPortHandler(new CameraRangeOutPortHandler(portInfo, false));
            break;
        case CAMERA_IMAGE:
            ret = registerOutPortHandler(new CameraImageOutPortHandler(portInfo, false));
            break;
        case RANGE_SENSOR:
            ret = registerOutPortHandler(new RangeSensorOutPortHandler(portInfo, false));
            break;
        default:
            break;
        }
    } else {

        switch(dataTypeId) {

        case JOINT_VALUE:
        case JOINT_VELOCITY:
        case JOINT_ACCELERATION:
        case JOINT_TORQUE:
        case ABS_TRANSFORM:
        case ABS_VELOCITY:
        case ABS_ACCELERATION:
        case CONSTRAINT_FORCE:
        case EXTERNAL_FORCE:
            ret = registerOutPortHandler(new LinkDataOutPortHandler(portInfo));
            break;
        case ABS_TRANSFORM2:
            ret = registerOutPortHandler(new AbsTransformOutPortHandler(portInfo));
            break;

        case FORCE_SENSOR:
        case RATE_GYRO_SENSOR:
        case ACCELERATION_SENSOR:
            ret = registerOutPortHandler(new SensorDataOutPortHandler(portInfo));
            break;
        case RATE_GYRO_SENSOR2:
            ret = registerOutPortHandler(new GyroSensorOutPortHandler(portInfo));
            break;
        case ACCELERATION_SENSOR2:
            ret = registerOutPortHandler(new AccelerationSensorOutPortHandler(portInfo));
            break;
        case LIGHT:
            ret = registerOutPortHandler(new LightOnOutPortHandler(portInfo));
            break;
        case CAMERA_RANGE:
            ret = registerOutPortHandler(new CameraRangeOutPortHandler(portInfo, false));
            break;
        case CAMERA_IMAGE:
            ret = registerOutPortHandler(new CameraImageOutPortHandler(portInfo, false));
            break;
        case RANGE_SENSOR:
            ret = registerOutPortHandler(new RangeSensorOutPortHandler(portInfo, false));
            break;
        default:
            break;
        }
    }
    return ret;
}


bool VirtualRobotRTC::createInPortHandler(PortInfo& portInfo)
{
    DataTypeId dataTypeId = portInfo.dataTypeId;
    bool ret = false;
    if(portInfo.dataOwnerNames.empty()){
        switch(dataTypeId) {
        case JOINT_VALUE:
        case JOINT_VELOCITY:
        case JOINT_ACCELERATION:
        case JOINT_TORQUE:
            ret = registerInPortHandler(new JointDataSeqInPortHandler(portInfo));
            break;
        default:
            break;
        }
    }else{
        switch(dataTypeId){
        case JOINT_VALUE:
        case JOINT_VELOCITY:
        case JOINT_ACCELERATION:
        case JOINT_TORQUE:
            ret = registerInPortHandler(new LinkDataInPortHandler(portInfo));
            break;
        case ABS_TRANSFORM2:
            ret = registerInPortHandler(new AbsTransformInPortHandler(portInfo));
            break;
        case ABS_TRANSFORM:
        case ABS_VELOCITY:
        case ABS_ACCELERATION:
        case EXTERNAL_FORCE:
            ret = registerInPortHandler(new LinkDataInPortHandler(portInfo));
            break;
        case LIGHT:
            ret = registerInPortHandler(new LightOnInPortHandler(portInfo));
            break;
        default:
            break;
        }
    }
    return ret;
}

PortHandlerPtr VirtualRobotRTC::getOutPortHandler(const std::string& name_)
{
    string name(name_);
    string::size_type index = name.rfind(".");
    if (index != string::npos) name = name.substr(index+1);

    PortHandlerPtr portHandler;

    OutPortHandlerMap::iterator p = outPortHandlers.find(name);
    if(p != outPortHandlers.end()){
        portHandler = p->second;
    }

    return portHandler;
}

PortHandlerPtr VirtualRobotRTC::getInPortHandler(const std::string& name_)
{
    string name(name_);
    string::size_type index = name.rfind(".");
    if (index != string::npos) name = name.substr(index+1);

    PortHandlerPtr portHandler;

    InPortHandlerMap::iterator q = inPortHandlers.find(name);
    if(q != inPortHandlers.end()){
        portHandler = q->second;
    }

    return portHandler;
}

void VirtualRobotRTC::updatePortObjectRefs()
{
    for(OutPortHandlerMap::iterator it = outPortHandlers.begin(); it != outPortHandlers.end(); ++it){
        OutPortHandlerPtr& handler = it->second;
        handler->portRef = RTC::PortService::_nil();
    }
    for(InPortHandlerMap::iterator it = inPortHandlers.begin(); it != inPortHandlers.end(); ++it){
        InPortHandlerPtr& handler = it->second;
        handler->portRef = RTC::PortService::_nil();
    }

    RTC::PortServiceList_var ports = get_ports();

    for(CORBA::ULong i=0; i < ports->length(); ++i){

        RTC::PortProfile_var profile = ports[i]->get_port_profile();
        const char* type;
        if (!(NVUtil::find(profile->properties, "port.port_type") >>= type)) 
            break;
        PortHandlerPtr portHandler;
        if(!std::strcmp(type,"DataInPort"))
            portHandler = getInPortHandler(string(profile->name));
        else
            portHandler = getOutPortHandler(string(profile->name));

        if(portHandler){
            portHandler->portRef = ports[i];
        }
    }
}

RTC::RTCList* VirtualRobotRTC::getConnectedRtcs()
{
    RTC::RTCList* rtcList = new RTC::RTCList;

    set<string> foundRtcNames;

    for(OutPortHandlerMap::iterator it = outPortHandlers.begin(); it != outPortHandlers.end(); ++it){
        OutPortHandlerPtr& handler = it->second;
        addConnectedRtcs(handler->portRef, *rtcList, foundRtcNames);
    }
    for(InPortHandlerMap::iterator it = inPortHandlers.begin(); it != inPortHandlers.end(); ++it){
        InPortHandlerPtr& handler = it->second;
        addConnectedRtcs(handler->portRef, *rtcList, foundRtcNames);
    }

    return rtcList;
}

void VirtualRobotRTC::addConnectedRtcs(RTC::PortService_ptr portRef, RTC::RTCList& rtcList, std::set<std::string>& foundRtcNames)
{
    RTC::PortProfile_var portProfile = portRef->get_port_profile();
    string portName(portProfile->name);

    RTC::ConnectorProfileList_var connectorProfiles = portRef->get_connector_profiles();

    for(CORBA::ULong i=0; i < connectorProfiles->length(); ++i){
        RTC::ConnectorProfile& connectorProfile = connectorProfiles[i];
        RTC::PortServiceList& connectedPorts = connectorProfile.ports;

        for(CORBA::ULong j=0; j < connectedPorts.length(); ++j){
            RTC::PortService_ptr connectedPortRef = connectedPorts[j];
            RTC::PortProfile_var connectedPortProfile = connectedPortRef->get_port_profile();
            RTC::RTObject_var connectedRtcRef = RTC::RTObject::_duplicate(connectedPortProfile->owner);
            RTC::RTObject_var thisRef = RTC::RTObject::_duplicate(getObjRef());

            if(!CORBA::is_nil(connectedRtcRef) && !connectedRtcRef->_is_equivalent(thisRef)){
                CORBA::ULong ii=0;
                for(; ii<rtcList.length(); ii++){
                    if(rtcList[ii]->_is_equivalent(connectedRtcRef))
                        break;
                }

                if(ii == rtcList.length()){
                    RTC::ComponentProfile_var componentProfile = connectedRtcRef->get_component_profile();
                    string connectedRtcName(componentProfile->instance_name);
                    MessageView* mv = MessageView::instance();
                    mv->putln(
                        format(_("detected RTC: {0}  Port connection: {1} <--> {2}"),
                               connectedRtcName, portName, string(connectedPortProfile->name)));

                    RTC::ExecutionContextList_var execServices = connectedRtcRef->get_owned_contexts();

                    for(CORBA::ULong k=0; k < execServices->length(); k++) {
                        RTC::ExecutionContext_var execContext = execServices[k];

												RTC::ExecutionContextService_var extTrigExecContext =
													RTC::ExecutionContextService::_narrow(execContext);

                        if(!CORBA::is_nil(extTrigExecContext)){
                            CORBA::ULong n = rtcList.length();
                            rtcList.length(n + 1);
                            rtcList[n] = connectedRtcRef;
                            foundRtcNames.insert(connectedRtcName);
                            RTC::PortServiceList_var portList = connectedRtcRef->get_ports();
                            for(CORBA::ULong jj=0; jj < portList->length(); ++jj){
                                addConnectedRtcs(portList[jj], rtcList, foundRtcNames);
                            }
                        }
                    }
                }
            }
        }
    }
}

void VirtualRobotRTC::inputDataFromSimulator(BodyRTCItem* bodyRTC)
{
    const double controlTime = bodyRTC->controlTime();
    const double controlTimeStep = bodyRTC->timeStep();
    for(OutPortHandlerMap::iterator it = outPortHandlers.begin(); it != outPortHandlers.end(); ++it){
        double stepTime = it->second->stepTime;
        if(stepTime){
            double w = controlTime - (int)(controlTime / stepTime) * stepTime + controlTimeStep / 2;
            if(w >= stepTime){
                w = 0.0;
            }
            if(w < controlTimeStep ){
                it->second->inputDataFromSimulator(bodyRTC);
            }
        }else{
            it->second->inputDataFromSimulator(bodyRTC);
        }
    }
}


void VirtualRobotRTC::outputDataToSimulator(const BodyPtr& body)
{
    for(InPortHandlerMap::iterator it = inPortHandlers.begin(); it != inPortHandlers.end(); ++it){
        it->second->outputDataToSimulator(body);
    }
}


void VirtualRobotRTC::writeDataToOutPorts(double controlTime, double controlTimeStep)
{
    for(OutPortHandlerMap::iterator it = outPortHandlers.begin(); it != outPortHandlers.end(); ++it){
        if(!it->second->synchController)
            continue;
        double stepTime = it->second->stepTime;
        if(stepTime){
            double w = controlTime-(int)(controlTime/stepTime)*stepTime + controlTimeStep/2;
            if(w >= stepTime) w=0;
            if(w < controlTimeStep )
                it->second->writeDataToPort();
        }else{
            it->second->writeDataToPort();
        }
    }
}


void VirtualRobotRTC::readDataFromInPorts()
{
    for(InPortHandlerMap::iterator it = inPortHandlers.begin(); it != inPortHandlers.end(); ++it){
        it->second->readDataFromPort();
    }
}

void VirtualRobotRTC::stop()
{

}

bool VirtualRobotRTC::checkOutPortStepTime(double controlTimeStep)
{
    bool ret = true;
    for(OutPortHandlerMap::iterator it = outPortHandlers.begin(); it != outPortHandlers.end(); ++it){
        double stepTime = it->second->stepTime;
        if(stepTime && stepTime < controlTimeStep){
            cerr << "OutPort(" << it->second->portName << ") : Output interval(" << stepTime << ") must be longer than the control interval(" << controlTimeStep << ")." << std::endl;
            ret &= false;
        }
    }
    return ret;
}

void VirtualRobotRTC::initialize(Body* simulationBody)
{
    for(OutPortHandlerMap::iterator it = outPortHandlers.begin(); it != outPortHandlers.end(); ++it){
        CameraImageOutPortHandler* cameaImageOutPortHandler = dynamic_cast<CameraImageOutPortHandler*>(it->second.get());
        if(cameaImageOutPortHandler)
            cameaImageOutPortHandler->initialize(simulationBody);
        CameraRangeOutPortHandler* cameaRangeOutPortHandler = dynamic_cast<CameraRangeOutPortHandler*>(it->second.get());
        if(cameaRangeOutPortHandler)
            cameaRangeOutPortHandler->initialize(simulationBody);
        RangeSensorOutPortHandler* rangeSensorOutPortHandler = dynamic_cast<RangeSensorOutPortHandler*>(it->second.get());
        if(rangeSensorOutPortHandler)
            rangeSensorOutPortHandler->initialize(simulationBody);
    }
}
