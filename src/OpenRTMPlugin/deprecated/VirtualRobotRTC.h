/**
   \file
   \author shizuko hattori
*/

#ifndef CNOID_OPENRTM_PLUGIN_VIRTUAL_ROBOT_RTC_H
#define CNOID_OPENRTM_PLUGIN_VIRTUAL_ROBOT_RTC_H

#include "VirtualRobotPortHandler.h"
#include <cnoid/Body>
#include <rtm/RTObject.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <set>
#include <string>

namespace cnoid {

class VirtualRobotRTC : public RTC::DataFlowComponentBase
{
public:
    static void registerFactory(RTC::Manager* manager, const char* componentTypeName);

    VirtualRobotRTC(RTC::Manager* manager);
    ~VirtualRobotRTC();

    using RTC::DataFlowComponentBase::initialize;

    RTC::ReturnCode_t onInitialize();
    void createPorts(BridgeConf* bridgeConf);

    PortHandlerPtr getOutPortHandler(const std::string& name);
    PortHandlerPtr getInPortHandler(const std::string& name);

    RTC::RTCList* getConnectedRtcs();

    void inputDataFromSimulator(BodyRTCItem* bodyRTC);
    void outputDataToSimulator(const BodyPtr& body);

    void writeDataToOutPorts(double controlTime, double controlTimeStep);
    void readDataFromInPorts();
    void stop();
    bool checkOutPortStepTime(double controlTimeStep);
    void initialize(Body* simulationBody);
   
private:
    typedef std::map<std::string, OutPortHandlerPtr> OutPortHandlerMap;
    OutPortHandlerMap outPortHandlers;

    typedef std::map<std::string, InPortHandlerPtr> InPortHandlerMap;
    InPortHandlerMap inPortHandlers;


    bool createOutPortHandler(PortInfo& portInfo);
    bool createInPortHandler(PortInfo& portInfo);

    template <class TOutPortHandler>
    bool registerOutPortHandler(TOutPortHandler* handler) {
        const std::string& name = handler->portName;
        if(!getOutPortHandler(name)){
            if (!addOutPort(name.c_str(), handler->getOutPort())) return false;
            outPortHandlers.insert(std::make_pair(name, OutPortHandlerPtr(handler)));
        }
        return true;
    }

    template <class TOutPortHandler>
    bool unregisterOutPortHandler(TOutPortHandler* handler) {
        if (deleteOutPort(handler->outPort)) 
            return true;     
        return false;
    }

    template <class TInPortHandler>
    bool registerInPortHandler(TInPortHandler* handler) {
        const std::string& name = handler->portName;
        if(!getInPortHandler(name)){
            if (!addInPort(name.c_str(), handler->inPort)) return false;
            inPortHandlers.insert(std::make_pair(name, InPortHandlerPtr(handler)));
        }
        return true;
    }

    void updatePortObjectRefs();

    void addConnectedRtcs(RTC::PortService_ptr portRef, RTC::RTCList& rtcList, std::set<std::string>& foundRtcNames);
};
}

#endif
