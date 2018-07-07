/**
   \file
   \author shizuko hattori
*/

#ifndef CNOID_OPENRTM_PLUGIN_VIRTUAL_ROBOT_PORT_HANDLER_H
#define CNOID_OPENRTM_PLUGIN_VIRTUAL_ROBOT_PORT_HANDLER_H

#include "BridgeConf.h"
#include <cnoid/Body>
#include <cnoid/BasicSensors>
#include <cnoid/Camera>
#include <cnoid/RangeCamera>
#include <cnoid/RangeSensor>
#include <rtm/RTC.h>
#include <rtm/PortBase.h>
#include <rtm/OutPort.h>
#include <rtm/InPort.h>
#include <rtm/idl/InterfaceDataTypes.hh>
#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <mutex>

namespace cnoid {
    
class BodyRTCItem;

class PortHandler
{
public:
    PortHandler(PortInfo& info);
    virtual ~PortHandler();
    RTC::PortService_var portRef;
    std::string portName;
};
    
typedef std::shared_ptr<PortHandler> PortHandlerPtr;
    

class OutPortHandler : public PortHandler
{
public:
    OutPortHandler(PortInfo& info, bool synchContorller = true);
    virtual RTC::OutPortBase& getOutPort() = 0;
    virtual void inputDataFromSimulator(BodyRTCItem* bodyRTC) = 0;
    virtual void writeDataToPort() = 0;
    template<class T> void setTime(T& value, double _time){
        value.tm.sec = (unsigned long)_time;
        value.tm.nsec = (unsigned long)((_time-value.tm.sec)*1000000000.0 + 0.5);
        if( value.tm.nsec >= 1000000000 ){
            value.tm.sec++;
            value.tm.nsec -= 1000000000;
        }
    }
    double stepTime;
    bool synchController;
};
    
typedef std::shared_ptr<OutPortHandler> OutPortHandlerPtr;
    
    
class InPortHandler : public PortHandler
{
public:
    InPortHandler(PortInfo& info) : PortHandler(info){} 
    virtual void outputDataToSimulator(const BodyPtr& body) = 0;
    virtual void readDataFromPort() = 0;
};
    
typedef std::shared_ptr<InPortHandler> InPortHandlerPtr;
    
    
class SensorStateOutPortHandler : public OutPortHandler
{
public:
    SensorStateOutPortHandler(PortInfo& info);
    virtual RTC::OutPortBase& getOutPort() override;
    virtual void inputDataFromSimulator(BodyRTCItem* bodyRTC) override;
    virtual void writeDataToPort() override;
private:
    RTC::TimedDoubleSeq values;
public:
    RTC::OutPort<RTC::TimedDoubleSeq> outPort;
private:
    DataTypeId dataTypeId;
};
    
    
class LinkDataOutPortHandler : public OutPortHandler
{
public:
    LinkDataOutPortHandler(PortInfo& info);
    virtual RTC::OutPortBase& getOutPort() override;
    virtual void inputDataFromSimulator(BodyRTCItem* bodyRTC) override;
    virtual void writeDataToPort() override;
private:
    RTC::TimedDoubleSeq value;
public:
    RTC::OutPort<RTC::TimedDoubleSeq> outPort;
private:
    std::vector<std::string> linkNames;
    DataTypeId linkDataType;
};
    
class AbsTransformOutPortHandler : public OutPortHandler
{
public:
    AbsTransformOutPortHandler(PortInfo& info);
    virtual RTC::OutPortBase& getOutPort() override;
    virtual void inputDataFromSimulator(BodyRTCItem* bodyRTC) override;
    virtual void writeDataToPort() override;
private:
    RTC::TimedPose3D value;
public:
    RTC::OutPort<RTC::TimedPose3D> outPort;
private:
    std::vector<std::string> linkNames;
    DataTypeId linkDataType;
};
    
    
class SensorDataOutPortHandler : public OutPortHandler
{
public:
    SensorDataOutPortHandler(PortInfo& info);
    virtual RTC::OutPortBase& getOutPort() override;
    virtual void inputDataFromSimulator(BodyRTCItem* bodyRTC) override;
    virtual void writeDataToPort() override;
private:
    RTC::TimedDoubleSeq value;
public:
    RTC::OutPort<RTC::TimedDoubleSeq> outPort;
private:
    std::vector<std::string> sensorNames;
};
    
class GyroSensorOutPortHandler : public OutPortHandler
{
public:
    GyroSensorOutPortHandler(PortInfo& info);
    virtual RTC::OutPortBase& getOutPort() override;
    virtual void inputDataFromSimulator(BodyRTCItem* bodyRTC) override;
    virtual void writeDataToPort() override;
private:
    RTC::TimedAngularVelocity3D value;
public:
    RTC::OutPort<RTC::TimedAngularVelocity3D> outPort;
private:
    std::vector<std::string> sensorNames;
};
    
class AccelerationSensorOutPortHandler : public OutPortHandler
{
public:
    AccelerationSensorOutPortHandler(PortInfo& info);
    virtual RTC::OutPortBase& getOutPort() override;
    virtual void inputDataFromSimulator(BodyRTCItem* bodyRTC) override;
    virtual void writeDataToPort() override;
private:
    RTC::TimedAcceleration3D value;
public:
    RTC::OutPort<RTC::TimedAcceleration3D> outPort;
private:
    std::vector<std::string> sensorNames;
};
    
class LightOnOutPortHandler : public OutPortHandler
{
public:
    LightOnOutPortHandler(PortInfo& info);
    virtual RTC::OutPortBase& getOutPort() override;
    virtual void inputDataFromSimulator(BodyRTCItem* bodyRTC) override;
    virtual void writeDataToPort() override;
private:
    RTC::TimedBooleanSeq value;
public:
    RTC::OutPort<RTC::TimedBooleanSeq> outPort;
private:
    std::vector<std::string> lightNames;
};

class CameraImageOutPortHandlerImpl;
    
class CameraImageOutPortHandler : public OutPortHandler
{
public:
    CameraImageOutPortHandler(PortInfo& info, bool synchController);
    virtual RTC::OutPortBase& getOutPort() override;
    virtual void inputDataFromSimulator(BodyRTCItem* bodyRTC) override;
    virtual void writeDataToPort() override;
    void initialize(Body* simulationBody);

private:
    CameraImageOutPortHandlerImpl* impl;
};

class CameraRangeOutPortHandlerImpl;

class CameraRangeOutPortHandler : public OutPortHandler
{
public:
    CameraRangeOutPortHandler(PortInfo& info, bool synchController);
    ~CameraRangeOutPortHandler();
    virtual RTC::OutPortBase& getOutPort() override;
    virtual void inputDataFromSimulator(BodyRTCItem* bodyRTC) override;
    virtual void writeDataToPort() override;
    void initialize(Body* simulationBody);

private:
    CameraRangeOutPortHandlerImpl* impl;
};


class RangeSensorOutPortHandler : public OutPortHandler
{
public:
    RangeSensorOutPortHandler(PortInfo& info, bool synchController);
    virtual RTC::OutPortBase& getOutPort() override;
    virtual void inputDataFromSimulator(BodyRTCItem* bodyRTC) override;
    virtual void writeDataToPort() override;
    void onRangeSensorStateChanged();
    void initialize(Body* simulationBody);
private:
    RTC::RangeData value;
public:
    RTC::OutPort<RTC::RangeData> outPort;
private:
    std::mutex mtx;
    RangeSensor* rangeSensor;
    std::string rangeSensorName;
    std::shared_ptr<const RangeSensor::RangeData> prevRangeData;
    double controlTime;
};

class JointDataSeqInPortHandler : public InPortHandler
{
public:
    JointDataSeqInPortHandler(PortInfo& info);
    virtual void outputDataToSimulator(const BodyPtr& body) override;
    virtual void readDataFromPort() override;
private:
    RTC::TimedDoubleSeq values;
public:
    RTC::InPort<RTC::TimedDoubleSeq> inPort;
private:
    DataTypeId linkDataType;
};
    
class LinkDataInPortHandler : public InPortHandler
{
public:
    LinkDataInPortHandler(PortInfo& info);
    virtual void outputDataToSimulator(const BodyPtr& body) override;
    virtual void readDataFromPort() override;
private:
    RTC::TimedDoubleSeq values;
public:
    RTC::InPort<RTC::TimedDoubleSeq> inPort;
private:
    std::vector<std::string> linkNames;
    DataTypeId linkDataType;
};

class AbsTransformInPortHandler : public InPortHandler
{
public:
    AbsTransformInPortHandler(PortInfo& info);
    virtual void outputDataToSimulator(const BodyPtr& body) override;
    virtual void readDataFromPort() override;
private:
    RTC::TimedPose3D values;
public:
    RTC::InPort<RTC::TimedPose3D> inPort;
private:
    std::vector<std::string> linkNames;
    DataTypeId linkDataType;
};

class LightOnInPortHandler : public InPortHandler
{
public:
    LightOnInPortHandler(PortInfo& info);
    virtual void outputDataToSimulator(const BodyPtr& body) override;
    virtual void readDataFromPort() override;
private:
    RTC::TimedBooleanSeq values;
public:
    RTC::InPort<RTC::TimedBooleanSeq> inPort;
private:
    std::vector<std::string> lightNames;
};

}

#endif
