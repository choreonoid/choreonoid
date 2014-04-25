/**
   \file
   \author shizuko hattori
*/

#ifndef CNOID_OPENRTM_PLUGIN_VIRTUAL_ROBOT_PORT_HANDLER_H_INCLUDED
#define CNOID_OPENRTM_PLUGIN_VIRTUAL_ROBOT_PORT_HANDLER_H_INCLUDED

#include <boost/shared_ptr.hpp>
#include <cnoid/Body>
#include <cnoid/Sensor>
#include <cnoid/Light>
#include <cnoid/Camera>
#include <cnoid/RangeSensor>
#include <cnoid/corba/CameraImage.hh>
#include <cnoid/corba/PointCloud.hh>
#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <rtm/idl/InterfaceDataTypes.hh>
#include <rtm/RTC.h>
#include <rtm/PortBase.h>
#include <rtm/OutPort.h>
#include <rtm/InPort.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>


#include "BridgeConf.h"

namespace cnoid {
    
class BodyRTCItem;

class PortHandler
{
public:
    PortHandler(PortInfo& info) : portName(info.portName){} 
    virtual ~PortHandler();
    RTC::PortService_var portRef;
    std::string portName;
};
    
typedef boost::shared_ptr<PortHandler> PortHandlerPtr;
    

class OutPortHandler : public PortHandler
{
public:
    OutPortHandler(PortInfo& info, bool synchContorller = true) 
        : PortHandler(info), synchController(synchContorller){}
    virtual void inputDataFromSimulator(BodyRTCItem* bodyRTC) = 0;
    virtual void writeDataToPort() = 0;
    template<class T> void setTime(T& value, double _time)
        {
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
    
typedef boost::shared_ptr<OutPortHandler> OutPortHandlerPtr;
    
    
class InPortHandler : public PortHandler
{
public:
    InPortHandler(PortInfo& info) : PortHandler(info){} 
    virtual void outputDataToSimulator(const BodyPtr& body) = 0;
    virtual void readDataFromPort() = 0;
};
    
typedef boost::shared_ptr<InPortHandler> InPortHandlerPtr;
    
    
class SensorStateOutPortHandler : public OutPortHandler
{
public:
    SensorStateOutPortHandler(PortInfo& info);
    virtual void inputDataFromSimulator(BodyRTCItem* bodyRTC);
    virtual void writeDataToPort();
    RTC::OutPort<RTC::TimedDoubleSeq> outPort;
private:
    RTC::TimedDoubleSeq values;
    DataTypeId dataTypeId;
};
    
    
class LinkDataOutPortHandler : public OutPortHandler
{
public:
    LinkDataOutPortHandler(PortInfo& info);
    virtual void inputDataFromSimulator(BodyRTCItem* bodyRTC);
    virtual void writeDataToPort();
    RTC::OutPort<RTC::TimedDoubleSeq> outPort;
private:
    std::vector<std::string> linkNames;
    DataTypeId linkDataType;
    RTC::TimedDoubleSeq value;
};
    
class AbsTransformOutPortHandler : public OutPortHandler
{
public:
    AbsTransformOutPortHandler(PortInfo& info);
    virtual void inputDataFromSimulator(BodyRTCItem* bodyRTC);
    virtual void writeDataToPort();
    RTC::OutPort<RTC::TimedPose3D> outPort;
private:
    std::vector<std::string> linkNames;
    DataTypeId linkDataType;
    RTC::TimedPose3D value;
};
    
    
class SensorDataOutPortHandler : public OutPortHandler
{
public:
    SensorDataOutPortHandler(PortInfo& info);
    virtual void inputDataFromSimulator(BodyRTCItem* bodyRTC);
    virtual void writeDataToPort();
    RTC::OutPort<RTC::TimedDoubleSeq> outPort;
private:
    RTC::TimedDoubleSeq value;
    std::vector<std::string> sensorNames;
};
    
class GyroSensorOutPortHandler : public OutPortHandler
{
public:
    GyroSensorOutPortHandler(PortInfo& info);
    virtual void inputDataFromSimulator(BodyRTCItem* bodyRTC);
    virtual void writeDataToPort();
    RTC::OutPort<RTC::TimedAngularVelocity3D> outPort;
private:
    RTC::TimedAngularVelocity3D value;
    std::vector<std::string> sensorNames;
};
    
class AccelerationSensorOutPortHandler : public OutPortHandler
{
public:
    AccelerationSensorOutPortHandler(PortInfo& info);
    virtual void inputDataFromSimulator(BodyRTCItem* bodyRTC);
    virtual void writeDataToPort();
    RTC::OutPort<RTC::TimedAcceleration3D> outPort;
private:
    RTC::TimedAcceleration3D value;
    std::vector<std::string> sensorNames;
};
    
class LightOnOutPortHandler : public OutPortHandler
{
public:
    LightOnOutPortHandler(PortInfo& info);
    virtual void inputDataFromSimulator(BodyRTCItem* bodyRTC);
    virtual void writeDataToPort();
    RTC::OutPort<RTC::TimedBooleanSeq> outPort;
private:
    RTC::TimedBooleanSeq value;
    std::vector<std::string> lightNames;
};
    
class CameraImageOutPortHandler : public OutPortHandler
{
public:
    CameraImageOutPortHandler(PortInfo& info, bool synchController);
    virtual void inputDataFromSimulator(BodyRTCItem* bodyRTC);
    virtual void writeDataToPort();
    void onCameraStateChanged();
    void initialize(Body* simulationBody);
    RTC::OutPort<Img::TimedCameraImage> outPort;
private:
    boost::mutex mtx;
    Img::TimedCameraImage value;
    Camera* camera;
    std::string cameraName;
    boost::shared_ptr<const Image> prevImage;
    double controlTime;
};

class CameraRangeOutPortHandler : public OutPortHandler
{
public:
    CameraRangeOutPortHandler(PortInfo& info, bool synchController);
    virtual void inputDataFromSimulator(BodyRTCItem* bodyRTC);
    virtual void writeDataToPort();
    void onCameraStateChanged();
    void initialize(Body* simulationBody);
    RTC::OutPort<PointCloudTypes::PointCloud> outPort;
private:
    boost::mutex mtx;
    PointCloudTypes::PointCloud value;
    RangeCamera* rangeCamera;
    std::string rangeCameraName;
    boost::shared_ptr<const RangeCamera::PointData> prevPoints;
    boost::shared_ptr<const Image> image;
    std::string format;
    double controlTime;
};

class RangeSensorOutPortHandler : public OutPortHandler
{
public:
    RangeSensorOutPortHandler(PortInfo& info, bool synchController);
    virtual void inputDataFromSimulator(BodyRTCItem* bodyRTC);
    virtual void writeDataToPort();
    void onRangeSensorStateChanged();
    void initialize(Body* simulationBody);
    RTC::OutPort<RTC::RangeData> outPort;
private:
    boost::mutex mtx;
    RTC::RangeData value;
    RangeSensor* rangeSensor;
    std::string rangeSensorName;
    boost::shared_ptr<const RangeSensor::RangeData> prevRangeData;
    double controlTime;
};

class JointDataSeqInPortHandler : public InPortHandler
{
public:
    JointDataSeqInPortHandler(PortInfo& info);
    virtual void outputDataToSimulator(const BodyPtr& body);
    virtual void readDataFromPort();
    RTC::InPort<RTC::TimedDoubleSeq> inPort;
private:
    RTC::TimedDoubleSeq values;
    DataTypeId linkDataType;
};
    
class LinkDataInPortHandler : public InPortHandler
{
public:
    LinkDataInPortHandler(PortInfo& info);
    virtual void outputDataToSimulator(const BodyPtr& body);
    virtual void readDataFromPort();
    RTC::InPort<RTC::TimedDoubleSeq> inPort;
private:
    RTC::TimedDoubleSeq values;
    std::vector<std::string> linkNames;
    DataTypeId linkDataType;
};

class AbsTransformInPortHandler : public InPortHandler
{
public:
    AbsTransformInPortHandler(PortInfo& info);
    virtual void outputDataToSimulator(const BodyPtr& body);
    virtual void readDataFromPort();
    RTC::InPort<RTC::TimedPose3D> inPort;
private:
    RTC::TimedPose3D values;
    std::vector<std::string> linkNames;
    DataTypeId linkDataType;
};

class LightOnInPortHandler : public InPortHandler
{
public:
    LightOnInPortHandler(PortInfo& info);
    virtual void outputDataToSimulator(const BodyPtr& body);
    virtual void readDataFromPort();
    RTC::InPort<RTC::TimedBooleanSeq> inPort;
private:
    RTC::TimedBooleanSeq values;
    std::vector<std::string> lightNames;
};
}


#endif
