/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_BASIC_SENSOR_SIMULATION_HELPER_H
#define CNOID_BODY_BASIC_SENSOR_SIMULATION_HELPER_H

#include "Body.h"
#include "BasicSensors.h"
#include "exportdecl.h"

namespace cnoid {

class Referenced;
class BasicSensorSimulationHelperImpl;

class CNOID_EXPORT BasicSensorSimulationHelper
{
public:
    BasicSensorSimulationHelper();
    ~BasicSensorSimulationHelper();

    void initialize(BodyPtr body, double timeStep, const Vector3& gravityAcceleration);

    bool isActive() const { return isActive_; }
    bool hasGyroOrAccelSensors() const { return !gyroSensors_.empty() || !accelSensors_.empty(); }

    const DeviceList<ForceSensor>& forceSensors() const { return forceSensors_; }
    const DeviceList<RateGyroSensor>& gyroSensors() const { return gyroSensors_; }
    const DeviceList<AccelSensor>& accelSensors() const { return accelSensors_; }
        
    void updateGyroAndAccelSensors();

private:
    BasicSensorSimulationHelperImpl* impl;
    bool isActive_;
    DeviceList<ForceSensor> forceSensors_;
    DeviceList<RateGyroSensor> gyroSensors_;
    DeviceList<AccelSensor> accelSensors_;

    friend class BasicSensorSimulationHelperImpl;
};

}

#endif
