#ifndef CNOID_BODY_BASIC_SENSOR_SIMULATION_HELPER_H
#define CNOID_BODY_BASIC_SENSOR_SIMULATION_HELPER_H

#include "DeviceList.h"
#include "ForceSensor.h"
#include "RateGyroSensor.h"
#include "AccelerationSensor.h"
#include "Imu.h"
#include "exportdecl.h"

namespace cnoid {

class Body;

class CNOID_EXPORT BasicSensorSimulationHelper
{
public:
    BasicSensorSimulationHelper();
    ~BasicSensorSimulationHelper();

    void setOldAccelSensorCalcMode(bool on);
    
    void initialize(Body* body, double timeStep, const Vector3& gravityAcceleration);

    bool isActive() const { return isActive_; }
    bool hasGyroOrAccelerationSensors() const {
        return !rateGyroSensors_.empty() || !accelerationSensors_.empty() || !imus_.empty();
    }

    const DeviceList<ForceSensor>& forceSensors() const { return forceSensors_; }
    const DeviceList<RateGyroSensor>& rateGyroSensors() const { return rateGyroSensors_; }
    const DeviceList<AccelerationSensor>& accelerationSensors() const { return accelerationSensors_; }
    const DeviceList<Imu>& imus() const { return imus_; }

    void updateGyroAndAccelerationSensors();

private:
    bool isActive_;
    DeviceList<ForceSensor> forceSensors_;
    DeviceList<RateGyroSensor> rateGyroSensors_;
    DeviceList<AccelerationSensor> accelerationSensors_;
    DeviceList<Imu> imus_;

    class Impl;
    Impl* impl;
};

}

#endif
