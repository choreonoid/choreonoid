/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_RATE_GYRO_SENSOR_H
#define CNOID_BODY_RATE_GYRO_SENSOR_H

#include "Device.h"
#include <memory>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT RateGyroSensor : public Device
{
public:
    RateGyroSensor();
    RateGyroSensor(const RateGyroSensor& org, bool copyStateOnly = false);
        
    virtual const char* typeName() override;
    void copyStateFrom(const RateGyroSensor& other);
    virtual void copyStateFrom(const DeviceState& other) override;
    virtual DeviceState* cloneState() const override;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;
    virtual void clearState() override;
    virtual int stateSize() const override;
    virtual const double* readState(const double* buf) override;
    virtual double* writeState(double* out_buf) const override;

    const Vector3& w() const { return w_; }
    Vector3& w() { return w_; }

    const Vector3& w_max() const { return spec->w_max; }
    Vector3& w_max() { return spec->w_max; }

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    Vector3 w_; // w = omega = angular velocity
        
    struct Spec {
        Vector3 w_max;
    };
    std::unique_ptr<Spec> spec;
};

typedef ref_ptr<RateGyroSensor> RateGyroSensorPtr;
    
}

#endif
