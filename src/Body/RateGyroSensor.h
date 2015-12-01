/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_RATE_GYRO_SENSOR_H
#define CNOID_BODY_RATE_GYRO_SENSOR_H

#include "Device.h"
#include <boost/scoped_ptr.hpp>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT RateGyroSensor : public Device
{
    Vector3 w_; // w = omega = angular velocity
        
    struct Spec {
        Vector3 w_max;
    };
    boost::scoped_ptr<Spec> spec;
        
public:
    RateGyroSensor();
    RateGyroSensor(const RateGyroSensor& org, bool copyAll = true);
        
    virtual const char* typeName();
    void copyStateFrom(const RateGyroSensor& other);
    virtual void copyStateFrom(const DeviceState& other);
    virtual DeviceState* cloneState() const;
    virtual Device* clone() const;
    virtual void forEachActualType(boost::function<bool(const std::type_info& type)> func);
    virtual void clearState();
    virtual int stateSize() const;
    virtual const double* readState(const double* buf);
    virtual double* writeState(double* out_buf) const;

    const Vector3& w() const { return w_; }
    Vector3& w() { return w_; }

    const Vector3& w_max() const { return spec->w_max; }
    Vector3& w_max() { return spec->w_max; }
};

typedef ref_ptr<RateGyroSensor> RateGyroSensorPtr;
    
};

#endif
