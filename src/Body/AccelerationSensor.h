/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_ACCELERATION_SENSOR_H
#define CNOID_BODY_ACCELERATION_SENSOR_H

#include "Device.h"
#include <boost/scoped_ptr.hpp>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT AccelerationSensor : public Device
{
    Vector3 dv_;

    struct Spec {
        Vector3 dv_max;
    };
    boost::scoped_ptr<Spec> spec;

public:
    AccelerationSensor();
    AccelerationSensor(const AccelerationSensor& org, bool copyAll = true);

    virtual const char* typeName();
    void copyStateFrom(const AccelerationSensor& other);
    virtual void copyStateFrom(const DeviceState& other);
    virtual DeviceState* cloneState() const;
    virtual Device* clone() const;
    virtual void forEachActualType(boost::function<bool(const std::type_info& type)> func);
    virtual void clearState();
    virtual int stateSize() const;
    virtual const double* readState(const double* buf);
    virtual double* writeState(double* out_buf) const;

    const Vector3& dv() const { return dv_; }
    Vector3& dv() { return dv_; }

    const Vector3& dv_max() const { return spec->dv_max; }
    Vector3& dv_max() { return spec->dv_max; }
};

typedef ref_ptr<AccelerationSensor> AccelerationSensorPtr;

// for the backward compatibility
//typedef AccelerationSensorPtr AccelSensor;
//typedef ref_ptr<AccelSensor> AccelSensorPtr;
    
};

#endif
