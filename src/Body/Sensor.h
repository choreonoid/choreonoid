/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_SENSORS_H
#define CNOID_BODY_SENSORS_H

#include "Device.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT Sensor : public Device
{
protected:
    Sensor() { }
    Sensor(const Sensor& org, bool copyAll = true) : Device(org, copyAll) { }
public:
    virtual void forEachActualType(boost::function<bool(const std::type_info& type)> func);
};

typedef ref_ptr<Sensor> SensorPtr;


class CNOID_EXPORT ForceSensor : public Sensor
{
    Vector6 F_; // f (linear force) + tau (torque)
        
    struct Spec {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Vector6 F_max;
    };
    boost::scoped_ptr<Spec> spec;
        
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    ForceSensor();
    ForceSensor(const ForceSensor& org, bool copyAll = true);

    void copyStateFrom(const ForceSensor& other);
    virtual void copyStateFrom(const DeviceState& other);
    virtual DeviceState* cloneState() const;
    virtual Device* clone() const;
    virtual void forEachActualType(boost::function<bool(const std::type_info& type)> func);
    virtual void clearState();
    virtual int stateSize() const;
    virtual const double* readState(const double* buf);
    virtual double* writeState(double* out_buf) const;
        
    const Vector6& F() const { return F_; }
    Vector6& F() { return F_; }

    Vector6::FixedSegmentReturnType<3>::Type f() { return F_.head<3>(); }
    Vector6::ConstFixedSegmentReturnType<3>::Type f() const { return F_.head<3>(); }

    Vector6::FixedSegmentReturnType<3>::Type tau() { return F_.tail<3>(); }
    Vector6::ConstFixedSegmentReturnType<3>::Type tau() const { return F_.tail<3>(); }

    const Vector6& F_max() const { return spec->F_max; }
    Vector6& F_max() { return spec->F_max; }
};

typedef ref_ptr<ForceSensor> ForceSensorPtr;
    

class CNOID_EXPORT RateGyroSensor : public Sensor
{
    Vector3 w_; // w = omega = angular velocity
        
    struct Spec {
        Vector3 w_max;
    };
    boost::scoped_ptr<Spec> spec;
        
public:
    RateGyroSensor();
    RateGyroSensor(const RateGyroSensor& org, bool copyAll = true);
        
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

    
class CNOID_EXPORT AccelSensor : public Sensor
{
    Vector3 dv_;

    struct Spec {
        Vector3 dv_max;
    };
    boost::scoped_ptr<Spec> spec;

public:
    AccelSensor();
    AccelSensor(const AccelSensor& org, bool copyAll = true);

    void copyStateFrom(const AccelSensor& other);
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

typedef ref_ptr<AccelSensor> AccelSensorPtr;
    
};


#endif
