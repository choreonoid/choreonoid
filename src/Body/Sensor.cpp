/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "Sensor.h"

using namespace cnoid;


void Sensor::forEachActualType(boost::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(Sensor))){
        Device::forEachActualType(func);
    }
}


ForceSensor::ForceSensor()
    : spec(new Spec)
{
    spec->F_max.setConstant(std::numeric_limits<double>::max());
    ForceSensor::clearState();
}


void ForceSensor::copyStateFrom(const ForceSensor& other)
{
    F_ = other.F_;
}


void ForceSensor::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(ForceSensor)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyStateFrom(static_cast<const ForceSensor&>(other));
}


ForceSensor::ForceSensor(const ForceSensor& org, bool copyAll)
    : Sensor(org, copyAll)
{
    copyStateFrom(org);

    if(copyAll){
        spec.reset(new Spec);
        if(org.spec){
            spec->F_max = org.spec->F_max;
        } else {
            spec->F_max.setConstant(std::numeric_limits<double>::max());
        }
    }
}


DeviceState* ForceSensor::cloneState() const
{
    return new ForceSensor(*this, false);
}


Device* ForceSensor::clone() const
{
    return new ForceSensor(*this);
}


void ForceSensor::forEachActualType(boost::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(ForceSensor))){
        Sensor::forEachActualType(func);
    }
}


void ForceSensor::clearState()
{
    F_.setZero();
}


int ForceSensor::stateSize() const
{
    return 6;
}


const double* ForceSensor::readState(const double* buf)
{
    F_ = Eigen::Map<const Vector6>(buf);
    return buf + 6;
}


double* ForceSensor::writeState(double* out_buf) const
{
    Eigen::Map<Vector6>(out_buf) << F_;
    return out_buf + 6;
}


RateGyroSensor::RateGyroSensor()
    : spec(new Spec)
{
    spec->w_max.setConstant(std::numeric_limits<double>::max());
    RateGyroSensor::clearState();
}


void RateGyroSensor::copyStateFrom(const RateGyroSensor& other)
{
    w_ = other.w_;
}


void RateGyroSensor::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(RateGyroSensor)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyStateFrom(static_cast<const RateGyroSensor&>(other));
}


RateGyroSensor::RateGyroSensor(const RateGyroSensor& org, bool copyAll)
    : Sensor(org, copyAll)
{
    copyStateFrom(org);

    if(copyAll){
        spec.reset(new Spec);
        if(org.spec){
            spec->w_max = org.spec->w_max;
        } else {
            spec->w_max.setConstant(std::numeric_limits<double>::max());
        }
    }
}


DeviceState* RateGyroSensor::cloneState() const
{
    return new RateGyroSensor(*this, false);
}


Device* RateGyroSensor::clone() const
{
    return new RateGyroSensor(*this);
}


void RateGyroSensor::forEachActualType(boost::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(RateGyroSensor))){
        Sensor::forEachActualType(func);
    }
}


void RateGyroSensor::clearState()
{
    w_.setZero();
}


int RateGyroSensor::stateSize() const
{
    return 3;
}


const double* RateGyroSensor::readState(const double* buf)
{
    w_ = Eigen::Map<const Vector3>(buf);
    return buf + 3;
}


double* RateGyroSensor::writeState(double* out_buf) const
{
    Eigen::Map<Vector3>(out_buf) << w_;
    return out_buf + 3;
}


AccelSensor::AccelSensor()
    : spec(new Spec)
{
    spec->dv_max.setConstant(std::numeric_limits<double>::max());
    AccelSensor::clearState();
}


void AccelSensor::copyStateFrom(const AccelSensor& other)
{
    dv_ = other.dv_;
}


void AccelSensor::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(AccelSensor)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyStateFrom(static_cast<const AccelSensor&>(other));
}


AccelSensor::AccelSensor(const AccelSensor& org, bool copyAll)
    : Sensor(org, copyAll)
{
    copyStateFrom(org);

    if(copyAll){
        spec.reset(new Spec);
        if(org.spec){
            spec->dv_max = org.spec->dv_max;
        } else {
            spec->dv_max.setConstant(std::numeric_limits<double>::max());
        }
    }
}


DeviceState* AccelSensor::cloneState() const
{
    return new AccelSensor(*this, false);
}


Device* AccelSensor::clone() const
{
    return new AccelSensor(*this);
}


void AccelSensor::forEachActualType(boost::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(AccelSensor))){
        Sensor::forEachActualType(func);
    }
}


void AccelSensor::clearState()
{
    dv_.setZero();
}


int AccelSensor::stateSize() const
{
    return 3;
}


const double* AccelSensor::readState(const double* buf)
{
    dv_ = Eigen::Map<const Vector3>(buf);
    return buf + 3;
}


double* AccelSensor::writeState(double* out_buf) const
{
    Eigen::Map<Vector3>(out_buf) << dv_;
    return out_buf + 3;
}
