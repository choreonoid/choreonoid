/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "RateGyroSensor.h"

using namespace cnoid;


RateGyroSensor::RateGyroSensor()
    : spec(new Spec)
{
    spec->w_max.setConstant(std::numeric_limits<double>::max());
    RateGyroSensor::clearState();
}


const char* RateGyroSensor::typeName()
{
    return "RateGyroSensor";
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


RateGyroSensor::RateGyroSensor(const RateGyroSensor& org, bool copyStateOnly)
    : Device(org, copyStateOnly)
{
    copyStateFrom(org);

    if(!copyStateOnly){
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
    return new RateGyroSensor(*this, true);
}


Referenced* RateGyroSensor::doClone(CloneMap*) const
{
    return new RateGyroSensor(*this);
}


void RateGyroSensor::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(RateGyroSensor))){
        Device::forEachActualType(func);
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
