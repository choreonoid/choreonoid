/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "AccelerationSensor.h"

using namespace cnoid;


AccelerationSensor::AccelerationSensor()
    : spec(new Spec)
{
    spec->dv_max.setConstant(std::numeric_limits<double>::max());
    AccelerationSensor::clearState();
}


const char* AccelerationSensor::typeName()
{
    return "AccelerationSensor";
}


void AccelerationSensor::copyStateFrom(const AccelerationSensor& other)
{
    dv_ = other.dv_;
}


void AccelerationSensor::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(AccelerationSensor)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyStateFrom(static_cast<const AccelerationSensor&>(other));
}


AccelerationSensor::AccelerationSensor(const AccelerationSensor& org, bool copyStateOnly)
    : Device(org, copyStateOnly)
{
    copyStateFrom(org);

    if(!copyStateOnly){
        spec.reset(new Spec);
        if(org.spec){
            spec->dv_max = org.spec->dv_max;
        } else {
            spec->dv_max.setConstant(std::numeric_limits<double>::max());
        }
    }
}


DeviceState* AccelerationSensor::cloneState() const
{
    return new AccelerationSensor(*this, true);
}


Referenced* AccelerationSensor::doClone(CloneMap*) const
{
    return new AccelerationSensor(*this);
}


void AccelerationSensor::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(AccelerationSensor))){
        Device::forEachActualType(func);
    }
}


void AccelerationSensor::clearState()
{
    dv_.setZero();
}


int AccelerationSensor::stateSize() const
{
    return 3;
}


const double* AccelerationSensor::readState(const double* buf)
{
    dv_ = Eigen::Map<const Vector3>(buf);
    return buf + 3;
}


double* AccelerationSensor::writeState(double* out_buf) const
{
    Eigen::Map<Vector3>(out_buf) << dv_;
    return out_buf + 3;
}
