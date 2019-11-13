/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "ForceSensor.h"

using namespace cnoid;


ForceSensor::ForceSensor()
    : spec(new Spec)
{
    spec->F_max.setConstant(std::numeric_limits<double>::max());
    ForceSensor::clearState();
}


const char* ForceSensor::typeName()
{
    return "ForceSensor";
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


ForceSensor::ForceSensor(const ForceSensor& org, bool copyStateOnly)
    : Device(org, copyStateOnly)
{
    copyStateFrom(org);

    if(!copyStateOnly){
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
    return new ForceSensor(*this, true);
}


Referenced* ForceSensor::doClone(CloneMap*) const
{
    return new ForceSensor(*this);
}


void ForceSensor::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(ForceSensor))){
        Device::forEachActualType(func);
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
