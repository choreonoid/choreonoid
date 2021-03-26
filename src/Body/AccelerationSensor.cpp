/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "AccelerationSensor.h"
#include "StdBodyFileUtil.h"
#include <cnoid/EigenArchive>

using namespace cnoid;


AccelerationSensor::AccelerationSensor()
    : spec(new Spec)
{
    spec->dv_max.setConstant(std::numeric_limits<double>::max());
    AccelerationSensor::clearState();
}


const char* AccelerationSensor::typeName() const
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


bool AccelerationSensor::readSpecifications(const Mapping* info)
{
    read(info, { "max_acceleration", "maxAcceleration" }, spec->dv_max);
    return true;
}


bool AccelerationSensor::writeSpecifications(Mapping* info) const
{
    if(!dv_max().isConstant(std::numeric_limits<double>::max())){
        write(info, "max_acceleration", dv_max());
    }
    return true;
}


namespace {

StdBodyFileDeviceTypeRegistration<AccelerationSensor>
registerHolderDevice(
    "AccelerationSensor",
     [](StdBodyLoader* loader, const Mapping* info){
         AccelerationSensorPtr sensor = new AccelerationSensor;
         if(sensor->readSpecifications(info)){
            return loader->readDevice(sensor, info);
        }
        return false;
    },
    [](StdBodyWriter* /* writer */, Mapping* info, const AccelerationSensor* sensor)
    {
        return sensor->writeSpecifications(info);
    });
}
