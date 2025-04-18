#include "RateGyroSensor.h"
#include "StdBodyFileUtil.h"
#include <cnoid/EigenArchive>

using namespace cnoid;


RateGyroSensor::RateGyroSensor()
    : spec(new Spec)
{
    spec->w_max.setConstant(std::numeric_limits<double>::max());
    RateGyroSensor::clearState();
}


const char* RateGyroSensor::typeName() const
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


const double* RateGyroSensor::readState(const double* buf, int /* size */)
{
    w_ = Eigen::Map<const Vector3>(buf);
    return buf + 3;
}


double* RateGyroSensor::writeState(double* out_buf) const
{
    Eigen::Map<Vector3>(out_buf) << w_;
    return out_buf + 3;
}


bool RateGyroSensor::readSpecifications(const Mapping* info)
{
    read(info, { "max_angular_velocity", "maxAngularVelocity" }, spec->w_max);
    return true;
}


bool RateGyroSensor::writeSpecifications(Mapping* info) const
{
    if(!spec->w_max.isConstant(std::numeric_limits<double>::max())){
        write(info, "max_angular_velocity", degree(spec->w_max));
    }
    return true;
}


namespace {

StdBodyFileDeviceTypeRegistration<RateGyroSensor>
registerHolderDevice(
    "RateGyroSensor",
     [](StdBodyLoader* loader, const Mapping* info){
         RateGyroSensorPtr sensor = new RateGyroSensor;
         if(sensor->readSpecifications(info)){
            return loader->readDevice(sensor, info);
        }
        return false;
    },
    [](StdBodyWriter* /* writer */, Mapping* info, const RateGyroSensor* sensor)
    {
        return sensor->writeSpecifications(info);
    });
}
