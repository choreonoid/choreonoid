#include "Imu.h"
#include "StdBodyFileUtil.h"
#include <cnoid/EigenArchive>

using namespace cnoid;


Imu::Imu()
    : spec(new Spec)
{
    spec->w_max.setConstant(std::numeric_limits<double>::max());
    spec->dv_max.setConstant(std::numeric_limits<double>::max());
    Imu::clearState();
}


Imu::Imu(const Imu& org, bool copyStateOnly)
    : Device(org, copyStateOnly)
{
    copyStateFrom(org);

    if(!copyStateOnly){
        spec.reset(new Spec);
        if(org.spec){
            spec->w_max = org.spec->w_max;
            spec->dv_max = org.spec->dv_max;
        } else {
            spec->w_max.setConstant(std::numeric_limits<double>::max());
            spec->dv_max.setConstant(std::numeric_limits<double>::max());
        }
    }
}


const char* Imu::typeName() const
{
    return "Imu";
}


void Imu::copyStateFrom(const Imu& other)
{
    w_ = other.w_;
    dv_ = other.dv_;
}


void Imu::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(Imu)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyStateFrom(static_cast<const Imu&>(other));
}


DeviceState* Imu::cloneState() const
{
    return new Imu(*this, true);
}


Referenced* Imu::doClone(CloneMap*) const
{
    return new Imu(*this);
}


void Imu::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(Imu))){
        Device::forEachActualType(func);
    }
}


void Imu::clearState()
{
    w_.setZero();
    dv_.setZero();
}


int Imu::stateSize() const
{
    return 6;
}


const double* Imu::readState(const double* buf)
{
    w_ = Eigen::Map<const Vector3>(buf);
    dv_ = Eigen::Map<const Vector3>(buf + 3);
    return buf + 6;
}


double* Imu::writeState(double* out_buf) const
{
    Eigen::Map<Vector3>(out_buf) << w_;
    Eigen::Map<Vector3>(out_buf + 3) << dv_;
    return out_buf + 6;
}


bool Imu::readSpecifications(const Mapping* info)
{
    read(info, "max_angular_velocity", spec->w_max);
    read(info, "max_acceleration", spec->dv_max);
    return true;
}


bool Imu::writeSpecifications(Mapping* info) const
{
    if(!spec->w_max.isConstant(std::numeric_limits<double>::max())){
        write(info, "max_angular_velocity", degree(spec->w_max));
    }
    if(!dv_max().isConstant(std::numeric_limits<double>::max())){
        write(info, "max_acceleration", dv_max());
    }
    return true;
}


namespace {

StdBodyFileDeviceTypeRegistration<Imu>
registerHolderDevice(
    "IMU",
     [](StdBodyLoader* loader, const Mapping* info){
         ImuPtr sensor = new Imu;
         if(sensor->readSpecifications(info)){
            return loader->readDevice(sensor, info);
        }
        return false;
    },
    [](StdBodyWriter* /* writer */, Mapping* info, const Imu* sensor)
    {
        return sensor->writeSpecifications(info);
    });
}
