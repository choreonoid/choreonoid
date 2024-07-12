#include "ZmpDevice.h"
#include "StdBodyFileUtil.h"
#include <cnoid/CloneMap>

using namespace std;
using namespace cnoid;


ZmpDevice::ZmpDevice()
{
    zmp_.setZero();
    on_ = true;
}


ZmpDevice::ZmpDevice(const ZmpDevice& org, bool copyStateOnly, CloneMap* cloneMap)
    : Device(org, copyStateOnly)
{
    copyZmpDeviceStateFrom(org);
}


ZmpDevice::~ZmpDevice()
{

}


const char* ZmpDevice::typeName() const
{
    return "ZmpDevice";
}


void ZmpDevice::copyZmpDeviceStateFrom(const ZmpDevice& other)
{
    zmp_ = other.zmp_;
    on_ = other.on_;
}


void ZmpDevice::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(ZmpDevice)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyZmpDeviceStateFrom(static_cast<const ZmpDevice&>(other));
}
    

DeviceState* ZmpDevice::cloneState() const
{
    return new ZmpDevice(*this, true, nullptr);

}


Referenced* ZmpDevice::doClone(CloneMap* cloneMap) const
{
    return new ZmpDevice(*this, false, cloneMap);
}


void ZmpDevice::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(ZmpDevice))){
        Device::forEachActualType(func);
    }
}


bool ZmpDevice::on() const
{
    return on_;
}


void ZmpDevice::on(bool on)
{
    on_ = on;
}


int ZmpDevice::stateSize() const
{
    return 2;
}


const double* ZmpDevice::readState(const double* buf)
{
    int i = 0;
    on_ = buf[i++];
    zmp_[0] = buf[i++];
    zmp_[1] = buf[i++];
    zmp_[2] = buf[i++];
    return buf + i;
}


double* ZmpDevice::writeState(double* out_buf) const
{
    int i = 0;
    out_buf[i++] = on_ ? 1.0 : 0.0;
    out_buf[i++] = zmp_[0];
    out_buf[i++] = zmp_[1];
    out_buf[i++] = zmp_[2];
    return out_buf + i;
}


namespace {

StdBodyFileDeviceTypeRegistration<ZmpDevice>
registerZmpDevice(
    "ZmpDevice",
    [](StdBodyLoader* loader, const Mapping* info){
        ZmpDevicePtr device = new ZmpDevice;
        return loader->readDevice(device, info);
    },
    [](StdBodyWriter* /* writer */, Mapping* /* info */, const ZmpDevice* /* zmpDevice */){
        return true;
    });
}
