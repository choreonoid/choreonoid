#include "ConveyorDevice.h"
#include "HolderDevice.h"
#include "Body.h"
#include "StdBodyLoader.h"
#include "StdBodyFileUtil.h"
#include <cnoid/CloneMap>
#include <cnoid/ValueTree>
#include <cnoid/EigenArchive>

using namespace std;
using namespace cnoid;


ConveyorDevice::ConveyorDevice()
{
    on_ = true;
    displacement_ = 0.0;
    speed_ = 0.0;
    
    conveyorType_ = LinearConveyor;
    axis_ = Vector3::UnitX();
}


ConveyorDevice::ConveyorDevice(const ConveyorDevice& org, bool copyStateOnly, CloneMap* cloneMap)
    : Device(org, copyStateOnly)
{
    copyConveyorDeviceStateFrom(org);

    if(!copyStateOnly){
        conveyorType_ = org.conveyorType_;
        axis_ = org.axis_;
    }
}


ConveyorDevice::~ConveyorDevice()
{

}


const char* ConveyorDevice::typeName() const
{
    return "ConveyorDevice";
}


void ConveyorDevice::copyConveyorDeviceStateFrom(const ConveyorDevice& other)
{
    on_ = other.on_;
    displacement_ = other.displacement_;
    speed_ = other.speed_;
}


void ConveyorDevice::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(ConveyorDevice)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyConveyorDeviceStateFrom(static_cast<const ConveyorDevice&>(other));
}
    

DeviceState* ConveyorDevice::cloneState() const
{
    return new ConveyorDevice(*this, true, nullptr);

}


Referenced* ConveyorDevice::doClone(CloneMap* cloneMap) const
{
    return new ConveyorDevice(*this, false, cloneMap);
}


void ConveyorDevice::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(ConveyorDevice))){
        Device::forEachActualType(func);
    }
}


bool ConveyorDevice::on() const
{
    return on_;
}


void ConveyorDevice::on(bool on)
{
    on_ = on;
}


int ConveyorDevice::stateSize() const
{
    return 3;
}
        

const double* ConveyorDevice::readState(const double* buf, int /* size */)
{
    int i = 0;
    on_ = buf[i++];
    displacement_ = buf[i++];
    speed_ = buf[i++];
    return buf + i;
}


double* ConveyorDevice::writeState(double* out_buf) const
{
    int i = 0;
    out_buf[i++] = on_ ? 1.0 : 0.0;
    out_buf[i++] = displacement_;
    out_buf[i++] = speed_;
    return out_buf + i;
}


bool ConveyorDevice::readSpecifications(const Mapping* info)
{
    string symbol;
    if(info->read("conveyor_type", symbol)){
        if(symbol == "linear"){
            setConveyorType(LinearConveyor);
        } else if(symbol == "rotary"){
            setConveyorType(RotaryConveyor);
        }
    }
    read(info, "axis", axis_);
    info->read("speed", speed_);

    return true;
}


bool ConveyorDevice::writeSpecifications(Mapping* info) const
{
    info->write("conveyor_type", (conveyorType_ == LinearConveyor) ? "linear" : "rotary");
    write(info, "axis", axis_);
    if(speed_ != 0.0){
        info->write("speed", speed_);
    }
    return true;
}


namespace {

StdBodyFileDeviceTypeRegistration<ConveyorDevice>
registerConveyorDevice(
    "Conveyor",
    [](StdBodyLoader* loader, const Mapping* info){
        ConveyorDevicePtr conveyor = new ConveyorDevice;
        if(conveyor->readSpecifications(info)){
            return loader->readDevice(conveyor, info);
        }
        return false;
    },
    [](StdBodyWriter* /* writer */, Mapping* info, const ConveyorDevice* conveyor){
        return conveyor->writeSpecifications(info);
    });

}
