/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "RainSnowDevice.h"
#include "SceneRainSnow.h"
#include <cnoid/YAMLBodyLoader>
#include <cnoid/SceneDevice>

using namespace std;
using namespace cnoid;

namespace {

bool readSnowDevice(YAMLBodyLoader& loader, Mapping& node)
{
    SnowDevicePtr fountain = new SnowDevice;
    return loader.readDevice(fountain, node);
}


SceneDevice* createSceneSnowDevice(Device* device)
{
    auto sceneSnow = new SceneSnow;
    auto sceneDevice = new SceneDevice(device, sceneSnow);

    sceneDevice->setFunctionOnTimeChanged(
        [sceneSnow](double time){
            sceneSnow->setTime(time);
            sceneSnow->notifyUpdate();
        });
            
    return sceneDevice;
}
                        
struct TypeRegistration
{
    TypeRegistration() {
        YAMLBodyLoader::addNodeType("SnowDevice", readSnowDevice);
        SceneDevice::registerSceneDeviceFactory<SnowDevice>(createSceneSnowDevice);
    }
} registration;

}


SnowDevice::SnowDevice()
{
    on_ = true;
}


SnowDevice::SnowDevice(const SnowDevice& org, bool copyStateOnly)
    : Device(org, copyStateOnly)
{
    copyStateFrom(org);
}


const char* SnowDevice::typeName()
{
    return "SnowDevice";
}


void SnowDevice::copyStateFrom(const SnowDevice& other)
{
    on_ = other.on_;
}


void SnowDevice::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(SnowDevice)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyStateFrom(static_cast<const SnowDevice&>(other));
}


DeviceState* SnowDevice::cloneState() const
{
    return new SnowDevice(*this, false);
}


Device* SnowDevice::clone() const
{
    return new SnowDevice(*this);
}


void SnowDevice::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(SnowDevice))){
        Device::forEachActualType(func);
    }
}


int SnowDevice::stateSize() const
{
    return 1;
}


const double* SnowDevice::readState(const double* buf)
{
    on_ = buf[0];
    return buf + 1;
}


double* SnowDevice::writeState(double* out_buf) const
{
    out_buf[0] = on_ ? 1.0 : 0.0;
    return out_buf + 1;
}
