/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "SmokeDevice.h"
#include "SceneSmoke.h"
#include <cnoid/YAMLBodyLoader>
#include <cnoid/SceneDevice>

using namespace std;
using namespace cnoid;

namespace {

bool readSmokeDevice(YAMLBodyLoader& loader, Mapping& node)
{
    SmokeDevicePtr fountain = new SmokeDevice;
    return loader.readDevice(fountain, node);
}


SceneDevice* createSceneSmokeDevice(Device* device)
{
    auto sceneSmoke = new SceneSmoke;
    auto sceneDevice = new SceneDevice(device, sceneSmoke);

    sceneDevice->setFunctionOnTimeChanged(
        [sceneSmoke](double time){
            sceneSmoke->setTime(time);
            sceneSmoke->notifyUpdate();
        });
            
    return sceneDevice;
}
                        
struct TypeRegistration
{
    TypeRegistration() {
        YAMLBodyLoader::addNodeType("SmokeDevice", readSmokeDevice);
        SceneDevice::registerSceneDeviceFactory<SmokeDevice>(createSceneSmokeDevice);
    }
} registration;

}


SmokeDevice::SmokeDevice()
{
    on_ = true;
}


SmokeDevice::SmokeDevice(const SmokeDevice& org, bool copyStateOnly)
    : Device(org, copyStateOnly)
{
    copyStateFrom(org);
}


const char* SmokeDevice::typeName()
{
    return "SmokeDevice";
}


void SmokeDevice::copyStateFrom(const SmokeDevice& other)
{
    on_ = other.on_;
}


void SmokeDevice::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(SmokeDevice)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyStateFrom(static_cast<const SmokeDevice&>(other));
}


DeviceState* SmokeDevice::cloneState() const
{
    return new SmokeDevice(*this, false);
}


Device* SmokeDevice::clone() const
{
    return new SmokeDevice(*this);
}


void SmokeDevice::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(SmokeDevice))){
        Device::forEachActualType(func);
    }
}


int SmokeDevice::stateSize() const
{
    return 1;
}


const double* SmokeDevice::readState(const double* buf)
{
    on_ = buf[0];
    return buf + 1;
}


double* SmokeDevice::writeState(double* out_buf) const
{
    out_buf[0] = on_ ? 1.0 : 0.0;
    return out_buf + 1;
}
