/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "FireDevice.h"
#include "SceneFire.h"
#include <cnoid/YAMLBodyLoader>
#include <cnoid/SceneDevice>

using namespace std;
using namespace cnoid;

namespace {

bool readFireDevice(YAMLBodyLoader& loader, Mapping& node)
{
    FireDevicePtr fountain = new FireDevice;
    return loader.readDevice(fountain, node);
}


SceneDevice* createSceneFireDevice(Device* device)
{
    auto sceneFire = new SceneFire;
    auto sceneDevice = new SceneDevice(device, sceneFire);

    sceneDevice->setFunctionOnTimeChanged(
        [sceneFire](double time){
            sceneFire->setTime(time);
            sceneFire->notifyUpdate();
        });
            
    return sceneDevice;
}
                        
struct TypeRegistration
{
    TypeRegistration() {
        YAMLBodyLoader::addNodeType("FireDevice", readFireDevice);
        SceneDevice::registerSceneDeviceFactory<FireDevice>(createSceneFireDevice);
    }
} registration;

}


FireDevice::FireDevice()
{
    on_ = true;
}


FireDevice::FireDevice(const FireDevice& org, bool copyStateOnly)
    : Device(org, copyStateOnly)
{
    copyStateFrom(org);
}


const char* FireDevice::typeName()
{
    return "FireDevice";
}


void FireDevice::copyStateFrom(const FireDevice& other)
{
    on_ = other.on_;
}


void FireDevice::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(FireDevice)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyStateFrom(static_cast<const FireDevice&>(other));
}


DeviceState* FireDevice::cloneState() const
{
    return new FireDevice(*this, false);
}


Device* FireDevice::clone() const
{
    return new FireDevice(*this);
}


void FireDevice::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(FireDevice))){
        Device::forEachActualType(func);
    }
}


int FireDevice::stateSize() const
{
    return 1;
}


const double* FireDevice::readState(const double* buf)
{
    on_ = buf[0];
    return buf + 1;
}


double* FireDevice::writeState(double* out_buf) const
{
    out_buf[0] = on_ ? 1.0 : 0.0;
    return out_buf + 1;
}
