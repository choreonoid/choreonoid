/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "FountainDevice.h"
#include "SceneFountain.h"
#include <cnoid/YAMLBodyLoader>
#include <cnoid/SceneDevice>

using namespace std;
using namespace cnoid;

namespace {

bool readFountainDevice(YAMLBodyLoader& loader, Mapping& node)
{
    FountainDevicePtr fountain = new FountainDevice;
    return loader.readDevice(fountain, node);
}


SceneDevice* createSceneFountainDevice(Device* device)
{
    auto sceneFountain = new SceneFountain;
    auto sceneDevice = new SceneDevice(device, sceneFountain);

    sceneDevice->setFunctionOnTimeChanged(
        [sceneFountain](double time){
            sceneFountain->setTime(time);
            sceneFountain->notifyUpdate();
        });
            
    return sceneDevice;
}
                        
struct TypeRegistration
{
    TypeRegistration() {
        YAMLBodyLoader::addNodeType("FountainDevice", readFountainDevice);
        SceneDevice::registerSceneDeviceFactory<FountainDevice>(createSceneFountainDevice);
    }
} registration;

}


FountainDevice::FountainDevice()
{
    on_ = true;
}


FountainDevice::FountainDevice(const FountainDevice& org, bool copyStateOnly)
    : Device(org, copyStateOnly)
{
    copyStateFrom(org);
}


const char* FountainDevice::typeName()
{
    return "FountainDevice";
}


void FountainDevice::copyStateFrom(const FountainDevice& other)
{
    on_ = other.on_;
}


void FountainDevice::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(FountainDevice)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyStateFrom(static_cast<const FountainDevice&>(other));
}


DeviceState* FountainDevice::cloneState() const
{
    return new FountainDevice(*this, false);
}


Device* FountainDevice::clone() const
{
    return new FountainDevice(*this);
}


void FountainDevice::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(FountainDevice))){
        Device::forEachActualType(func);
    }
}


int FountainDevice::stateSize() const
{
    return 1;
}


const double* FountainDevice::readState(const double* buf)
{
    on_ = buf[0];
    return buf + 1;
}


double* FountainDevice::writeState(double* out_buf) const
{
    out_buf[0] = on_ ? 1.0 : 0.0;
    return out_buf + 1;
}
