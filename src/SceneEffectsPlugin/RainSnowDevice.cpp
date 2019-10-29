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

template <class DeviceType>
bool readDevice(YAMLBodyLoader& loader, Mapping& node)
{
    ref_ptr<DeviceType> device = new DeviceType;
    device->particleSystem().readParameters(loader.sceneReader(), node);
    return loader.readDevice(device, node);
}

template <class DeviceType, class SceneNodeType>
SceneDevice* createSceneDevice(Device* device)
{
    auto customDevice = static_cast<DeviceType*>(device);
    auto sceneNode = new SceneNodeType;
    auto sceneDevice = new SceneDevice(device, sceneNode);

    sceneDevice->setFunctionOnStateChanged(
        [sceneNode, customDevice](){
            sceneNode->particleSystem() = customDevice->particleSystem();
            sceneNode->notifyUpdate();
        });

    sceneDevice->setFunctionOnTimeChanged(
        [sceneNode](double time){
            sceneNode->setTime(time);
            sceneNode->notifyUpdate();
        });
            
    return sceneDevice;
}

YAMLBodyLoader::NodeTypeRegistration
registerSnowDevice("SnowDevice", readDevice<SnowDevice>);

SceneDevice::FactoryRegistration<SnowDevice>
registerSceneSmokeDeviceFactory(createSceneDevice<SnowDevice, SceneSnow>);

YAMLBodyLoader::NodeTypeRegistration
registerRainDevice("RainDevice", readDevice<RainDevice>);

SceneDevice::FactoryRegistration<RainDevice>
registerSceneRainDeviceFactory(createSceneDevice<RainDevice, SceneRain>);

}


RainSnowDevice::RainSnowDevice()
{
    on_ = true;
    particleSystem_.setNumParticles(8000);
}


RainSnowDevice::RainSnowDevice(const RainSnowDevice& org, bool copyStateOnly)
    : Device(org, copyStateOnly),
      on_(org.on_),
      particleSystem_(org.particleSystem_)
{

}


void RainSnowDevice::copyStateFrom(const RainSnowDevice& other)
{
    on_ = other.on_;
    particleSystem_ = other.particleSystem_;
}


void RainSnowDevice::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(RainSnowDevice))){
        Device::forEachActualType(func);
    }
}


int RainSnowDevice::stateSize() const
{
    return 1;
}


const double* RainSnowDevice::readState(const double* buf)
{
    on_ = buf[0];
    return buf + 1;
}


double* RainSnowDevice::writeState(double* out_buf) const
{
    out_buf[0] = on_ ? 1.0 : 0.0;
    return out_buf + 1;
}


RainDevice::RainDevice()
{
    particleSystem().setParticleSize(0.02f);
}


RainDevice::RainDevice(const RainDevice& org, bool copyStateOnly)
    : RainSnowDevice(org, copyStateOnly)
{

}


const char* RainDevice::typeName()
{
    return "RainDevice";
}


void RainDevice::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(RainDevice)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    RainSnowDevice::copyStateFrom(static_cast<const RainDevice&>(other));
}


DeviceState* RainDevice::cloneState() const
{
    return new RainDevice(*this, true);
}


Referenced* RainDevice::doClone(CloneMap*) const
{
    return new RainDevice(*this);
}


void RainDevice::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(RainDevice))){
        RainSnowDevice::forEachActualType(func);
    }
}


SnowDevice::SnowDevice()
{
    particleSystem().setParticleSize(0.025f);
}


SnowDevice::SnowDevice(const SnowDevice& org, bool copyStateOnly)
    : RainSnowDevice(org, copyStateOnly)
{
    
}


const char* SnowDevice::typeName()
{
    return "SnowDevice";
}


void SnowDevice::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(SnowDevice)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    RainSnowDevice::copyStateFrom(static_cast<const SnowDevice&>(other));
}


DeviceState* SnowDevice::cloneState() const
{
    return new SnowDevice(*this, false);
}


Referenced* SnowDevice::doClone(CloneMap*) const
{
    return new SnowDevice(*this);
}


void SnowDevice::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(SnowDevice))){
        RainSnowDevice::forEachActualType(func);
    }
}
