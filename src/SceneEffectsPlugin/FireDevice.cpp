/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "FireDevice.h"
#include "SceneFire.h"
#include <cnoid/YAMLBodyLoader>
#include <cnoid/SceneDevice>
#include <cnoid/EigenArchive>

using namespace std;
using namespace cnoid;

namespace {

YAMLBodyLoader::NodeTypeRegistration
registerFireDevice(
    "FireDevice",
    [](YAMLBodyLoader& loader, Mapping& node){
        FireDevicePtr fire = new FireDevice;
        fire->particleSystem().readParameters(loader.sceneReader(), node);
        return loader.readDevice(fire, node);
    });

SceneDevice::FactoryRegistration<FireDevice>
registerSceneFireDeviceFactory(
    [](Device* device){
        auto fireDevice = static_cast<FireDevice*>(device);
        auto sceneFire = new SceneFire;
        auto sceneDevice = new SceneDevice(fireDevice, sceneFire);

        sceneDevice->setFunctionOnStateChanged(
            [sceneFire, fireDevice](){
                sceneFire->particleSystem() = fireDevice->particleSystem();
                sceneFire->notifyUpdate();
            });

        sceneDevice->setFunctionOnTimeChanged(
            [sceneFire](double time){
                sceneFire->setTime(time);
                sceneFire->notifyUpdate();
            });
            
        return sceneDevice;
    });

}


FireDevice::FireDevice()
{
    on_ = true;
}


FireDevice::FireDevice(const FireDevice& org, bool copyStateOnly)
    : Device(org, copyStateOnly),
      on_(org.on_),
      particleSystem_(org.particleSystem_)
{

}


const char* FireDevice::typeName()
{
    return "FireDevice";
}


void FireDevice::copyStateFrom(const FireDevice& other)
{
    on_ = other.on_;
    particleSystem_ = other.particleSystem_;
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
    return 8;
}


const double* FireDevice::readState(const double* buf)
{
    int i = 0;
    auto& ps = particleSystem_;

    on_ = buf[i++];
    ps.setNumParticles(buf[i++]);
    ps.setAcceleration(Vector3f(buf[i++], buf[i++], buf[i++]));
    ps.setEmissionRange(buf[i++]);
    ps.setInitialSpeedAverage(buf[i++]);
    ps.setInitialSpeedVariation(buf[i++]);
    
    return buf + i;
}


double* FireDevice::writeState(double* out_buf) const
{
    int i = 0;
    auto& ps = particleSystem_;
    
    out_buf[i++] = on_ ? 1.0 : 0.0;
    out_buf[i++] = ps.numParticles();
    out_buf[i++] = ps.acceleration()[0];
    out_buf[i++] = ps.acceleration()[1];
    out_buf[i++] = ps.acceleration()[2];
    out_buf[i++] = ps.emissionRange();
    out_buf[i++] = ps.initialSpeedAverage();
    out_buf[i++] = ps.initialSpeedVariation();

    return out_buf + i;
}
