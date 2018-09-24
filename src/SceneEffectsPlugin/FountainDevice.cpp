/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "FountainDevice.h"
#include "SceneFountain.h"
#include <cnoid/YAMLBodyLoader>
#include <cnoid/SceneDevice>
#include <cnoid/Body>
#include <cnoid/EigenUtil>

using namespace std;
using namespace cnoid;

namespace {

YAMLBodyLoader::NodeTypeRegistration registerFountainDevice(
    "FountainDevice",
    [](YAMLBodyLoader& loader, Mapping& node){
        FountainDevicePtr fountain = new FountainDevice;
        fountain->particleSystem().readParameters(loader.sceneReader(), node);
        return loader.readDevice(fountain, node);
    });

SceneDevice::FactoryRegistration<FountainDevice>
registerSceneFountainDeviceFactory(
    [](Device* device){
        auto fountainDevice = static_cast<FountainDevice*>(device);
        auto sceneFountain = new SceneFountain;
        auto sceneDevice = new SceneDevice(device, sceneFountain);
        
        sceneDevice->setFunctionOnStateChanged(
            [sceneFountain, fountainDevice](){
                sceneFountain->particleSystem() = fountainDevice->particleSystem();
                sceneFountain->notifyUpdate();
            });
        
        sceneDevice->setFunctionOnTimeChanged(
            [sceneFountain](double time){
                sceneFountain->setTime(time);
                sceneFountain->notifyUpdate();
            });
        
        return sceneDevice;
    });

}


FountainDevice::FountainDevice()
{
    auto& ps = particleSystem_;
    ps.setLifeTime(3.0f);
    ps.setParticleSize(0.06f);
    ps.setNumParticles(10000);
    ps.setAcceleration(Vector3f(0.0f, 0.0f, -0.2f));
    ps.setEmissionRange(radian(30.0f));
}


FountainDevice::FountainDevice(const FountainDevice& org, bool copyStateOnly)
    : Device(org, copyStateOnly),
      particleSystem_(org.particleSystem_)
{

}


const char* FountainDevice::typeName()
{
    return "FountainDevice";
}


void FountainDevice::copyStateFrom(const FountainDevice& other)
{
    particleSystem_ = other.particleSystem_;
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


bool FountainDevice::on() const
{
    return particleSystem_.on();
}


void FountainDevice::on(bool on)
{
    if(on && !particleSystem_.on()){
        if(link()){
            particleSystem_.setOffsetTime(-link()->body()->currentTime());
        }
    }
    particleSystem_.on(on);
}


int FountainDevice::stateSize() const
{
    return 7;
}


const double* FountainDevice::readState(const double* buf)
{
    int i = 0;
    auto& ps = particleSystem_;

    ps.on(buf[i++]);
    ps.setOffsetTime(buf[i++]);
    ps.setNumParticles(buf[i++]);
    ps.setAcceleration(Vector3f(buf[i++], buf[i++], buf[i++]));
    ps.setEmissionRange(buf[i++]);
    
    return buf + i;
}


double* FountainDevice::writeState(double* out_buf) const
{
    int i = 0;
    auto& ps = particleSystem_;
    
    out_buf[i++] = ps.on() ? 1.0 : 0.0;
    out_buf[i++] = ps.offsetTime();
    out_buf[i++] = ps.numParticles();
    out_buf[i++] = ps.acceleration()[0];
    out_buf[i++] = ps.acceleration()[1];
    out_buf[i++] = ps.acceleration()[2];
    out_buf[i++] = ps.emissionRange();

    return out_buf + i;
}
