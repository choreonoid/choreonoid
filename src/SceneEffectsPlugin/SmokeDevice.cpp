/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "SmokeDevice.h"
#include "SceneSmoke.h"
#include <cnoid/YAMLBodyLoader>
#include <cnoid/Body>
#include <cnoid/SceneDevice>
#include <cnoid/EigenUtil>

using namespace std;
using namespace cnoid;

namespace {

YAMLBodyLoader::NodeTypeRegistration
registerSmokeDevice(
    "SmokeDevice",
    [](YAMLBodyLoader& loader, Mapping& node){
        SmokeDevicePtr smoke = new SmokeDevice;
        smoke->particleSystem().readParameters(loader.sceneReader(), node);
        return loader.readDevice(smoke, node);
    });

SceneDevice::FactoryRegistration<SmokeDevice>
registerSceneSmokeDeviceFactory(
    [](Device* device){
        auto smokeDevice = static_cast<SmokeDevice*>(device);
        auto sceneSmoke = new SceneSmoke;
        auto sceneDevice = new SceneDevice(device, sceneSmoke);

        sceneDevice->setFunctionOnStateChanged(
            [sceneSmoke, smokeDevice](){
                sceneSmoke->particleSystem() = smokeDevice->particleSystem();
                sceneSmoke->notifyUpdate();
            });
    
        sceneDevice->setFunctionOnTimeChanged(
            [sceneSmoke](double time){
                sceneSmoke->setTime(time);
                sceneSmoke->notifyUpdate();
            });
            
        return sceneDevice;
    });

}


SmokeDevice::SmokeDevice()
{
    auto& ps = particleSystem_;
    ps.setLifeTime(5.0f);
    ps.setNumParticles(2000);
    ps.setParticleSize(0.06f);
    ps.setEmissionRange(radian(120.0f));
    ps.setAcceleration(Vector3f(0.0f, 0.0f, 0.04f));
}


SmokeDevice::SmokeDevice(const SmokeDevice& org, bool copyStateOnly)
    : Device(org, copyStateOnly),
      particleSystem_(org.particleSystem_)
{

}


const char* SmokeDevice::typeName()
{
    return "SmokeDevice";
}


void SmokeDevice::copyStateFrom(const SmokeDevice& other)
{
    particleSystem_ = other.particleSystem_;
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
    return new SmokeDevice(*this, true);
}


Referenced* SmokeDevice::doClone(CloneMap*) const
{
    return new SmokeDevice(*this);
}


void SmokeDevice::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(SmokeDevice))){
        Device::forEachActualType(func);
    }
}


bool SmokeDevice::on() const
{
    return particleSystem_.on();
}


void SmokeDevice::on(bool on)
{
    if(on && !particleSystem_.on()){
        if(link()){
            particleSystem_.setOffsetTime(-link()->body()->currentTime());
        }
    }
    particleSystem_.on(on);
}



int SmokeDevice::stateSize() const
{
    return 11;
}


const double* SmokeDevice::readState(const double* buf)
{
    int i = 0;
    auto& ps = particleSystem_;

    ps.on(buf[i++]);
    ps.setOffsetTime(buf[i++]);
    ps.setLifeTime(buf[i++]);
    ps.setNumParticles(buf[i++]);
    ps.setParticleSize(buf[i++]);
    ps.setInitialSpeedAverage(buf[i++]);
    ps.setInitialSpeedVariation(buf[i++]);
    ps.setEmissionRange(buf[i++]);
    ps.setAcceleration(Vector3f(buf[i], buf[i+1], buf[i+2]));
    i += 3;
    
    return buf + i;
}


double* SmokeDevice::writeState(double* out_buf) const
{
    int i = 0;
    auto& ps = particleSystem_;
    
    out_buf[i++] = ps.on() ? 1.0 : 0.0;
    out_buf[i++] = ps.offsetTime();
    out_buf[i++] = ps.lifeTime();
    out_buf[i++] = ps.numParticles();
    out_buf[i++] = ps.particleSize();
    out_buf[i++] = ps.initialSpeedAverage();
    out_buf[i++] = ps.initialSpeedVariation();
    out_buf[i++] = ps.emissionRange();
    out_buf[i++] = ps.acceleration()[0];
    out_buf[i++] = ps.acceleration()[1];
    out_buf[i++] = ps.acceleration()[2];

    return out_buf + i;
}
