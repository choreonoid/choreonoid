/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "FireDevice.h"
#include "SceneFire.h"
#include <cnoid/YAMLBodyLoader>
#include <cnoid/SceneDevice>
#include <cnoid/EigenArchive>
#include <cnoid/Body>

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
    particleSystem_.setLifeTime(5.0f);
    particleSystem_.setAcceleration(Vector3f(0.0f, 0.0f, 0.05f));
    particleSystem_.setNumParticles(200);
    particleSystem_.setParticleSize(0.25f);
    particleSystem_.setEmissionRange(PI / 2.0);
    particleSystem_.setInitialSpeedAverage(0.1f);
    particleSystem_.setInitialSpeedVariation(0.1f);
}


FireDevice::FireDevice(const FireDevice& org, bool copyStateOnly)
    : Device(org, copyStateOnly),
      particleSystem_(org.particleSystem_)
{

}


const char* FireDevice::typeName()
{
    return "FireDevice";
}


void FireDevice::copyStateFrom(const FireDevice& other)
{
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
    return new FireDevice(*this, true);
}


Referenced* FireDevice::doClone(CloneMap*) const
{
    return new FireDevice(*this);
}


void FireDevice::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(FireDevice))){
        Device::forEachActualType(func);
    }
}


bool FireDevice::on() const
{
    return particleSystem_.on();
}


void FireDevice::on(bool on)
{
    if(on && !particleSystem_.on()){
        if(link()){
            particleSystem_.setOffsetTime(-link()->body()->currentTime());
        }
    }
    particleSystem_.on(on);
}


int FireDevice::stateSize() const
{
    return 11;
}


const double* FireDevice::readState(const double* buf)
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


double* FireDevice::writeState(double* out_buf) const
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
