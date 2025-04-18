#include "FountainDevice.h"
#include "SceneFountain.h"
#include "SceneEffectDeviceTypeRegistration.h"
#include <cnoid/Body>
#include <cnoid/MathUtil>

using namespace std;
using namespace cnoid;

namespace {

SceneEffectDeviceTypeRegistration<FountainDevice, SceneFountain> snowDeviceRegistration("FountainDevice");;

}

FountainDevice::FountainDevice()
{
    auto& ps = particleSystem_;
    ps.setLifeTime(3.0f);
    ps.setParticleSize(0.05f);
    ps.setNumParticles(1000);
    ps.setAcceleration(Vector3f(0.0f, 0.0f, -0.2f));
    ps.setEmissionRange(radian(30.0f));
}


FountainDevice::FountainDevice(const FountainDevice& org, bool copyStateOnly)
    : Device(org, copyStateOnly),
      particleSystem_(org.particleSystem_)
{

}


const char* FountainDevice::typeName() const
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
    return new FountainDevice(*this, true);
}


Referenced* FountainDevice::doClone(CloneMap*) const
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
    return 11;
}


const double* FountainDevice::readState(const double* buf, int /* size */)
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


double* FountainDevice::writeState(double* out_buf) const
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
