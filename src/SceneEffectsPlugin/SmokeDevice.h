/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_SCENE_EFFECTS_PLUGIN_SMOKE_DEVICE_H
#define CNOID_SCENE_EFFECTS_PLUGIN_SMOKE_DEVICE_H

#include "ParticleSystem.h"
#include <cnoid/Device>

namespace cnoid {

class SmokeDevice : public Device
{
public:
    SmokeDevice();
    SmokeDevice(const SmokeDevice& org, bool copyStateOnly = false);
    virtual const char* typeName() override;
    void copyStateFrom(const SmokeDevice& other);
    virtual void copyStateFrom(const DeviceState& other) override;
    virtual DeviceState* cloneState() const override;
    virtual Device* clone() const override;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;
    virtual int stateSize() const override;
    virtual const double* readState(const double* buf) override;
    virtual double* writeState(double* out_buf) const override;

    bool on() const { return on_; }
    void on(bool on) { on_ = on; }

    ParticleSystem& particleSystem() { return particleSystem_; }
    const ParticleSystem& particleSystem() const { return particleSystem_; }
        
private:
    bool on_;
    ParticleSystem particleSystem_;
};

typedef ref_ptr<SmokeDevice> SmokeDevicePtr;

}

#endif
