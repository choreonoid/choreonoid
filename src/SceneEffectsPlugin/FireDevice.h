/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_SCENE_EFFECTS_PLUGIN_FIRE_DEVICE_H
#define CNOID_SCENE_EFFECTS_PLUGIN_FIRE_DEVICE_H

#include "FireParticleSystem.h"
#include <cnoid/Device>

namespace cnoid {

class YAMLBodyLoader;
class Mapping;

class FireDevice : public Device
{
public:
    FireDevice();
    FireDevice(const FireDevice& org, bool copyStateOnly = false);
    virtual const char* typeName() override;
    void copyStateFrom(const FireDevice& other);
    virtual void copyStateFrom(const DeviceState& other) override;
    virtual DeviceState* cloneState() const override;
    virtual Device* clone() const override;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;
    virtual int stateSize() const override;
    virtual const double* readState(const double* buf) override;
    virtual double* writeState(double* out_buf) const override;

    bool on() const { return on_; }
    void on(bool on) { on_ = on; }

    FireParticleSystem& particleSystem() { return particleSystem_; }
    const FireParticleSystem& particleSystem() const { return particleSystem_; }
        
private:
    bool on_;
    FireParticleSystem particleSystem_;
};

typedef ref_ptr<FireDevice> FireDevicePtr;

}

#endif
