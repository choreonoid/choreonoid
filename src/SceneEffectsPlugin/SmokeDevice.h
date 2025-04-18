#ifndef CNOID_SCENE_EFFECTS_PLUGIN_SMOKE_DEVICE_H
#define CNOID_SCENE_EFFECTS_PLUGIN_SMOKE_DEVICE_H

#include "ParticleSystem.h"
#include <cnoid/Device>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT SmokeDevice : public Device
{
public:
    SmokeDevice();
    SmokeDevice(const SmokeDevice& org, bool copyStateOnly = false);
    virtual const char* typeName() const override;
    void copyStateFrom(const SmokeDevice& other);
    virtual void copyStateFrom(const DeviceState& other) override;
    virtual DeviceState* cloneState() const override;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;
    virtual int stateSize() const override;
    virtual const double* readState(const double* buf, int size) override;
    virtual double* writeState(double* out_buf) const override;

    virtual bool on() const override;
    virtual void on(bool on) override;

    ParticleSystem& particleSystem() { return particleSystem_; }
    const ParticleSystem& particleSystem() const { return particleSystem_; }

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
        
private:
    ParticleSystem particleSystem_;
};

typedef ref_ptr<SmokeDevice> SmokeDevicePtr;

}

#endif
