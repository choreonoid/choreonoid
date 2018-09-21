/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_SCENE_EFFECTS_PLUGIN_FOUNTAIN_DEVICE_H
#define CNOID_SCENE_EFFECTS_PLUGIN_FOUNTAIN_DEVICE_H

#include "ParticleSystem.h"
#include <cnoid/Device>

namespace cnoid {

class FountainDevice : public Device
{
public:
    FountainDevice();
    FountainDevice(const FountainDevice& org, bool copyStateOnly = false);
    virtual const char* typeName() override;
    void copyStateFrom(const FountainDevice& other);
    virtual void copyStateFrom(const DeviceState& other) override;
    virtual DeviceState* cloneState() const override;
    virtual Device* clone() const override;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;
    virtual int stateSize() const override;
    virtual const double* readState(const double* buf) override;
    virtual double* writeState(double* out_buf) const override;

    virtual bool on() const;
    virtual void on(bool on);
        
    ParticleSystem& particleSystem() { return particleSystem_; }
    const ParticleSystem& particleSystem() const { return particleSystem_; }

private:
    ParticleSystem particleSystem_;
};

typedef ref_ptr<FountainDevice> FountainDevicePtr;

}

#endif
