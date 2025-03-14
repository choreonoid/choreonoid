#ifndef CNOID_SCENE_EFFECTS_PLUGIN_RAIN_SNOW_DEVICE_H
#define CNOID_SCENE_EFFECTS_PLUGIN_RAIN_SNOW_DEVICE_H

#include "ParticleSystem.h"
#include <cnoid/Device>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT RainSnowDevice : public Device
{
public:
    void copyStateFrom(const RainSnowDevice& other);
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;
    virtual int stateSize() const override;
    virtual const double* readState(const double* buf, int size) override;
    virtual double* writeState(double* out_buf) const override;

    virtual bool on() const override;
    virtual void on(bool on) override;
        
    ParticleSystem& particleSystem() { return particleSystem_; }
    const ParticleSystem& particleSystem() const { return particleSystem_; }

protected:
    RainSnowDevice();
    RainSnowDevice(const RainSnowDevice& org, bool copyStateOnly = false);
    
private:
    bool on_;
    ParticleSystem particleSystem_;
};


class RainDevice : public RainSnowDevice
{
public:
    RainDevice();
    RainDevice(const RainDevice& org, bool copyStateOnly = false);
    virtual const char* typeName() const override;
    virtual void copyStateFrom(const DeviceState& other) override;
    virtual DeviceState* cloneState() const override;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
};

typedef ref_ptr<RainDevice> RainDevicePtr;


class SnowDevice : public RainSnowDevice
{
public:
    SnowDevice();
    SnowDevice(const SnowDevice& org, bool copyStateOnly = false);
    virtual const char* typeName() const override;
    virtual void copyStateFrom(const DeviceState& other) override;
    virtual DeviceState* cloneState() const override;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
};

typedef ref_ptr<SnowDevice> SnowDevicePtr;

}

#endif
