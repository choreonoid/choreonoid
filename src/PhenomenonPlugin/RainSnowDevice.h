/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_PHENOMENON_PLUGIN_RAIN_SNOW_DEVICE_H
#define CNOID_PHENOMENON_PLUGIN_RAIN_SNOW_DEVICE_H

#include <cnoid/Device>

namespace cnoid {

class RainDevice : public Device
{
public:
    RainDevice();
    RainDevice(const RainDevice& org, bool copyStateOnly = false);
    virtual const char* typeName() override;
    void copyStateFrom(const RainDevice& other);
    virtual void copyStateFrom(const DeviceState& other) override;
    virtual DeviceState* cloneState() const override;
    virtual Device* clone() const override;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;
    virtual int stateSize() const override;
    virtual const double* readState(const double* buf) override;
    virtual double* writeState(double* out_buf) const override;

    bool on() const { return on_; }
    void on(bool on) { on_ = on; }
        
private:
    bool on_;
};


class SnowDevice : public Device
{
public:
    SnowDevice();
    SnowDevice(const SnowDevice& org, bool copyStateOnly = false);
    virtual const char* typeName() override;
    void copyStateFrom(const SnowDevice& other);
    virtual void copyStateFrom(const DeviceState& other) override;
    virtual DeviceState* cloneState() const override;
    virtual Device* clone() const override;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;
    virtual int stateSize() const override;
    virtual const double* readState(const double* buf) override;
    virtual double* writeState(double* out_buf) const override;

    bool on() const { return on_; }
    void on(bool on) { on_ = on; }
        
private:
    bool on_;
};

typedef ref_ptr<SnowDevice> SnowDevicePtr;

}

#endif
