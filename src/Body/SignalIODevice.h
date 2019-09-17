#ifndef CNOID_BODY_SIGNAL_IO_DEVICE_H
#define CNOID_BODY_SIGNAL_IO_DEVICE_H

#include "Device.h"
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class YAMLBodyLoader;
class Mapping;

class CNOID_EXPORT SignalIODevice : public Device
{
public:
    SignalIODevice();
    virtual ~SignalIODevice();

    virtual const char* typeName() override;
    void copySignalIODeviceStateFrom(const SignalIODevice& other);
    virtual void copyStateFrom(const DeviceState& other) override;
    virtual DeviceState* cloneState() const override;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;
    virtual int stateSize() const override;
    virtual const double* readState(const double* buf) override;
    virtual double* writeState(double* out_buf) const override;

    virtual bool on() const override;
    virtual void on(bool on) override;

    int numBinarySignals() const { return out_.size(); }

    bool out(int index) const { return out_[index]; }
    bool in(int index) const { return in_[index]; }
    void setOut(int index, bool on, bool doNotify = true);
    void setIn(int index, bool on, bool doNotify = true);
    
    SignalProxy<void(bool on)> sigSignalOutput(int index);
    SignalProxy<void(bool on)> sigSignalInput(int index);

    bool readDescription(YAMLBodyLoader& loader, Mapping& node);

protected:
    SignalIODevice(const SignalIODevice& org, bool copyStateOnly, BodyCloneMap* cloneMap);
    virtual Device* doClone(BodyCloneMap* cloneMap) const override;

private:
    std::vector<bool> out_;
    std::vector<bool> in_;
    bool on_;

    class NonState;
    NonState* ns;
};

typedef ref_ptr<SignalIODevice> SignalIODevicePtr;

}

#endif
