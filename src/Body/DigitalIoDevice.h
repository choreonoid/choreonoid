#ifndef CNOID_BODY_DIGITAL_IO_DEVICE_H
#define CNOID_BODY_DIGITAL_IO_DEVICE_H

#include "Device.h"
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class Mapping;

class CNOID_EXPORT DigitalIoDevice : public Device
{
public:
    DigitalIoDevice();
    virtual ~DigitalIoDevice();

    virtual const char* typeName() const override;
    void copyDigitalIoDeviceStateFrom(const DigitalIoDevice& other);
    virtual void copyStateFrom(const DeviceState& other) override;
    virtual DeviceState* cloneState() const override;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;
    virtual int stateSize() const override;
    virtual const double* readState(const double* buf) override;
    virtual double* writeState(double* out_buf) const override;

    virtual bool on() const override;
    virtual void on(bool on) override;

    int numSignalLines() const { return out_.size(); }
    void setNumSignalLines(int n);

    bool out(int index) const { return out_[index]; }
    void setOut(int index, bool on, bool doNotify = true);
    bool in(int index) const { return in_[index]; }
    void setIn(int index, bool on, bool doNotify = true);

    const std::string& outLabel(int index) const;
    void setOutLabel(int index, const std::string& label);
    std::vector<std::pair<int, std::string&>> getOutLabels() const;
    const std::string& inLabel(int index) const;
    void setInLabel(int index, const std::string& label);
    std::vector<std::pair<int, std::string&>> getInLabels() const;
    
    SignalProxy<void(bool on)> sigOutput(int index);
    SignalProxy<void(bool on)> sigInput(int index);

    bool readDescription(const Mapping* info);
    bool readSpecifications(const Mapping* info);
    bool readConfiguration(const Mapping* info);
    bool writeDescription(Mapping* info) const;
    bool writeSpecifications(Mapping* info) const;
    bool writeConfiguration(Mapping* info) const;

    // Tentative api. The role of this API will be replaced with the StdActionController device.
    void setInputToDeviceSwitchConnection(int inputIndex, const std::string& deviceName);
    std::vector<std::pair<int, std::string&>> getInputToDeviceSwitchConnections() const;
    void removeInputToDeviceSwitchConnection(int inputIndex);
    void clearInputToDeviceSwitchConnections();
    bool readInputToDeviceSwitchConnections(const Mapping& archive);

protected:
    DigitalIoDevice(const DigitalIoDevice& org, bool copyStateOnly, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    std::vector<bool> out_;
    std::vector<bool> in_;
    bool on_;

    class Impl;
    Impl* impl;
};

typedef ref_ptr<DigitalIoDevice> DigitalIoDevicePtr;

}

#endif
