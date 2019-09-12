#ifndef CNOID_BODY_HOLDER_DEVICE_H
#define CNOID_BODY_HOLDER_DEVICE_H

#include "Device.h"
#include "exportdecl.h"

namespace cnoid {

class Body;
typedef ref_ptr<Body> BodyPtr;
class YAMLBodyLoader;
class Mapping;

class CNOID_EXPORT HolderDevice : public Device
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    HolderDevice();
    HolderDevice(const HolderDevice& org, bool copyStateOnly = false);
    virtual ~HolderDevice();

    virtual const char* typeName() override;
    void copyHolderDeviceStateFrom(const HolderDevice& other);
    virtual void copyStateFrom(const DeviceState& other) override;
    virtual DeviceState* cloneState() const override;
    virtual Device* clone() const override;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;
    virtual int stateSize() const override;
    virtual const double* readState(const double* buf) override;
    virtual double* writeState(double* out_buf) const override;

    virtual bool on() const override;
    virtual void on(bool on) override;

    std::string attachment() const;
    void setAttachment(const std::string& attachment);
    void clearAttachment();

    bool readDescription(YAMLBodyLoader& loader, Mapping& node);

private:
    Position targetLocalPosition_;
    BodyPtr targetBody_;
    bool on_;

    std::string* attachment_;
};

typedef ref_ptr<HolderDevice> HolderDevicePtr;

}

#endif
