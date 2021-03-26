#ifndef CNOID_BODY_ATTACHMENT_DEVICE_H
#define CNOID_BODY_ATTACHMENT_DEVICE_H

#include "Device.h"
#include "exportdecl.h"

namespace cnoid {

class HolderDevice;
class StdBodyLoader;
class Mapping;

class CNOID_EXPORT AttachmentDevice : public Device
{
public:
    AttachmentDevice();
    virtual ~AttachmentDevice();

    virtual const char* typeName() const override;
    void copyAttachmentDeviceStateFrom(const AttachmentDevice& other);
    virtual void copyStateFrom(const DeviceState& other) override;
    virtual DeviceState* cloneState() const override;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;
    virtual int stateSize() const override;
    virtual const double* readState(const double* buf) override;
    virtual double* writeState(double* out_buf) const override;

    virtual bool on() const override;
    virtual void on(bool on) override;

    HolderDevice* holder();
    bool isAttaching() const;
    void detach();

    std::string category() const;
    void setCategory(const std::string& category);
    void clearCategory();

    bool readSpecifications(const Mapping* info);
    bool writeSpecifications(Mapping* info) const;

protected:
    AttachmentDevice(const AttachmentDevice& org, bool copyStateOnly, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    weak_ref_ptr<HolderDevice> weak_holder;
    bool on_;

    std::string* category_; // Not a state

    friend class HolderDevice;
    void setHolder(HolderDevice* holder);
};

typedef ref_ptr<AttachmentDevice> AttachmentDevicePtr;

}

#endif
