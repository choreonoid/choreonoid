#ifndef CNOID_BODY_ZMP_DEVICE_H
#define CNOID_BODY_ZMP_DEVICE_H

#include "Device.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ZmpDevice : public Device
{
public:
    ZmpDevice();
    virtual ~ZmpDevice();

    virtual const char* typeName() const override;
    void copyZmpDeviceStateFrom(const ZmpDevice& other);
    virtual void copyStateFrom(const DeviceState& other) override;
    virtual DeviceState* cloneState() const override;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;
    virtual int stateSize() const override;
    virtual const double* readState(const double* buf) override;
    virtual double* writeState(double* out_buf) const override;

    virtual bool on() const override;
    virtual void on(bool on) override;

    const Vector3& zmp() const { return zmp_; };
    void setZmp(const Vector3& zmp) { zmp_ = zmp; }

protected:
    ZmpDevice(const ZmpDevice& org, bool copyStateOnly, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    Vector3 zmp_;
    bool on_;
};

typedef ref_ptr<ZmpDevice> ZmpDevicePtr;

}

#endif
