/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_ACCELERATION_SENSOR_H
#define CNOID_BODY_ACCELERATION_SENSOR_H

#include "Device.h"
#include <memory>
#include "exportdecl.h"

namespace cnoid {

class Mapping;

class CNOID_EXPORT AccelerationSensor : public Device
{
public:
    AccelerationSensor();
    AccelerationSensor(const AccelerationSensor& org, bool copyStateOnly = false);

    virtual const char* typeName() const override;
    void copyStateFrom(const AccelerationSensor& other);
    virtual void copyStateFrom(const DeviceState& other) override;
    virtual DeviceState* cloneState() const override;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;
    virtual void clearState() override;
    virtual int stateSize() const override;
    virtual const double* readState(const double* buf) override;
    virtual double* writeState(double* out_buf) const override;

    const Vector3& dv() const { return dv_; }
    Vector3& dv() { return dv_; }

    const Vector3& dv_max() const { return spec->dv_max; }
    Vector3& dv_max() { return spec->dv_max; }

    bool readSpecifications(const Mapping* info);
    bool writeSpecifications(Mapping* info) const;

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    Vector3 dv_;

    struct Spec {
        Vector3 dv_max;
    };
    std::unique_ptr<Spec> spec;
};

typedef ref_ptr<AccelerationSensor> AccelerationSensorPtr;

}

#endif
