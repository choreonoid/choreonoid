/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_FORCE_SENSOR_H
#define CNOID_BODY_FORCE_SENSOR_H

#include "Device.h"
#include <memory>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ForceSensor : public Device
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ForceSensor();
    ForceSensor(const ForceSensor& org, bool copyStateOnly = false);

    virtual const char* typeName() override;
    void copyStateFrom(const ForceSensor& other);
    virtual void copyStateFrom(const DeviceState& other) override;
    virtual DeviceState* cloneState() const override;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;
    virtual void clearState() override;
    virtual int stateSize() const override;
    virtual const double* readState(const double* buf) override;
    virtual double* writeState(double* out_buf) const override;
        
    const Vector6& F() const { return F_; }
    Vector6& F() { return F_; }

    Vector6::FixedSegmentReturnType<3>::Type f() { return F_.head<3>(); }
    Vector6::ConstFixedSegmentReturnType<3>::Type f() const { return F_.head<3>(); }

    Vector6::FixedSegmentReturnType<3>::Type tau() { return F_.tail<3>(); }
    Vector6::ConstFixedSegmentReturnType<3>::Type tau() const { return F_.tail<3>(); }

    const Vector6& F_max() const { return spec->F_max; }
    Vector6& F_max() { return spec->F_max; }

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    Vector6 F_; // f (linear force) + tau (torque)
        
    struct Spec {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Vector6 F_max;
    };
    std::unique_ptr<Spec> spec;
};

typedef ref_ptr<ForceSensor> ForceSensorPtr;
    
}

#endif
