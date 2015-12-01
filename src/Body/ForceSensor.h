/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_FORCE_SENSOR_H
#define CNOID_BODY_FORCE_SENSOR_H

#include "Device.h"
#include <boost/scoped_ptr.hpp>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ForceSensor : public Device
{
    Vector6 F_; // f (linear force) + tau (torque)
        
    struct Spec {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Vector6 F_max;
    };
    boost::scoped_ptr<Spec> spec;
        
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    ForceSensor();
    ForceSensor(const ForceSensor& org, bool copyAll = true);

    virtual const char* typeName();
    void copyStateFrom(const ForceSensor& other);
    virtual void copyStateFrom(const DeviceState& other);
    virtual DeviceState* cloneState() const;
    virtual Device* clone() const;
    virtual void forEachActualType(boost::function<bool(const std::type_info& type)> func);
    virtual void clearState();
    virtual int stateSize() const;
    virtual const double* readState(const double* buf);
    virtual double* writeState(double* out_buf) const;
        
    const Vector6& F() const { return F_; }
    Vector6& F() { return F_; }

    Vector6::FixedSegmentReturnType<3>::Type f() { return F_.head<3>(); }
    Vector6::ConstFixedSegmentReturnType<3>::Type f() const { return F_.head<3>(); }

    Vector6::FixedSegmentReturnType<3>::Type tau() { return F_.tail<3>(); }
    Vector6::ConstFixedSegmentReturnType<3>::Type tau() const { return F_.tail<3>(); }

    const Vector6& F_max() const { return spec->F_max; }
    Vector6& F_max() { return spec->F_max; }
};

typedef ref_ptr<ForceSensor> ForceSensorPtr;
    
};

#endif
