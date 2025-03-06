#ifndef CNOID_BODY_CONVEYOR_DEVICE_H
#define CNOID_BODY_CONVEYOR_DEVICE_H

#include "Device.h"
#include "exportdecl.h"

namespace cnoid {

class HolderDevice;
class StdBodyLoader;
class Mapping;

/**
   The conveyor device makes its owner link behave as a conveyor.
   Only one conveyor device can be associated with a link, and the joint type of the link must be the fixed link.
   The q parameter of the link stores the displacement of the conveyor belt.
*/
class CNOID_EXPORT ConveyorDevice : public Device
{
public:
    ConveyorDevice();
    virtual ~ConveyorDevice();

    virtual const char* typeName() const override;
    void copyConveyorDeviceStateFrom(const ConveyorDevice& other);
    virtual void copyStateFrom(const DeviceState& other) override;
    virtual DeviceState* cloneState() const override;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;
    virtual int stateSize() const override;
    virtual const double* readState(const double* buf, int size) override;
    virtual double* writeState(double* out_buf) const override;

    virtual bool on() const override;
    virtual void on(bool on) override;

    enum ConveyorType { LinearConveyor, RotaryConveyor };
    
    int conveyorType() const { return conveyorType_; }
    void setConveyorType(int type) { conveyorType_ = type; }
    bool isLinearConveyor() const { return conveyorType_ == LinearConveyor; }
    bool isRotaryConveyor() const { return conveyorType_ == RotaryConveyor; }
    const Vector3& axis() const { return axis_; }
    void setAxis(const Vector3& a) { axis_ = a; }

    double displacement() const { return displacement_; }
    void setDisplacement(double d) { displacement_ = d; }

    double speed() const { return speed_; }
    void setSpeed(double s) { speed_ = s; }

    bool readSpecifications(const Mapping* info);
    bool writeSpecifications(Mapping* info) const;

protected:
    ConveyorDevice(const ConveyorDevice& org, bool copyStateOnly, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    bool on_;
    double displacement_;
    double speed_;

    // Non-state members
    int conveyorType_;
    Vector3 axis_;
};

typedef ref_ptr<ConveyorDevice> ConveyorDevicePtr;

}

#endif
