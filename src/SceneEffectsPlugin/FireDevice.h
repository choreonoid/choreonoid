/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_PHENOMENON_PLUGIN_FIRE_DEVICE_H
#define CNOID_PHENOMENON_PLUGIN_FIRE_DEVICE_H

#include <cnoid/Device>

namespace cnoid {

class FireDevice : public Device
{
public:
    FireDevice();
    FireDevice(const FireDevice& org, bool copyStateOnly = false);
    virtual const char* typeName() override;
    void copyStateFrom(const FireDevice& other);
    virtual void copyStateFrom(const DeviceState& other) override;
    virtual DeviceState* cloneState() const override;
    virtual Device* clone() const override;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;
    virtual int stateSize() const override;
    virtual const double* readState(const double* buf) override;
    virtual double* writeState(double* out_buf) const override;

    bool on() const { return on_; }
    void on(bool on) { on_ = on; }
    void setAcceleration(const Vector3f& a) { acceleration_ = a; }
    const Vector3f& acceleration() const { return acceleration_; }
    void setInitialSpeedAverage(float average){ initialSpeedAverage_ = average; }
    void setInitialSpeedVariation(float variation){ initialSpeedVariation_ = variation; }
    void setInitialVelocityAngleRange(float angle){ initialVelocityAngleRange_ = angle; }
    float initialSpeedAverage() const { return initialSpeedAverage_; }
    float initialSpeedVariation() const { return initialSpeedVariation_; }
    float initialVelocityAngleRange() const { return initialVelocityAngleRange_; }
        
private:
    bool on_;
    Vector3f acceleration_;
    float initialSpeedAverage_;
    float initialSpeedVariation_;
    float initialVelocityAngleRange_;
};

typedef ref_ptr<FireDevice> FireDevicePtr;

}

#endif
