/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_LIGHT_H
#define CNOID_BODY_LIGHT_H

#include "Device.h"
#include "exportdecl.h"

namespace cnoid {

class Mapping;

class CNOID_EXPORT Light : public Device
{
protected:
    Light();
    Light(const Light& org, bool copyStateOnly = false);

public:
    void copyStateFrom(const Light& other);
    using Device::copyStateFrom;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;

    bool on() const override;
    void on(bool on) override;

    const Vector3f& color() const { return color_; }
    void setColor(const Vector3f& c) { color_ = c; }
    void setColor(const Vector3& c) { color_ = c.cast<Vector3f::Scalar>(); }

    float intensity() const { return intensity_; }
    void setIntensity(float intensity) { intensity_ = intensity; }
        
    static int lightStateSize();
    virtual const double* readState(const double* buf) override;
    virtual double* writeState(double* out_buf) const override;

    bool readSpecifications(const Mapping* info);
    bool writeSpecifications(Mapping* info) const;

private:
    Vector3f color_;
    float intensity_;
    bool on_;
};

typedef ref_ptr<Light> LightPtr;

}

#endif
