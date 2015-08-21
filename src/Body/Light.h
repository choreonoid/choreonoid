/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_LIGHT_H
#define CNOID_BODY_LIGHT_H

#include "Device.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT Light : public ActiveDevice
{
protected:
    Light();
    Light(const Light& org, bool copyState = true);

public:
    void copyStateFrom(const Light& other);

    virtual void forEachActualType(boost::function<bool(const std::type_info& type)> func);
    virtual const double* readState(const double* buf);
    virtual double* writeState(double* out_buf) const;

    bool on() const { return on_; }
    void on(bool on) { on_ = on; }

    const Vector3f& color() const { return color_; }
    void setColor(const Vector3f& c) { color_ = c; }

    float intensity() const { return intensity_; }
    void setIntensity(float intensity) { intensity_ = intensity; }
        
private:
    Vector3f color_;
    float intensity_;
    bool on_;
};

typedef ref_ptr<Light> LightPtr;


class CNOID_EXPORT PointLight : public Light
{
public:
    PointLight();
    PointLight(const PointLight& org, bool copyState = true);

    void copyStateFrom(const PointLight& other);
    virtual void copyStateFrom(const DeviceState& other);
    virtual DeviceState* cloneState() const;
    virtual Device* clone() const;
    virtual void forEachActualType(boost::function<bool(const std::type_info& type)> func);
    virtual int stateSize() const;
    virtual const double* readState(const double* buf);
    virtual double* writeState(double* out_buf) const;

    float constantAttenuation() const { return constantAttenuation_; }
    void setConstantAttenuation(float a) { constantAttenuation_ = a; }

    float linearAttenuation() const { return linearAttenuation_; }
    void setLinearAttenuation(float a) { linearAttenuation_ = a; }

    float quadraticAttenuation() const { return quadraticAttenuation_; }
    void setQuadraticAttenuation(float a) { quadraticAttenuation_ = a; }
        
private:
    float constantAttenuation_;
    float linearAttenuation_;
    float quadraticAttenuation_;
};

typedef ref_ptr<PointLight> PointLightPtr;
    

class CNOID_EXPORT SpotLight : public PointLight
{
public:
    SpotLight();
    SpotLight(const SpotLight& org, bool copyState = true);

    void copyStateFrom(const SpotLight& other);
    virtual void copyStateFrom(const DeviceState& other);
    virtual DeviceState* cloneState() const;
    virtual Device* clone() const;
    virtual void forEachActualType(boost::function<bool(const std::type_info& type)> func);
    virtual int stateSize() const;
    virtual const double* readState(const double* buf);
    virtual double* writeState(double* out_buf) const;

    const Vector3& direction() const { return direction_; }
    void setDirection(const Vector3& direction) { direction_ = direction; }

    float beamWidth() const { return beamWidth_; }
    void setBeamWidth(float beamWidth) { beamWidth_ = beamWidth; }

    float cutOffAngle() const { return cutOffAngle_; }
    void setCutOffAngle(float angle) { cutOffAngle_ = angle; }
        
private:
    Vector3 direction_;
    float beamWidth_;
    float cutOffAngle_;
};

typedef ref_ptr<SpotLight> SpotLightPtr;

}

#endif
