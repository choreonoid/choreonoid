/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_SPOT_LIGHT_H
#define CNOID_BODY_SPOT_LIGHT_H

#include "PointLight.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT SpotLight : public PointLight
{
public:
    SpotLight();
    SpotLight(const SpotLight& org, bool copyStateOnly = false);

    virtual const char* typeName() const override;
    void copyStateFrom(const SpotLight& other);
    virtual void copyStateFrom(const DeviceState& other) override;
    virtual DeviceState* cloneState() const override;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;
    virtual int stateSize() const override;
    virtual const double* readState(const double* buf) override;
    virtual double* writeState(double* out_buf) const override;

    const Vector3& direction() const { return direction_; }
    void setDirection(const Vector3& direction) { direction_ = direction; }

    float beamWidth() const { return beamWidth_; }
    void setBeamWidth(float beamWidth) { beamWidth_ = beamWidth; }

    float cutOffAngle() const { return cutOffAngle_; }
    void setCutOffAngle(float angle) { cutOffAngle_ = angle; }

    float cutOffExponent() const { return cutOffExponent_; }
    void setCutOffExponent(float e) { cutOffExponent_ = e; }

    bool readSpecifications(const Mapping* info);
    bool writeSpecifications(Mapping* info) const;

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
    
private:
    Vector3 direction_;
    float beamWidth_;
    float cutOffAngle_;
    float cutOffExponent_;
};

typedef ref_ptr<SpotLight> SpotLightPtr;

}

#endif
