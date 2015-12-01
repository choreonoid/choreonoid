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
    SpotLight(const SpotLight& org, bool copyState = true);

    virtual const char* typeName();
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
