/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "SpotLight.h"

using namespace cnoid;

namespace {

const int PointLightStateSize = PointLight::pointLightStateSize();

}


SpotLight::SpotLight()
{
    direction_ << 0.0f, 0.0f, -1.0f;
    beamWidth_ = 1.570796f;
    cutOffAngle_ = 0.785398f;
    cutOffExponent_ = 1.0f;
}


const char* SpotLight::typeName()
{
    return "SpotLight";
}


void SpotLight::copyStateFrom(const SpotLight& other)
{
    PointLight::copyStateFrom(other);
    direction_ = other.direction_;
    beamWidth_ = other.beamWidth_;
    cutOffAngle_ = other.cutOffAngle_;
    cutOffExponent_ = other.cutOffExponent_;
}


void SpotLight::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(SpotLight)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyStateFrom(static_cast<const SpotLight&>(other));
}


SpotLight::SpotLight(const SpotLight& org, bool copyStateOnly)
    : PointLight(org, copyStateOnly)
{
    copyStateFrom(org);
}


DeviceState* SpotLight::cloneState() const
{
    return new SpotLight(*this, true);
}


Referenced* SpotLight::doClone(CloneMap*) const
{
    return new SpotLight(*this);
}


void SpotLight::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(SpotLight))){
        PointLight::forEachActualType(func);
    }
}


int SpotLight::stateSize() const
{
    return PointLightStateSize + 6;
}


const double* SpotLight::readState(const double* buf)
{
    buf = PointLight::readState(buf);
    direction_ = Eigen::Map<const Vector3>(buf);
    beamWidth_ = buf[3];
    cutOffAngle_ = buf[4];
    cutOffExponent_ = buf[5];
    return buf + 6;
}


double* SpotLight::writeState(double* out_buf) const
{
    out_buf = PointLight::writeState(out_buf);
    Eigen::Map<Vector3>(out_buf) << direction_;
    out_buf[3] = beamWidth_;
    out_buf[4] = cutOffAngle_;
    out_buf[5] = cutOffExponent_;
    return out_buf + 6;
}
