/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "Lights.h"

using namespace cnoid;

namespace {

const int LightStateSize = 5;

}


Light::Light()
{
    on_ = true;
    color_.setConstant(1.0f);
    intensity_ = 1.0f;
}


void Light::copyStateFrom(const Light& other)
{
    on_ = other.on_;
    color_ = other.color_;
    intensity_ = other.intensity_;
}


Light::Light(const Light& org, bool copyAll)
    : Device(org, copyAll)
{
    copyStateFrom(org);
}


void Light::forEachActualType(boost::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(Light))){
        Device::forEachActualType(func);
    }
}


const double* Light::readState(const double* buf)
{
    on_ = buf[0];
    color_ = Eigen::Map<const Vector3>(buf + 1).cast<float>();
    intensity_ = buf[4];
    return buf + 5;
}


double* Light::writeState(double* out_buf) const
{
    out_buf[0] = on_ ? 1.0 : 0.0;
    out_buf[1] = color_[0];
    out_buf[2] = color_[1];
    out_buf[3] = color_[2];
    out_buf[4] = intensity_;
    return out_buf + 5;
}


PointLight::PointLight()
{
    constantAttenuation_ = 1.0f;
    linearAttenuation_ = 0.0f;
    quadraticAttenuation_ = 0.0f;
}


const char* PointLight::typeName()
{
    return "PointLight";
}


void PointLight::copyStateFrom(const PointLight& other)
{
    Light::copyStateFrom(other);

    constantAttenuation_ = other.constantAttenuation_;
    linearAttenuation_ = other.linearAttenuation_;
    quadraticAttenuation_ = other.quadraticAttenuation_;
}


void PointLight::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(PointLight)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyStateFrom(static_cast<const PointLight&>(other));
}


PointLight::PointLight(const PointLight& org, bool copyAll)
    : Light(org, copyAll)
{
    copyStateFrom(org);
}


DeviceState* PointLight::cloneState() const
{
    return new PointLight(*this, false);
}


Device* PointLight::clone() const
{
    return new PointLight(*this);
}


void PointLight::forEachActualType(boost::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(PointLight))){
        Light::forEachActualType(func);
    }
}


int PointLight::stateSize() const
{
    return LightStateSize + 3;
}


const double* PointLight::readState(const double* buf)
{
    buf = Light::readState(buf);
    constantAttenuation_ = buf[0];
    linearAttenuation_ = buf[1];
    quadraticAttenuation_ = buf[2];
    return buf + 3;
}


double* PointLight::writeState(double* out_buf) const
{
    out_buf = Light::writeState(out_buf);
    out_buf[0] = constantAttenuation_;
    out_buf[1] = linearAttenuation_;
    out_buf[2] = quadraticAttenuation_;
    return out_buf + 3;
}


SpotLight::SpotLight()
{
    direction_ << 0.0f, 0.0f, -1.0f;
    beamWidth_ = 1.570796f;
    cutOffAngle_ = 0.785398f;
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
}


void SpotLight::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(SpotLight)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyStateFrom(static_cast<const SpotLight&>(other));
}


SpotLight::SpotLight(const SpotLight& org, bool copyAll)
    : PointLight(org, copyAll)
{
    copyStateFrom(org);
}


DeviceState* SpotLight::cloneState() const
{
    return new SpotLight(*this, false);
}


Device* SpotLight::clone() const
{
    return new SpotLight(*this);
}


void SpotLight::forEachActualType(boost::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(SpotLight))){
        PointLight::forEachActualType(func);
    }
}


int SpotLight::stateSize() const
{
    return LightStateSize + 5;
}


const double* SpotLight::readState(const double* buf)
{
    buf = Light::readState(buf);
    direction_ = Eigen::Map<const Vector3>(buf);
    beamWidth_ = buf[3];
    cutOffAngle_ = buf[4];
    return buf + 5;
}


double* SpotLight::writeState(double* out_buf) const
{
    out_buf = Light::writeState(out_buf);
    Eigen::Map<Vector3>(out_buf) << direction_;
    out_buf[3] = beamWidth_;
    out_buf[4] = cutOffAngle_;
    return out_buf + 5;
}
