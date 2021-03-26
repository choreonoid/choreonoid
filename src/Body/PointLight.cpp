/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "PointLight.h"
#include "StdBodyFileUtil.h"
#include <cnoid/EigenArchive>

using namespace cnoid;

namespace {

const int LightStateSize = Light::lightStateSize();

}

PointLight::PointLight()
{
    constantAttenuation_ = 1.0f;
    linearAttenuation_ = 0.0f;
    quadraticAttenuation_ = 0.0f;
}


const char* PointLight::typeName() const
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


PointLight::PointLight(const PointLight& org, bool copyStateOnly)
    : Light(org, copyStateOnly)
{
    copyStateFrom(org);
}


DeviceState* PointLight::cloneState() const
{
    return new PointLight(*this, true);
}


Referenced* PointLight::doClone(CloneMap*) const
{
    return new PointLight(*this);
}


void PointLight::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(PointLight))){
        Light::forEachActualType(func);
    }
}


int PointLight::pointLightStateSize()
{
    return LightStateSize + 3;
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


bool PointLight::readSpecifications(const Mapping* info)
{
    if(!Light::readSpecifications(info)){
        return false;
    }
    Vector3 a;
    if(read(info, "attenuation", a)){
        constantAttenuation_ = a[0];
        linearAttenuation_ = a[1];
        quadraticAttenuation_ = a[2];
    }
    return true;
}


bool PointLight::writeSpecifications(Mapping* info) const
{
    if(!Light::writeSpecifications(info)){
        return false;
    }
    write(info, "attenuation",
          Vector3(constantAttenuation_, linearAttenuation_, quadraticAttenuation_));
    return true;
}


namespace {

StdBodyFileDeviceTypeRegistration<PointLight>
registerHolderDevice(
    "PointLight",
     [](StdBodyLoader* loader, const Mapping* info){
         PointLightPtr light = new PointLight;
         if(light->readSpecifications(info)){
            return loader->readDevice(light, info);
        }
        return false;
    },
    [](StdBodyWriter* writer, Mapping* info, const PointLight* light)
    {
        return light->writeSpecifications(info);
    });
}
