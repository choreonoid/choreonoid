/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "Light.h"
#include <cnoid/EigenArchive>

using namespace cnoid;


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


Light::Light(const Light& org, bool copyStateOnly)
    : Device(org, copyStateOnly)
{
    copyStateFrom(org);
}


void Light::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(Light))){
        Device::forEachActualType(func);
    }
}


bool Light::on() const
{
    return on_;
}


void Light::on(bool on)
{
    on_ = on;
}


int Light::lightStateSize()
{
    return 5;
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


bool Light::readSpecifications(const Mapping* info)
{
    read(info, "color", color_);
    info->read("intensity", intensity_);
    return true;
}


bool Light::writeSpecifications(Mapping* info) const
{
    write(info, "color", color_);
    info->write("intensity", intensity_);
    return true;
}
