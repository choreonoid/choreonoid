/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "VisionSensor.h"

using namespace cnoid;

VisionSensor::VisionSensor()
{
    delay_ = 0.0;
}


VisionSensor::VisionSensor(const VisionSensor& org, bool copyStateOnly)
    : Device(org, copyStateOnly)
{
    copyStateFrom(org);
}


void VisionSensor::copyStateFrom(const VisionSensor& other)
{
    delay_ = other.delay_;
}


void VisionSensor::forEachActualType(boost::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(VisionSensor))){
        Device::forEachActualType(func);
    }
}


const double* VisionSensor::readState(const double* buf)
{
    delay_ = buf[0];
    return buf + 1;
}


double* VisionSensor::writeState(double* out_buf) const
{
    out_buf[0] = delay_;
    return out_buf + 1;
}
