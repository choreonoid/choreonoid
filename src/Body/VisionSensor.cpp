/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "VisionSensor.h"

using namespace cnoid;

void VisionSensor::forEachActualType(boost::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(VisionSensor))){
        Device::forEachActualType(func);
    }
}
