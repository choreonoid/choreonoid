/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_VISION_SENSOR_H_INCLUDED
#define CNOID_BODY_VISION_SENSOR_H_INCLUDED

#include "Device.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT VisionSensor : public Device
{
protected:
    VisionSensor() { }
    VisionSensor(const VisionSensor& org, bool copyAll = true) : Device(org, copyAll) { }
public:
    virtual void forEachActualType(boost::function<bool(const std::type_info& type)> func);
};

typedef ref_ptr<VisionSensor> VisionSensorPtr;
};

#endif
