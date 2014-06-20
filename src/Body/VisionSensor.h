/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_VISION_SENSOR_H
#define CNOID_BODY_VISION_SENSOR_H

#include "Device.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT VisionSensor : public Device
{
protected:
    VisionSensor();
    VisionSensor(const VisionSensor& org, bool copyAll = true);
    
public:
    void copyStateFrom(const VisionSensor& other);

    virtual void forEachActualType(boost::function<bool(const std::type_info& type)> func);
    virtual const double* readState(const double* buf);
    virtual double* writeState(double* out_buf) const;

    /**
       Actual elapsed time [s] after the shooting is started.
    */
    double delay() const { return delay_; }
    
    void setDelay(double time) { delay_ = time; }

private:
    double delay_;
};

typedef ref_ptr<VisionSensor> VisionSensorPtr;

};

#endif
