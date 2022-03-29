#include "VisionSensor.h"
#include <cnoid/ValueTree>
#include <cnoid/EigenArchive>

using namespace std;
using namespace cnoid;


VisionSensor::VisionSensor()
    : spec(new Spec)
{
    on_ = true;
    frameRate_ = 30.0;
    delay_ = 0.0;

    spec->R_optical.setIdentity();
}


VisionSensor::VisionSensor(const VisionSensor& org, bool copyStateOnly)
    : Device(org, copyStateOnly)
{
    copyVisionSensorStateFrom(org);

    if(!copyStateOnly){
        spec = make_unique<Spec>();
        if(org.spec){
            spec->R_optical = org.spec->R_optical;
        } else {
            spec->R_optical.setIdentity();
        }
    }
}


void VisionSensor::copyVisionSensorStateFrom(const VisionSensor& other)
{
    on_ = other.on_;
    frameRate_ = other.frameRate_;
    delay_ = other.delay_;
}


void VisionSensor::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(VisionSensor))){
        Device::forEachActualType(func);
    }
}


bool VisionSensor::on() const
{
    return on_;
}


void VisionSensor::on(bool on)
{
    on_ = on;
}


int VisionSensor::visionSensorStateSize()
{
    return 3;
}


const double* VisionSensor::readState(const double* buf)
{
    on_ = buf[0];
    frameRate_ = buf[1];
    delay_ = buf[2];
    return buf + 3;
}


double* VisionSensor::writeState(double* out_buf) const
{
    out_buf[0] = on_ ? 1.0 : 0.0;
    out_buf[1] = frameRate_;
    out_buf[2] = delay_;
    return out_buf + 3;
}


bool VisionSensor::readSpecifications(const Mapping* info)
{
    info->read({ "frame_rate", "frameRate" }, frameRate_);

    Matrix3 R_optical;
    string opticalFrameType;
    bool hasOpticalFrame = false;
    
    if(info->read("optical_frame", opticalFrameType)){
        if(opticalFrameType == "gl"){
            R_optical.setIdentity();
            hasOpticalFrame = true;
        } else if(opticalFrameType == "cv"){
            R_optical <<
                1.0,  0.0,  0.0,
                0.0, -1.0,  0.0,
                0.0,  0.0, -1.0;
            hasOpticalFrame = true;
        }
    };

    AngleAxis aa;
    if(readAngleAxis(info, "optical_frame_rotation", aa)){
        if(hasOpticalFrame){
            R_optical = R_optical * aa;
        } else {
            R_optical = aa.toRotationMatrix();
            hasOpticalFrame = true;
        }
    }

    if(hasOpticalFrame){
        spec->R_optical = R_optical;
    }

    return true;
}


bool VisionSensor::writeSpecifications(Mapping* info) const
{
    info->write("frame_rate", frameRate_);

    if(!spec->R_optical.isIdentity()){
        writeDegreeAngleAxis(info, "optical_frame_rotation", AngleAxis(spec->R_optical));
    }
    
    return true;
}

