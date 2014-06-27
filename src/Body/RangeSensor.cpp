/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "RangeSensor.h"
#include <boost/make_shared.hpp>

using namespace std;
using namespace cnoid;

namespace {
const double PI = 3.14159265358979323846;
}


RangeSensor::RangeSensor()
{
    on_ = true;
    isRangeDataSetAsState_ = false;

    yawRange_ = PI / 2.0;
    yawResolution_ = 100;
    
    pitchRange_ = 0.0;
    pitchResolution_ = 1;
    
    minDistance_ = 0.01;
    maxDistance_ = 10.0;

    frameRate_ = 10.0;

    rangeData_ = boost::make_shared<RangeData>();
}


void RangeSensor::copyStateFrom(const RangeSensor& other)
{
    VisionSensor::copyStateFrom(other);
    
    on_ = other.on_;
    yawResolution_ = other.yawResolution_;
    pitchResolution_ = other.pitchResolution_;
    yawRange_ = other.yawRange_;
    pitchRange_ = other.pitchRange_;
    minDistance_ = other.minDistance_;
    maxDistance_ = other.maxDistance_;
    frameRate_ = other.frameRate_;

    if(other.isRangeDataSetAsState_){
        rangeData_ = other.rangeData_;
    } else {
        rangeData_ = boost::make_shared<RangeData>();
    }
}


void RangeSensor::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(RangeSensor)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyStateFrom(static_cast<const RangeSensor&>(other));
}


RangeSensor::RangeSensor(const RangeSensor& org, bool copyAll)
    : VisionSensor(org, copyAll)
{
    isRangeDataSetAsState_ = org.isRangeDataSetAsState_;
    copyStateFrom(org);
}

        
DeviceState* RangeSensor::cloneState() const
{
    return new RangeSensor(*this, false);
}


Device* RangeSensor::clone() const
{
    return new RangeSensor(*this);
}


void RangeSensor::forEachActualType(boost::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(RangeSensor))){
        VisionSensor::forEachActualType(func);
    }
}


void RangeSensor::setYawRange(double angle)
{
    if(angle > 0.0){
        yawRange_ = angle;
    }
}


void RangeSensor::setYawResolution(int n)
{
    if(n >= 1 ){
        yawResolution_ = n;
    }
}


double RangeSensor::yawStep() const
{
    if(yawResolution_ >= 2){
        return yawRange_ / (yawResolution_ - 1);
    } else {
        return 0.0;
    }
}


void RangeSensor::setPitchRange(double angle)
{
    if(angle >= 0.0){
        pitchRange_ = angle;
    }
}


void RangeSensor::setPitchResolution(double n)
{
    if(n >= 1){
        pitchResolution_ = n;
    }
}


double RangeSensor::pitchStep() const
{
    if(pitchResolution_ >= 2){
        return pitchRange_ / (pitchResolution_ - 1);
    } else {
        return 0.0;
    }
}


void RangeSensor::setMaxDistance(double d)
{
    if(d > 0.0){
        if(d > minDistance_){
            maxDistance_ = d;
        } else {
            minDistance_ = d;
        }
    }
}


void RangeSensor::setMinDistance(double d)
{
    if(d > 0.0){
        if(d < maxDistance_){
            minDistance_ = d;
        } else {
            maxDistance_ = d;
        }
    }
}


void RangeSensor::setFrameRate(double r)
{
    if(r > 0.0){
        frameRate_ = r;
    }
}


RangeSensor::RangeData& RangeSensor::rangeData()
{
    if(rangeData_.use_count() > 1){
        rangeData_ = boost::make_shared<RangeData>(*rangeData_);
    }
    return *rangeData_;
}


RangeSensor::RangeData& RangeSensor::newRangeData()
{
    rangeData_ = boost::make_shared<RangeData>();
    return *rangeData_;
}


void RangeSensor::setRangeData(boost::shared_ptr<RangeData>& data)
{
    if(data.use_count() == 1){
        rangeData_ = data;
    } else {
        rangeData_ = boost::make_shared<RangeData>(*data);
    }
    data.reset();
}


void RangeSensor::clearState()
{
    if(rangeData_.use_count() == 1){
        rangeData_->clear();
    } else {
        rangeData_ = boost::make_shared<RangeData>();
    }
}


int RangeSensor::stateSize() const
{
    return 8;
}


const double* RangeSensor::readState(const double* buf)
{
    buf = VisionSensor::readState(buf);
    on_ = buf[0];
    yawRange_ = buf[1];
    yawResolution_ = buf[2];
    pitchRange_ = buf[3];
    pitchResolution_ = buf[4];
    minDistance_ = buf[5];
    maxDistance_ = buf[6];
    frameRate_ = buf[7];
    return buf + 8;
}


double* RangeSensor::writeState(double* out_buf) const
{
    out_buf = VisionSensor::writeState(out_buf);
    out_buf[0] = on_ ? 1.0 : 0.0;
    out_buf[1] = yawRange_;
    out_buf[2] = yawResolution_;
    out_buf[3] = pitchRange_;
    out_buf[4] = pitchResolution_;
    out_buf[5] = minDistance_;
    out_buf[6] = maxDistance_;
    out_buf[7] = frameRate_;
    return out_buf + 8;
}
