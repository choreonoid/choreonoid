/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "RangeSensor.h"

using namespace std;
using namespace cnoid;

namespace {
const double PI = 3.14159265358979323846;
}


const char* RangeSensor::typeName()
{
    return "RangeSensor";
}


RangeSensor::RangeSensor()
{
    on_ = true;
    isRangeDataStateClonable_ = false;
    yawRange_ = PI / 2.0;
    yawResolution_ = 100;
    pitchRange_ = 0.0;
    pitchResolution_ = 1;
    minDistance_ = 0.01;
    maxDistance_ = 10.0;
    frameRate_ = 10.0;
    delay_ = 0.0;
    rangeData_ = std::make_shared<RangeData>();
}


void RangeSensor::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(RangeSensor)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyStateFrom(static_cast<const RangeSensor&>(other));
}


void RangeSensor::copyStateFrom(const RangeSensor& other)
{
    copyRangeSensorStateFrom(other);
    rangeData_ = other.rangeData_;
}


void RangeSensor::copyRangeSensorStateFrom(const RangeSensor& other)
{
    on_ = other.on_;
    isRangeDataStateClonable_ = other.isRangeDataStateClonable_;
    yawResolution_ = other.yawResolution_;
    pitchResolution_ = other.pitchResolution_;
    yawRange_ = other.yawRange_;
    pitchRange_ = other.pitchRange_;
    minDistance_ = other.minDistance_;
    maxDistance_ = other.maxDistance_;
    frameRate_ = other.frameRate_;
    delay_ = other.delay_;
}


RangeSensor::RangeSensor(const RangeSensor& org, bool copyStateOnly)
    : Device(org, copyStateOnly),
      rangeData_(org.rangeData_)
{
    copyRangeSensorStateFrom(org);
}

        
Device* RangeSensor::clone() const
{
    return new RangeSensor(*this);
}


/**
   Used for cloneState()
*/
RangeSensor::RangeSensor(const RangeSensor& org, int x /* dummy */)
    : Device(org, true)
{
    copyRangeSensorStateFrom(org);

    if(org.isRangeDataStateClonable_){
        rangeData_ = org.rangeData_;
    } else {
        rangeData_ = std::make_shared<RangeData>();
    }
}

        
DeviceState* RangeSensor::cloneState() const
{
    return new RangeSensor(*this, false);
}


void RangeSensor::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(RangeSensor))){
        Device::forEachActualType(func);
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
        return yawRange_ / (yawResolution_-1);
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


void RangeSensor::setPitchResolution(int n)
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
        rangeData_ = std::make_shared<RangeData>(*rangeData_);
    }
    return *rangeData_;
}


RangeSensor::RangeData& RangeSensor::newRangeData()
{
    rangeData_ = std::make_shared<RangeData>();
    return *rangeData_;
}


void RangeSensor::setRangeData(std::shared_ptr<RangeData>& data)
{
    if(data.use_count() == 1){
        rangeData_ = data;
    } else {
        rangeData_ = std::make_shared<RangeData>(*data);
    }
    data.reset();
}


void RangeSensor::clearState()
{
    if(rangeData_.use_count() == 1){
        rangeData_->clear();
    } else {
        rangeData_ = std::make_shared<RangeData>();
    }
}


int RangeSensor::stateSize() const
{
    return 9;
}


const double* RangeSensor::readState(const double* buf)
{
    on_ = buf[0];
    yawRange_ = buf[1];
    yawResolution_ = buf[2];
    pitchRange_ = buf[3];
    pitchResolution_ = buf[4];
    minDistance_ = buf[5];
    maxDistance_ = buf[6];
    frameRate_ = buf[7];
    delay_ = buf[8];
    return buf + 9;
}


double* RangeSensor::writeState(double* out_buf) const
{
    out_buf[0] = on_ ? 1.0 : 0.0;
    out_buf[1] = yawRange_;
    out_buf[2] = yawResolution_;
    out_buf[3] = pitchRange_;
    out_buf[4] = pitchResolution_;
    out_buf[5] = minDistance_;
    out_buf[6] = maxDistance_;
    out_buf[7] = frameRate_;
    out_buf[8] = delay_;
    return out_buf + 9;
}
