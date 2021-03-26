/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "RangeSensor.h"
#include "StdBodyFileUtil.h"
#include <cnoid/ValueTree>
#include <cnoid/EigenUtil>

using namespace std;
using namespace cnoid;


const char* RangeSensor::typeName() const
{
    return "RangeSensor";
}


RangeSensor::RangeSensor()
{
    on_ = true;
    isRangeDataStateClonable_ = false;
    yawRange_ = radian(120.0);
    yawStep_ = 1.0;
    pitchRange_ = 0.0;
    pitchStep_ = 0.0;
    minDistance_ = 0.1;
    maxDistance_ = 10.0;
    scanRate_ = 10.0;
    delay_ = 0.0;
    rangeData_ = std::make_shared<RangeData>();
}


RangeSensor::RangeSensor(const RangeSensor& org, bool copyStateOnly)
    : Device(org, copyStateOnly),
      rangeData_(org.rangeData_)
{
    if(copyStateOnly){
        isRangeDataStateClonable_ = true;
    } else {
        isRangeDataStateClonable_ = org.isRangeDataStateClonable_;
    }

    copyRangeSensorStateFrom(org);
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
    yawRange_ = other.yawRange_;
    yawStep_ = other.yawStep_;
    pitchRange_ = other.pitchRange_;
    pitchStep_ = other.pitchStep_;
    minDistance_ = other.minDistance_;
    maxDistance_ = other.maxDistance_;
    scanRate_ = other.scanRate_;
    delay_ = other.delay_;

    if(other.isRangeDataStateClonable_){
        rangeData_ = other.rangeData_;
    } else {
        rangeData_ = std::make_shared<RangeData>();
    }
}


Referenced* RangeSensor::doClone(CloneMap*) const
{
    return new RangeSensor(*this, false);
}


DeviceState* RangeSensor::cloneState() const
{
    return new RangeSensor(*this, true);
}


void RangeSensor::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(RangeSensor))){
        Device::forEachActualType(func);
    }
}


int RangeSensor::numYawSamples() const
{
    if(yawStep_ > 0.0){
        return static_cast<int>(yawRange_ / yawStep_ + 1.0e-7) + 1;
    } else {
        return 1;
    }
}


int RangeSensor::numPitchSamples() const
{
    if(pitchStep_ > 0.0){
        return static_cast<int>(pitchRange_ / pitchStep_ + 1.0e-7) + 1;
    } else {
        return 1;
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
    clearRangeData();
}


void RangeSensor::clearRangeData()
{
    if(rangeData_.use_count() == 1){
        rangeData_->clear();
    } else {
        rangeData_ = std::make_shared<RangeData>();
    }
}


bool RangeSensor::on() const
{
    return on_;
}


void RangeSensor::on(bool on)
{
    on_ = on;
}

int RangeSensor::stateSize() const
{
    return 9;
}


const double* RangeSensor::readState(const double* buf)
{
    on_ = buf[0];
    yawRange_ = buf[1];
    yawStep_ = buf[2];
    pitchRange_ = buf[3];
    pitchStep_ = buf[4];
    minDistance_ = buf[5];
    maxDistance_ = buf[6];
    scanRate_ = buf[7];
    delay_ = buf[8];
    return buf + 9;
}


double* RangeSensor::writeState(double* out_buf) const
{
    out_buf[0] = on_ ? 1.0 : 0.0;
    out_buf[1] = yawRange_;
    out_buf[2] = yawStep_;
    out_buf[3] = pitchRange_;
    out_buf[4] = pitchStep_;
    out_buf[5] = minDistance_;
    out_buf[6] = maxDistance_;
    out_buf[7] = scanRate_;
    out_buf[8] = delay_;
    return out_buf + 9;
}


bool RangeSensor::readSpecifications(const Mapping* info)
{
    info->readAngle({ "yaw_range", "yawRange", "scanAngle" }, yawRange_);
    info->readAngle({ "yaw_step", "yawStep", "scanStep" }, yawStep_);
    info->readAngle({ "pitch_range", "pitchRange" }, pitchRange_);
    info->readAngle({ "pitch_step", "pitchStep" }, pitchStep_);
    info->read({ "min_distance", "minDistance" }, minDistance_);
    info->read({ "max_distance", "maxDistance" }, maxDistance_);
    info->read({ "scan_rate", "scanRate" }, scanRate_);
    info->read("delay", delay_);
    return true;
}


bool RangeSensor::writeSpecifications(Mapping* info) const
{
    info->write("yaw_range", degree(yawRange_));
    info->write("yaw_step", degree(yawStep_));
    info->write("pitch_range", degree(pitchRange_));
    info->write("pitch_step", degree(pitchStep_));
    info->write("min_distance", minDistance_);
    info->write("max_distance", maxDistance_);
    info->write("scan_rate", scanRate_);
    return true;
}


namespace {

StdBodyFileDeviceTypeRegistration<RangeSensor>
registerHolderDevice(
    "RangeSensor",
     [](StdBodyLoader* loader, const Mapping* info){
         RangeSensorPtr sensor = new RangeSensor;
         if(sensor->readSpecifications(info)){
            return loader->readDevice(sensor, info);
        }
        return false;
    },
    [](StdBodyWriter* /* writer */, Mapping* info, const RangeSensor* sensor)
    {
        return sensor->writeSpecifications(info);
    });
}
