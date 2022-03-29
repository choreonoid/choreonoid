#include "RangeSensor.h"
#include "StdBodyFileUtil.h"
#include <cnoid/ValueTree>
#include <cnoid/EigenUtil>
#include <cnoid/EigenArchive>

using namespace std;
using namespace cnoid;

namespace {

const int VisionSensorStateSize = VisionSensor::visionSensorStateSize();

}


const char* RangeSensor::typeName() const
{
    return "RangeSensor";
}


RangeSensor::RangeSensor()
    : spec(new Spec)
{
    setScanRate(10.0);
    
    yawRange_ = radian(120.0);
    yawStep_ = 1.0;
    pitchRange_ = 0.0;
    pitchStep_ = 0.0;
    minDistance_ = 0.1;
    maxDistance_ = 10.0;
    rangeData_ = std::make_shared<RangeData>();

    spec->isRangeDataStateClonable = false;
}


RangeSensor::RangeSensor(const RangeSensor& org, bool copyStateOnly)
    : VisionSensor(org, copyStateOnly)
{
    if(!copyStateOnly){
        spec = make_unique<Spec>();
        if(org.spec){
            spec->isRangeDataStateClonable = org.spec->isRangeDataStateClonable;
        } else {
            spec->isRangeDataStateClonable = false;
        }
    }

    copyRangeSensorStateFrom(org, false);
}


void RangeSensor::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(RangeSensor)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyRangeSensorStateFrom(static_cast<const RangeSensor&>(other), true);
}


void RangeSensor::copyRangeSensorStateFrom(const RangeSensor& other, bool doCopyVisionSensorState)
{
    if(doCopyVisionSensorState){
        VisionSensor::copyVisionSensorStateFrom(other);
    }

    yawRange_ = other.yawRange_;
    yawStep_ = other.yawStep_;
    pitchRange_ = other.pitchRange_;
    pitchStep_ = other.pitchStep_;
    minDistance_ = other.minDistance_;
    maxDistance_ = other.maxDistance_;
    
    if(!other.spec){
        rangeData_ = other.rangeData_;
    } else {
        if(other.spec->isRangeDataStateClonable){
            rangeData_ = other.rangeData_;
        } else {
            rangeData_ = std::make_shared<RangeData>();
        }
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
        VisionSensor::forEachActualType(func);
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


void RangeSensor::setRangeDataStateClonable(bool on)
{
    if(spec){
        spec->isRangeDataStateClonable = on;
    }
}


void RangeSensor::clearRangeData()
{
    if(rangeData_.use_count() == 1){
        rangeData_->clear();
    } else {
        rangeData_ = std::make_shared<RangeData>();
    }
}


int RangeSensor::stateSize() const
{
    return VisionSensorStateSize + 6;
}


const double* RangeSensor::readState(const double* buf)
{
    buf = VisionSensor::readState(buf);
    yawRange_ = buf[0];
    yawStep_ = buf[1];
    pitchRange_ = buf[2];
    pitchStep_ = buf[3];
    minDistance_ = buf[4];
    maxDistance_ = buf[5];
    return buf + 6;
}


double* RangeSensor::writeState(double* out_buf) const
{
    out_buf = VisionSensor::writeState(out_buf);
    out_buf[0] = yawRange_;
    out_buf[1] = yawStep_;
    out_buf[2] = pitchRange_;
    out_buf[3] = pitchStep_;
    out_buf[4] = minDistance_;
    out_buf[5] = maxDistance_;
    return out_buf + 6;
}


bool RangeSensor::readSpecifications(const Mapping* info)
{
    if(!VisionSensor::readSpecifications(info)){
        return false;
    }

    info->readAngle({ "yaw_range", "yawRange", "scanAngle" }, yawRange_);
    info->readAngle({ "yaw_step", "yawStep", "scanStep" }, yawStep_);
    info->readAngle({ "pitch_range", "pitchRange" }, pitchRange_);
    info->readAngle({ "pitch_step", "pitchStep" }, pitchStep_);
    info->read({ "min_distance", "minDistance" }, minDistance_);
    info->read({ "max_distance", "maxDistance" }, maxDistance_);
    info->read({ "scan_rate", "scanRate" }, scanRate_);

    return true;
}


bool RangeSensor::writeSpecifications(Mapping* info) const
{
    if(!VisionSensor::writeSpecifications(info)){
        return false;
    }
    
    info->write("yaw_range", degree(yawRange_));
    info->write("yaw_step", degree(yawStep_));
    info->write("pitch_range", degree(pitchRange_));
    info->write("pitch_step", degree(pitchStep_));
    info->write("min_distance", minDistance_);
    info->write("max_distance", maxDistance_);

    info->remove("frame_rate");
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
