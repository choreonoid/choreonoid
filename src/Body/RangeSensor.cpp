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

    maxYawAngle_ = radian(60.0);
    minYawAngle_ = -maxYawAngle_;
    setYawStep(1.0);

    minPitchAngle_ = 0.0;
    maxPitchAngle_ = 0.0;
    setPitchStep(0.0);
    
    minDistance_ = 0.1;
    maxDistance_ = 10.0;
    
    rangeData_ = std::make_shared<RangeData>();

    spec->detectionRate = 1.0;
    spec->errorDeviation = 0.0;
    spec->isRangeDataStateClonable = false;
}


RangeSensor::RangeSensor(const RangeSensor& org, bool copyStateOnly)
    : VisionSensor(org, copyStateOnly)
{
    if(!copyStateOnly){
        spec = make_unique<Spec>();
        if(org.spec){
            spec->detectionRate = org.spec->detectionRate;
            spec->errorDeviation = org.spec->errorDeviation;
            spec->isRangeDataStateClonable = org.spec->isRangeDataStateClonable;
        } else {
            spec->detectionRate = 1.0;
            spec->errorDeviation = 0.0;
            spec->isRangeDataStateClonable = false;
        }
    }

    copyRangeSensorStateFrom(org, false, org.isRangeDataStateClonable());
}


void RangeSensor::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(RangeSensor)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyRangeSensorStateFrom(static_cast<const RangeSensor&>(other), true, true);
}


void RangeSensor::copyRangeSensorStateFrom(const RangeSensor& other, bool doCopyVisionSensorState, bool doCopyRangeData)
{
    if(doCopyVisionSensorState){
        VisionSensor::copyVisionSensorStateFrom(other);
    }

    minYawAngle_ = other.minYawAngle_;
    maxYawAngle_ = other.maxYawAngle_;
    yawStep_ = other.yawStep_;
    
    minPitchAngle_ = other.minPitchAngle_;
    maxPitchAngle_ = other.maxPitchAngle_;
    pitchStep_ = other.pitchStep_;

    numYawSamples_ = other.numYawSamples_;
    numPitchSamples_ = other.numPitchSamples_;
    isNumYawSamplesCalculatedFromYawStep_ = other.isNumYawSamplesCalculatedFromYawStep_;
    isNumPitchSamplesCalculatedFromPitchStep_ = other.isNumPitchSamplesCalculatedFromPitchStep_;
    
    minDistance_ = other.minDistance_;
    maxDistance_ = other.maxDistance_;

    if(doCopyRangeData){
        if(!other.rangeData_->empty()){
            rangeData_ = other.rangeData_;
        } else {            
            rangeData_ = std::make_shared<RangeData>();
        }
        if(other.sphericalAngleData_ && !other.sphericalAngleData_->empty()){
            sphericalAngleData_ = other.sphericalAngleData_;
        } else {
            sphericalAngleData_.reset();
        }
    } else {
        rangeData_ = std::make_shared<RangeData>();
        sphericalAngleData_.reset();
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


void RangeSensor::setYawRange(double minAngle, double maxAngle)
{
    if(maxAngle >= minAngle){
        minYawAngle_ = minAngle;
        maxYawAngle_ = maxAngle;
        if(isNumYawSamplesCalculatedFromYawStep_){
            setYawStep(yawStep_);
        } else {
            setNumYawSamples(numYawSamples_);
        }
    }
}


void RangeSensor::setYawRange(double angle)
{
    double h = std::abs(angle / 2.0);
    setYawRange(-h, h);
}


void RangeSensor::setNumYawSamples(int n)
{
    if(n >= 1){
        numYawSamples_ = n;
        if(n == 1){
            yawStep_ = 0.0;
        } else {
            yawStep_ = yawRange() / (n - 1);
        }
        isNumYawSamplesCalculatedFromYawStep_ = false;
    }
}


void RangeSensor::setYawStep(double step)
{
    if(step >= 0.0){
        yawStep_ = step;
        if(step == 0.0){
            numYawSamples_ = 1;
        } else {
            numYawSamples_ = static_cast<int>(yawRange() / step + 1.0e-7) + 1;
        }
        isNumYawSamplesCalculatedFromYawStep_ = true;
    }
}


void RangeSensor::setPitchRange(double minAngle, double maxAngle)
{
    if(maxAngle >= minAngle){
        minPitchAngle_ = minAngle;
        maxPitchAngle_ = maxAngle;
        if(isNumPitchSamplesCalculatedFromPitchStep_){
            setPitchStep(pitchStep_);
        } else {
            setNumPitchSamples(numPitchSamples_);
        }
    }
}


void RangeSensor::setPitchRange(double angle)
{
    double h = std::abs(angle / 2.0);
    setPitchRange(-h, h);
}


void RangeSensor::setNumPitchSamples(int n)
{
    if(n >= 1){
        numPitchSamples_ = n;
        if(n == 1){
            pitchStep_ = 0.0;
        } else {
            pitchStep_ = pitchRange() / (n - 1);
        }
        isNumPitchSamplesCalculatedFromPitchStep_ = false;
    }
}


void RangeSensor::setPitchStep(double step)
{
    if(step >= 0.0){
        pitchStep_ = step;
        if(step == 0.0){
            numPitchSamples_ = 1;
        } else {
            numPitchSamples_ = static_cast<int>(pitchRange() / step + 1.0e-7) + 1;
        }
        isNumPitchSamplesCalculatedFromPitchStep_ = true;
    }
}


void RangeSensor::setDetectionRate(double r)
{
    if(spec){
        spec->detectionRate = r;
    }
}


void RangeSensor::setErrorDeviation(double d)
{
    if(spec){
        spec->errorDeviation = d;
    }
}


void RangeSensor::setRangeDataStateClonable(bool on)
{
    if(spec){
        spec->isRangeDataStateClonable = on;
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


void RangeSensor::clearRangeData()
{
    if(rangeData_.use_count() == 1){
        rangeData_->clear();
    } else {
        rangeData_ = std::make_shared<RangeData>();
    }
}


std::vector<Vector2f>& RangeSensor::sphericalAngleData()
{
    if(!sphericalAngleData_ || sphericalAngleData_.use_count() > 1){
        sphericalAngleData_ = std::make_shared<std::vector<Vector2f>>(*sphericalAngleData_);
    }
    return *sphericalAngleData_;
}

std::vector<Vector2f>& RangeSensor::newSphericalAngleData()
{
    sphericalAngleData_ = std::make_shared<std::vector<Vector2f>>();
    return *sphericalAngleData_;
}


void RangeSensor::setSphericalAngleData(std::shared_ptr<std::vector<Vector2f>>& angleData)
{
    if(angleData.use_count() == 1){
        sphericalAngleData_ = angleData;
    } else {
        sphericalAngleData_ = std::make_shared<std::vector<Vector2f>>(*angleData);
    }
    angleData.reset();
}


void RangeSensor::clearState()
{
    clearRangeData();
    clearSphericalAngleData();
}


int RangeSensor::stateSize() const
{
    return VisionSensorStateSize + 6;
}


const double* RangeSensor::readState(const double* buf)
{
    buf = VisionSensor::readState(buf);
    minYawAngle_ = buf[0];
    maxYawAngle_ = buf[1];
    numYawSamples_ = buf[2];
    minPitchAngle_ = buf[3];
    maxPitchAngle_ = buf[4];
    numPitchSamples_ = buf[5];
    minDistance_ = buf[6];
    maxDistance_ = buf[7];
    return buf + 8;
}


double* RangeSensor::writeState(double* out_buf) const
{
    out_buf = VisionSensor::writeState(out_buf);
    out_buf[0] = minYawAngle_;
    out_buf[1] = maxYawAngle_;
    out_buf[2] = numYawSamples_;
    out_buf[3] = minPitchAngle_;
    out_buf[4] = maxPitchAngle_;
    out_buf[5] = numPitchSamples_;
    out_buf[6] = minDistance_;
    out_buf[7] = maxDistance_;
    return out_buf + 8;
}


bool RangeSensor::readSpecifications(const Mapping* info)
{
    if(!VisionSensor::readSpecifications(info)){
        return false;
    }
    
    double minAngle;
    double maxAngle;
    double range;
    if(info->readAngle("min_yaw_angle", minAngle) && info->readAngle("max_yaw_angle", maxAngle)){
        setYawRange(minAngle, maxAngle);
    } else if(info->readAngle({ "yaw_range", "yawRange", "scanAngle" }, range)){
        setYawRange(range);
    }
    if(info->readAngle("min_pitch_angle", minAngle) && info->readAngle("max_pitch_angle", maxAngle)){
        setPitchRange(minAngle, maxAngle);
    } else if(info->readAngle({ "pitch_range", "pitchRange" }, range)){
        setPitchRange(range);
    }

    double step;
    int samples;
    if(info->readAngle({ "yaw_step", "yawStep", "scanStep" }, step)){
        setYawStep(step);
    } else if(info->read("yaw_samples", samples)){
        setNumYawSamples(samples);
    }
    if(info->readAngle({ "pitch_step", "pitchStep" }, step)){
        setPitchStep(step);
    } else if(info->read("pitch_samples", samples)){
        setNumPitchSamples(samples);
    }

    info->read({ "min_distance", "minDistance" }, minDistance_);
    info->read({ "max_distance", "maxDistance" }, maxDistance_);

    double scanRate;
    if(info->read({ "scan_rate", "scanRate" }, scanRate)){
        setFrameRate(scanRate);
    }

    info->read("detection_rate", spec->detectionRate);
    info->read("error_deviation", spec->errorDeviation);

    return true;
}


bool RangeSensor::writeSpecifications(Mapping* info) const
{
    if(!VisionSensor::writeSpecifications(info)){
        return false;
    }
    
    info->write("min_yaw_angle", degree(minYawAngle_));
    info->write("max_yaw_angle", degree(maxYawAngle_));

    if(isNumYawSamplesCalculatedFromYawStep_){
        info->write("yaw_step", yawStep_);
    } else {
        info->write("yaw_samples", numYawSamples_);
    }

    if(minPitchAngle_ != 0.0 || maxPitchAngle_ != 0.0){
        info->write("min_pitch_angle", degree(minPitchAngle_));
        info->write("max_pitch_angle", degree(maxPitchAngle_));
        if(isNumPitchSamplesCalculatedFromPitchStep_){
            info->write("pitch_step", pitchStep_);
        } else {
            info->write("pitch_samples", numPitchSamples_);
        }
    }
    
    info->write("min_distance", minDistance_);
    info->write("max_distance", maxDistance_);
    info->write("scan_rate", scanRate());
    info->write("detection_rate", spec->detectionRate);
    info->write("error_deviation", spec->errorDeviation);

    return true;
}


namespace {

StdBodyFileDeviceTypeRegistration<RangeSensor>
registerRangeSensor(
    "RangeSensor",
     [](StdBodyLoader* loader, const Mapping* info){
         RangeSensorPtr sensor = new RangeSensor;
         if(sensor->readSpecifications(info)){
            return loader->readDevice(sensor, info);
        }
        return false;
    },
    [](StdBodyWriter* /* writer */, Mapping* info, const RangeSensor* sensor){
        return sensor->writeSpecifications(info);
    });
}
