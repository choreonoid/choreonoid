#ifndef CNOID_BODY_RANGE_SENSOR_H
#define CNOID_BODY_RANGE_SENSOR_H

#include "VisionSensor.h"
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT RangeSensor : public VisionSensor
{
public:
    RangeSensor();
    RangeSensor(const RangeSensor& org, bool copyStateOnly = false);

    virtual const char* typeName() const override;
    virtual void copyStateFrom(const DeviceState& other) override;
    virtual DeviceState* cloneState() const override;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;
    virtual void clearState() override;

    double minYawAngle() const { return minYawAngle_; }
    double maxYawAngle() const { return maxYawAngle_; }
    double yawRange() const { return maxYawAngle_ - minYawAngle_; }
    void setYawRange(double minAngle, double maxAngle);
    void setYawRange(double angle);
    int numYawSamples() const { return numYawSamples_; }
    void setNumYawSamples(int n);
    double yawStep() const { return yawStep_; }
    void setYawStep(double step);
    bool isNumYawSamplesCalculatedFromYawStep() const { return isNumYawSamplesCalculatedFromYawStep_; }
    
    double minPitchAngle() const { return minPitchAngle_; }
    double maxPitchAngle() const { return maxPitchAngle_; }
    double pitchRange() const { return maxPitchAngle_ - minPitchAngle_; }
    void setPitchRange(double minAngle, double maxAngle);
    void setPitchRange(double angle);
    int numPitchSamples() const { return numPitchSamples_; }
    void setNumPitchSamples(int n);
    double pitchStep() const { return pitchStep_; }
    void setPitchStep(double step);
    bool isNumPitchSamplesCalculatedFromPitchStep() const { return isNumPitchSamplesCalculatedFromPitchStep_; }

    double maxDistance() const { return maxDistance_; }
    void setMaxDistance(double d) { maxDistance_ = d; }
    double minDistance() const { return minDistance_; }
    void setMinDistance(double d) { minDistance_ = d; }

    double scanRate() const { return frameRate(); }
    void setScanRate(double r) { setFrameRate(r); }

    double detectionRate() const { return spec ? spec->detectionRate : 1.0; }
    void setDetectionRate(double r);
    double errorDeviation() const { return spec ? spec->errorDeviation : 0.0; }
    void setErrorDeviation(double d);

    typedef std::vector<double> RangeData;

    void setRangeDataStateClonable(bool on);
    bool isRangeDataStateClonable() const { return spec ? spec->isRangeDataStateClonable : true; }

    /**
       \note You must check if the range data is not empty before accessing the data
    */
    const RangeData& rangeData() const { return *rangeData_; }
    const RangeData& constRangeData() const { return *rangeData_; }
    RangeData& rangeData();
    RangeData& newRangeData();

    std::shared_ptr<RangeData> sharedRangeData() const { return rangeData_; }

    /**
       Move semantics. If the use_count() of the given shared range data pointer is one,
       the data is moved to the Camera object and the ownership of the given pointer is released.
       Otherwise, the data is copied.
    */
    void setRangeData(std::shared_ptr<RangeData>& rangeData);

    void clearRangeData();

    bool hasSphericalAngleData() const { return sphericalAngleData_ && !sphericalAngleData_->empty(); }
    const std::vector<Vector2f>& sphericalAngleData() const { return *sphericalAngleData_; }
    const std::vector<Vector2f>& constSpherialAngleData() const { return *sphericalAngleData_; }
    std::vector<Vector2f>& sphericalAngleData();
    std::vector<Vector2f>& newSphericalAngleData();
    std::shared_ptr<std::vector<Vector2f>> sharedSphericalAngleData() const { return sphericalAngleData_; }
    void setSphericalAngleData(std::shared_ptr<std::vector<Vector2f>>& angleData);
    void clearSphericalAngleData() { sphericalAngleData_.reset(); }

    Vector2 getSphericalAngle(int rangeDataIndex) const {
        if(sphericalAngleData_){
            return (*sphericalAngleData_)[rangeDataIndex].cast<double>();
        } else {
            const int yawIndex = rangeDataIndex % numYawSamples_;
            const int pitchIndex = rangeDataIndex / numYawSamples_;
            return Vector2(yawIndex * yawStep_ + minYawAngle_, pitchIndex * pitchStep_ + minPitchAngle_);
        }
    }

    virtual int stateSize() const override;
    virtual const double* readState(const double* buf, int size) override;
    virtual double* writeState(double* out_buf) const override;

    bool readSpecifications(const Mapping* info);
    bool writeSpecifications(Mapping* info) const;

protected:
    void copyRangeSensorStateFrom(const RangeSensor& other, bool doCopyVisionSensorState, bool doCopyRangeData); 
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    std::shared_ptr<RangeData> rangeData_;
    std::shared_ptr<std::vector<Vector2f>> sphericalAngleData_;
    double minYawAngle_;
    double maxYawAngle_;
    double yawStep_;
    double minPitchAngle_;
    double maxPitchAngle_;
    double pitchStep_;
    int numYawSamples_;
    int numPitchSamples_;
    bool isNumYawSamplesCalculatedFromYawStep_;
    bool isNumPitchSamplesCalculatedFromPitchStep_;
    double minDistance_;
    double maxDistance_;

    struct Spec {
        double detectionRate;
        double errorDeviation;
        bool isRangeDataStateClonable;
    };
    std::unique_ptr<Spec> spec;
};

typedef ref_ptr<RangeSensor> RangeSensorPtr;

}

#endif
