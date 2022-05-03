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
    void copyRangeSensorStateFrom(const RangeSensor& other, bool doCopyVisionSensorState); 
    virtual void copyStateFrom(const DeviceState& other) override;
    virtual DeviceState* cloneState() const override;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;
    virtual void clearState() override;

    double yawRange() const { return yawRange_; }
    void setYawRange(double theta) { yawRange_ = theta; }
    double yawStep() const { return yawStep_; }
    void setYawStep(double s) { yawStep_ = s; }
    int numYawSamples() const;

    double pitchRange() const { return pitchRange_; }
    void setPitchRange(double theta) { pitchRange_ = theta; }
    double pitchStep() const { return pitchStep_; }
    void setPitchStep(double s) { pitchStep_ = s; }
    int numPitchSamples() const;

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

    virtual int stateSize() const override;
    virtual const double* readState(const double* buf) override;
    virtual double* writeState(double* out_buf) const override;

    bool readSpecifications(const Mapping* info);
    bool writeSpecifications(Mapping* info) const;

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    std::shared_ptr<RangeData> rangeData_;
    double yawRange_;
    double yawStep_;
    double pitchRange_;
    double pitchStep_;
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
