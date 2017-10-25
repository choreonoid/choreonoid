/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_RANGE_SENSOR_H
#define CNOID_BODY_RANGE_SENSOR_H

#include "Device.h"
#include <vector>
#include <memory>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT RangeSensor : public Device
{
public:
    RangeSensor();
    RangeSensor(const RangeSensor& org, bool copyStateOnly = false);

    virtual const char* typeName() override;
    void copyStateFrom(const RangeSensor& other); 
    virtual void copyStateFrom(const DeviceState& other) override;
    virtual DeviceState* cloneState() const override;
    virtual Device* clone() const override;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;
    virtual void clearState() override;
    virtual int stateSize() const override;
    virtual const double* readState(const double* buf) override;
    virtual double* writeState(double* out_buf) const override;

    bool on() const { return on_; }
    void on(bool on) { on_ = on; }

    double yawRange() const { return yawRange_; }
    void setYawRange(double angle);
    int yawResolution() const { return (yawRange_ == 0.0) ? 1 : yawResolution_; }
    void setYawResolution(int n);
    double yawStep() const;

    double pitchRange() const { return pitchRange_; }
    void setPitchRange(double angle);
    int pitchResolution() const { return (pitchRange_ == 0.0) ? 1 : pitchResolution_; }
    void setPitchResolution(double n);
    double pitchStep() const;

    double maxDistance() const { return maxDistance_; }
    void setMaxDistance(double d);
    double minDistance() const { return minDistance_; }
    void setMinDistance(double d);

    double frameRate() const { return frameRate_; }
    void setFrameRate(double r);

    typedef std::vector<double> RangeData;

    void setRangeDataStateClonable(bool on) { isRangeDataStateClonable_ = on; }
    bool isRangeDataStateClonable() const { return isRangeDataStateClonable_; }

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

    /**
       Time [s] consumed in the measurement
    */
    double delay() const { return delay_; }
    void setDelay(double time) { delay_ = time; }

private:
    bool on_;
    bool isRangeDataStateClonable_;
    int yawResolution_;
    int pitchResolution_;
    double yawRange_;
    double pitchRange_;
    double minDistance_;
    double maxDistance_;
    double frameRate_;
    double delay_;
    std::shared_ptr<RangeData> rangeData_;

    RangeSensor(const RangeSensor& org, int x /* dummy */);
    void copyRangeSensorStateFrom(const RangeSensor& other);    
};

typedef ref_ptr<RangeSensor> RangeSensorPtr;

}

#endif
