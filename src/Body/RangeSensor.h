/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_RANGE_SENSOR_H
#define CNOID_BODY_RANGE_SENSOR_H

#include "VisionSensor.h"
#include <boost/shared_ptr.hpp>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT RangeSensor : public VisionSensor
{
public:
    RangeSensor();
    RangeSensor(const RangeSensor& org, bool copyAll = true);

    virtual const char* typeName();
    void copyStateFrom(const RangeSensor& other); 
    virtual void copyStateFrom(const DeviceState& other);
    virtual DeviceState* cloneState() const;
    virtual Device* clone() const;
    virtual void forEachActualType(boost::function<bool(const std::type_info& type)> func);
    virtual void clearState();
    virtual int stateSize() const;
    virtual const double* readState(const double* buf);
    virtual double* writeState(double* out_buf) const;

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
        
    void setRangeDataAsState(bool on) { isRangeDataSetAsState_ = on; }
    bool isRangeDataSetAsState() const { return isRangeDataSetAsState_; }

    /**
       \note You must check if the range data is not empty before accessing the data
    */
    const RangeData& rangeData() const { return *rangeData_; }
    const RangeData& constRangeData() const { return *rangeData_; }
    RangeData& rangeData();
    RangeData& newRangeData();

    boost::shared_ptr<RangeData> sharedRangeData() const { return rangeData_; }

    /**
       Move semantics. If the use_count() of the given shared range data pointer is one,
       the data is moved to the Camera object and the ownership of the given pointer is released.
       Otherwise, the data is copied.
    */
    void setRangeData(boost::shared_ptr<RangeData>& rangeData);

private:
    bool on_;
    bool isRangeDataSetAsState_;
    int yawResolution_;
    int pitchResolution_;
    double yawRange_;
    double pitchRange_;
    double minDistance_;
    double maxDistance_;
    double frameRate_;
    boost::shared_ptr<RangeData> rangeData_;
};

typedef ref_ptr<RangeSensor> RangeSensorPtr;
};

#endif
