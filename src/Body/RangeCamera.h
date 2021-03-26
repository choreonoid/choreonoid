/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_RANGE_CAMERA_H
#define CNOID_BODY_RANGE_CAMERA_H

#include "Camera.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT RangeCamera : public Camera
{
public:
    RangeCamera();
    RangeCamera(const RangeCamera& org, bool copyStateOnly = false);

    virtual const char* typeName() const override;
    void copyStateFrom(const RangeCamera& other); 
    virtual void copyStateFrom(const DeviceState& other) override;
    virtual DeviceState* cloneState() const override;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;
    virtual void clearState() override;

    int numPoints() const { return points_->size(); }

    typedef std::vector<Vector3f> PointData;

    const PointData& points() const { return *points_; }
    const PointData& constPoints() const { return *points_; }
    PointData& points();
    PointData& newPoints();

    /**
       The maximum measurable distance of the depth measurement. Note that this parameter
       is different from the far clip distance, which affects the camera image. Note also that
       it does not currently work with the depth image simulation by GLVisionSimulator.
    */
    double maxDistance() const { return maxDistance_; }
    void setMaxDistance(double d) { maxDistance_ = d; }

    /**
       The minimum measurable distance of the depth measurement. The same notes as for
       the maxDistance parameter apply to this parameter.
    */
    double minDistance() const { return minDistance_; }
    void setMinDistance(double d) { minDistance_ = d; }
            
    bool isOrganized() const { return isOrganized_; }
    void setOrganized(bool on);

    bool isDense() const { return isDense_; }
    void setDense(bool on) { isDense_ = on; }

    std::shared_ptr<const PointData> sharedPoints() const { return points_; }

    /**
       Move semantics. If the use_count() of the given shared point data pointer is one,
       the data is moved to the Camera object and the ownership of the given pointer is released.
       Otherwise, the data is copied.
    */
    void setPoints(std::shared_ptr<PointData>& points);

    void clearPoints();

    bool readSpecifications(const Mapping* info);
    bool writeSpecifications(Mapping* info) const;

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    std::shared_ptr<std::vector<Vector3f>> points_;
    double minDistance_;
    double maxDistance_;
    bool isOrganized_;
    bool isDense_;

    void copyRangeCameraStateFrom(const RangeCamera& other);    
};

typedef ref_ptr<RangeCamera> RangeCameraPtr;

}

#endif
