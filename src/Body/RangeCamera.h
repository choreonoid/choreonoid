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

    virtual const char* typeName() override;
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

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    std::shared_ptr< std::vector<Vector3f> > points_;
    bool isOrganized_;
    bool isDense_;

    void copyRangeCameraStateFrom(const RangeCamera& other);    
};

typedef ref_ptr<RangeCamera> RangeCameraPtr;

}

#endif
