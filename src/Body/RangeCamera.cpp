/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "RangeCamera.h"

using namespace cnoid;


RangeCamera::RangeCamera()
{
    setImageType(NO_IMAGE);
    points_ = std::make_shared<PointData>();
    setNearClipDistance(0.5);
    setFarClipDistance(4.0);
    isOrganized_ = false;
    isDense_ = false;
}


RangeCamera::RangeCamera(const RangeCamera& org, bool copyStateOnly)
    : Camera(org, copyStateOnly),
      points_(org.points_)
{
    copyRangeCameraStateFrom(org);
}


const char* RangeCamera::typeName()
{
    return "RangeCamera";
}


void RangeCamera::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(RangeCamera)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyStateFrom(static_cast<const RangeCamera&>(other));
}


void RangeCamera::copyStateFrom(const RangeCamera& other)
{
    Camera::copyStateFrom(other);
    copyRangeCameraStateFrom(other);
    points_ = other.points_;
}


void RangeCamera::copyRangeCameraStateFrom(const RangeCamera& other)
{
    if(other.isImageStateClonable()){
        points_ = other.points_;
    } else {
        points_ = std::make_shared<PointData>();
    }

    isOrganized_ = other.isOrganized_;
    isDense_ = other.isDense_;
}


Referenced* RangeCamera::doClone(CloneMap*) const
{
    return new RangeCamera(*this, false);
}


DeviceState* RangeCamera::cloneState() const
{
    return new RangeCamera(*this, true);
}


void RangeCamera::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(RangeCamera))){
        Camera::forEachActualType(func);
    }
}


RangeCamera::PointData& RangeCamera::points()
{
    if(points_.use_count() > 1){
        points_ = std::make_shared<PointData>(*points_);
    }
    return *points_;
}


RangeCamera::PointData& RangeCamera::newPoints()
{
    points_ = std::make_shared<PointData>();
    return *points_;
}


void RangeCamera::setPoints(std::shared_ptr<PointData>& points)
{
    if(points.use_count() == 1){
        points_ = points;
    } else {
        points_ = std::make_shared<PointData>(*points);
    }
    points.reset();
}


void RangeCamera::clearState()
{
    Camera::clearState();

    clearPoints();
}


void RangeCamera::clearPoints()
{
    if(points_.use_count() == 1){
        points_->clear();
    } else {
        points_ = std::make_shared<PointData>();
    }
}    


void RangeCamera::setOrganized(bool on)
{
    if(on != isOrganized_){
        clearState();
    }
    isOrganized_ = on;
}
