/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "RangeCamera.h"
#include <boost/make_shared.hpp>

using namespace cnoid;


RangeCamera::RangeCamera()
{
    setImageType(NO_IMAGE);
    points_ = boost::make_shared<PointData>();
    isOrganized_ = false;
}


const char* RangeCamera::typeName()
{
    return "RangeCamera";
}


void RangeCamera::copyStateFrom(const RangeCamera& other)
{
    Camera::copyStateFrom(other);
    copyRangeCameraStateFrom(other);
}


void RangeCamera::copyRangeCameraStateFrom(const RangeCamera& other)
{
    if(isShotDataSetAsState() || other.isShotDataSetAsState()){
        points_ = other.points_;
    } else {
        points_ = boost::make_shared<PointData>();
    }
    isOrganized_ = other.isOrganized_;
}


void RangeCamera::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(RangeCamera)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyStateFrom(static_cast<const RangeCamera&>(other));
}


RangeCamera::RangeCamera(const RangeCamera& org, bool copyAll)
    : Camera(org, copyAll)
{
    copyRangeCameraStateFrom(org);
}

        
DeviceState* RangeCamera::cloneState() const
{
    return new RangeCamera(*this, false);
}


Device* RangeCamera::clone() const
{
    return new RangeCamera(*this);
}


void RangeCamera::forEachActualType(boost::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(RangeCamera))){
        Camera::forEachActualType(func);
    }
}


RangeCamera::PointData& RangeCamera::points()
{
    if(points_.use_count() > 1){
        points_ = boost::make_shared<PointData>(*points_);
    }
    return *points_;
}


RangeCamera::PointData& RangeCamera::newPoints()
{
    points_ = boost::make_shared<PointData>();
    return *points_;
}


void RangeCamera::setPoints(boost::shared_ptr<PointData>& points)
{
    if(points.use_count() == 1){
        points_ = points;
    } else {
        points_ = boost::make_shared<PointData>(*points);
    }
    points.reset();
}


void RangeCamera::clearState()
{
    Camera::clearState();

    if(points_.use_count() == 1){
        points_->clear();
    } else {
        points_ = boost::make_shared<PointData>();
    }
}


void RangeCamera::setOrganized(bool on)
{
    if(on != isOrganized_){
        clearState();
    }
    isOrganized_ = on;
}
