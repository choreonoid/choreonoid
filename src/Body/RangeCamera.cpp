/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "RangeCamera.h"
#include "StdBodyFileUtil.h"
#include <cnoid/ValueTree>

using namespace std;
using namespace cnoid;


RangeCamera::RangeCamera()
{
    setImageType(NO_IMAGE);
    points_ = std::make_shared<PointData>();
    maxDistance_ = 10.0;
    minDistance_ = 0.1;
    isOrganized_ = false;
    isDense_ = false;
}


RangeCamera::RangeCamera(const RangeCamera& org, bool copyStateOnly)
    : Camera(org, copyStateOnly)
{
    copyRangeCameraStateFrom(org);
}


const char* RangeCamera::typeName() const
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

    maxDistance_ = other.maxDistance_;
    minDistance_ = other.minDistance_;
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


bool RangeCamera::readSpecifications(const Mapping* info)
{
    if(!Camera::readSpecifications(info)){
        return false;
    }

    string format;
    if(!info->read("format", format)){
        format = "DEPTH";
    }
    if(format == "DEPTH"){
        setOrganized(true);
        setImageType(Camera::NO_IMAGE);
    } else if(format == "COLOR_DEPTH"){
        setOrganized(true);
        setImageType(Camera::COLOR_IMAGE);
    } else if(format == "POINT_CLOUD"){
        setOrganized(false);
        setImageType(Camera::NO_IMAGE);
    } else if(format == "COLOR_POINT_CLOUD"){
        setOrganized(false);
        setImageType(Camera::COLOR_IMAGE);
    } else {
        return false;
    }

    info->read("min_distance", minDistance_);
    info->read("max_distance", maxDistance_);

    return true;
}
        
        
bool RangeCamera::writeSpecifications(Mapping* info) const
{
    if(!Camera::writeSpecifications(info)){
        return false;
    }
    
    if(imageType() == Camera::COLOR_IMAGE){
        if(isOrganized()){
            info->write("format", "COLOR_DEPTH");
        } else {
            info->write("format", "COLOR_POINT_CLOUD");
        }
    } else {
        if(isOrganized()){
            info->write("format", "DEPTH");
        } else {
            info->write("format", "POINT_CLOUD");
        }
    }
    info->write("min_distance", minDistance_);
    info->write("max_distance", maxDistance_);

    return true;
}


namespace {

StdBodyFileDeviceTypeRegistration<RangeCamera>
registerHolderDevice(
    "Camera",
    nullptr,
    [](StdBodyWriter* writer, Mapping* info, const RangeCamera* camera)
    {
        return camera->writeSpecifications(info);
    });
}
