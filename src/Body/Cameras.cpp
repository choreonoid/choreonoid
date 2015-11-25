/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "Cameras.h"
#include <boost/make_shared.hpp>

using namespace cnoid;


Camera::Camera()
{
    on_ = true;
    imageType_ = COLOR_IMAGE;
    isShotDataSetAsState_ = false;
    resolutionX_ = 640;
    resolutionY_ = 480;
    fieldOfView_ = 0.785398;
    nearDistance_ = 0.01;
    farDistance_ = 100.0;
    frameRate_ = 30.0;
    image_ = boost::make_shared<Image>();
}


const char* Camera::typeName()
{
    return "Camera";
}


void Camera::copyStateFrom(const Camera& other)
{
    VisionSensor::copyStateFrom(other);
    
    on_ = other.on_;
    imageType_ = other.imageType_;
    resolutionX_ = other.resolutionX_;
    resolutionY_ = other.resolutionY_;
    fieldOfView_ = other.fieldOfView_;
    nearDistance_ = other.nearDistance_;
    farDistance_ = other.farDistance_;
    frameRate_ = other.frameRate_;

    if(other.isShotDataSetAsState_){
        image_ = other.image_;
    } else {
        image_ = boost::make_shared<Image>();
    }
}


void Camera::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(Camera)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyStateFrom(static_cast<const Camera&>(other));
}


Camera::Camera(const Camera& org, bool copyAll)
    : VisionSensor(org, copyAll)
{
    isShotDataSetAsState_ = org.isShotDataSetAsState_;
    copyStateFrom(org);
}

        
DeviceState* Camera::cloneState() const
{
    return new Camera(*this, false);
}


Device* Camera::clone() const
{
    return new Camera(*this);
}


void Camera::forEachActualType(boost::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(Camera))){
        VisionSensor::forEachActualType(func);
    }
}


Image& Camera::image()
{
    if(image_.use_count() > 1){
        image_ = boost::make_shared<Image>(*image_);
    }
    return *image_;
}


Image& Camera::newImage()
{
    image_ = boost::make_shared<Image>();
    return *image_;
}


void Camera::setImage(boost::shared_ptr<Image>& image)
{
    if(image.use_count() == 1){
        image_ = image;
    } else {
        image_ = boost::make_shared<Image>(*image);
    }
    image.reset();
}


void Camera::clearState()
{
    if(image_.use_count() == 1){
        image_->clear();
    } else {
        image_ = boost::make_shared<Image>();
    }
}


int Camera::stateSize() const
{
    return 7 + VisionSensor::StateSize;
}


const double* Camera::readState(const double* buf)
{
    buf = VisionSensor::readState(buf);
    on_ = buf[0];
    resolutionX_ = buf[1];
    resolutionY_ = buf[2];
    nearDistance_ = buf[3];
    farDistance_ = buf[4];
    fieldOfView_ = buf[5];
    frameRate_ = buf[6];
    return buf + 7;
}


double* Camera::writeState(double* out_buf) const
{
    out_buf = VisionSensor::writeState(out_buf);
    out_buf[0] = on_ ? 1.0 : 0.0;
    out_buf[1] = resolutionX_;
    out_buf[2] = resolutionY_;
    out_buf[3] = nearDistance_;
    out_buf[4] = farDistance_;
    out_buf[5] = fieldOfView_;
    out_buf[6] = frameRate_;
    return out_buf + 7;
}


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

    if(other.isShotDataSetAsState()){
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
    if(org.isShotDataSetAsState()){
        points_ = org.points_;
    } else {
        points_ = boost::make_shared<PointData>();
    }
    isOrganized_ = org.isOrganized_;
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
