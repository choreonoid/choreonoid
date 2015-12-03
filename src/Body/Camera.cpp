/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "Camera.h"
#include <boost/make_shared.hpp>

using namespace cnoid;


const char* Camera::typeName()
{
    return "Camera";
}


Camera::Camera()
{
    on_ = true;
    imageType_ = COLOR_IMAGE;
    isImageStateClonable_ = false;
    resolutionX_ = 640;
    resolutionY_ = 480;
    fieldOfView_ = 0.785398;
    nearDistance_ = 0.01;
    farDistance_ = 100.0;
    frameRate_ = 30.0;
    image_ = boost::make_shared<Image>();
}


void Camera::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(Camera)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyStateFrom(static_cast<const Camera&>(other));
}


void Camera::copyStateFrom(const Camera& other)
{
    VisionSensor::copyStateFrom(other);
    copyCameraStateFrom(other);
    image_ = other.image_;
}


void Camera::copyCameraStateFrom(const Camera& other)
{
    on_ = other.on_;
    isImageStateClonable_ = other.isImageStateClonable_;
    imageType_ = other.imageType_;
    resolutionX_ = other.resolutionX_;
    resolutionY_ = other.resolutionY_;
    fieldOfView_ = other.fieldOfView_;
    nearDistance_ = other.nearDistance_;
    farDistance_ = other.farDistance_;
    frameRate_ = other.frameRate_;
}


Camera::Camera(const Camera& org, bool copyStateOnly)
    : VisionSensor(org, copyStateOnly),
      image_(org.image_)
{
    copyCameraStateFrom(org);
}


Device* Camera::clone() const
{
    return new Camera(*this, false);
}


/**
   Used for cloneState()
*/
Camera::Camera(const Camera& org, int x /* dummy */)
    : VisionSensor(org, true)
{
    copyCameraStateFrom(org);
    
    if(org.isImageStateClonable_){
        image_ = org.image_;
    } else {
        image_ = boost::make_shared<Image>();
    }
}

        
DeviceState* Camera::cloneState() const
{
    return new Camera(*this, 0);
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
