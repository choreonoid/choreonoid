/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "Camera.h"
#include "RangeCamera.h"
#include "StdBodyLoader.h"
#include "StdBodyWriter.h"
#include <cnoid/ValueTree>
#include <cnoid/EigenUtil>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;


const char* Camera::typeName() const
{
    return "Camera";
}


Camera::Camera()
{
    on_ = true;
    isImageStateClonable_ = false;
    imageType_ = COLOR_IMAGE;
    lensType_ = NORMAL_LENS;
    resolutionX_ = 640;
    resolutionY_ = 480;
    fieldOfView_ = 0.785398;
    nearClipDistance_ = 0.04;
    farClipDistance_ = 200.0;
    frameRate_ = 30.0;
    delay_ = 0.0;
    image_ = std::make_shared<Image>();
}


Camera::Camera(const Camera& org, bool copyStateOnly)
    : Device(org, copyStateOnly),
      image_(org.image_)
{
    if(copyStateOnly){
        isImageStateClonable_ = true;
    } else {
        isImageStateClonable_ = org.isImageStateClonable_;
    }

    copyCameraStateFrom(org);
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
    copyCameraStateFrom(other);
    image_ = other.image_;
}


void Camera::copyCameraStateFrom(const Camera& other)
{
    on_ = other.on_;
    imageType_ = other.imageType_;
    lensType_ = other.lensType_;
    resolutionX_ = other.resolutionX_;
    resolutionY_ = other.resolutionY_;
    fieldOfView_ = other.fieldOfView_;
    nearClipDistance_ = other.nearClipDistance_;
    farClipDistance_ = other.farClipDistance_;
    frameRate_ = other.frameRate_;
    delay_ = other.delay_;

    if(other.isImageStateClonable_){
        image_ = other.image_;
    } else {
        image_ = std::make_shared<Image>();
    }
}


Referenced* Camera::doClone(CloneMap*) const
{
    return new Camera(*this, false);
}


DeviceState* Camera::cloneState() const
{
    return new Camera(*this, true);
}


void Camera::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(Camera))){
        Device::forEachActualType(func);
    }
}


void Camera::clearState()
{
    clearImage();
}


void Camera::clearImage()
{
    if(image_.use_count() == 1){
        image_->clear();
    } else {
        image_ = std::make_shared<Image>();
    }
}    


bool Camera::on() const
{
    return on_;
}


void Camera::on(bool on)
{
    on_ = on;
}


Image& Camera::image()
{
    if(image_.use_count() > 1){
        image_ = std::make_shared<Image>(*image_);
    }
    return *image_;
}


Image& Camera::newImage()
{
    image_ = std::make_shared<Image>();
    return *image_;
}


void Camera::setImage(std::shared_ptr<Image>& image)
{
    if(image.use_count() == 1){
        image_ = image;
    } else {
        image_ = std::make_shared<Image>(*image);
    }
    image.reset();
}


int Camera::stateSize() const
{
    return 8;
}


const double* Camera::readState(const double* buf)
{
    on_ = buf[0];
    resolutionX_ = buf[1];
    resolutionY_ = buf[2];
    nearClipDistance_ = buf[3];
    farClipDistance_ = buf[4];
    fieldOfView_ = buf[5];
    frameRate_ = buf[6];
    delay_ = buf[7];
    return buf + 8;
}


double* Camera::writeState(double* out_buf) const
{
    out_buf[0] = on_ ? 1.0 : 0.0;
    out_buf[1] = resolutionX_;
    out_buf[2] = resolutionY_;
    out_buf[3] = nearClipDistance_;
    out_buf[4] = farClipDistance_;
    out_buf[5] = fieldOfView_;
    out_buf[6] = frameRate_;
    out_buf[7] = delay_;
    return out_buf + 8;
}


bool Camera::readSpecifications(const Mapping* info)
{
    string symbol;
    
    setImageType(NO_IMAGE);
    if(info->read("format", symbol)){
        if(symbol == "COLOR"){
            setImageType(COLOR_IMAGE);
        }
    }
    if(info->read({ "lens_type", "lensType" }, symbol)){
        if(symbol == "NORMAL"){
            setLensType(NORMAL_LENS);
        } else if(symbol == "FISHEYE"){
            setLensType(FISHEYE_LENS);
        } else if(symbol == "DUAL_FISHEYE"){
            setLensType(DUAL_FISHEYE_LENS);
        }
    }
    info->read("width", resolutionX_);
    info->read("height", resolutionY_);
    info->readAngle({ "field_of_view", "fieldOfView" }, fieldOfView_);
    info->read({ "near_clip_distance", "nearClipDistance" }, nearClipDistance_);
    info->read({ "far_clip_distance", "farClipDistance" }, farClipDistance_);
    info->read({ "frame_rate", "frameRate" }, frameRate_);

    return true;
}


bool Camera::writeSpecifications(Mapping* info) const
{
    if(imageType_ == COLOR_IMAGE){
        info->write("format", "COLOR");
    }
    if(lensType_ == FISHEYE_LENS){
        info->write("lens_type", "FISHEYE");
    } else if(lensType_ == DUAL_FISHEYE_LENS){
        info->write("lens_type", "DUAL_FISHEYE");
    }
    info->write("width", resolutionX_);
    info->write("height", resolutionY_);
    info->write("field_of_view", degree(fieldOfView_));
    info->write("near_clip_distance", nearClipDistance_);
    info->write("far_clip_distance", farClipDistance_);
    info->write("frame_rate", frameRate_);

    return true;
}


namespace {

bool readCamera(StdBodyLoader* loader, const Mapping* info)
{
    CameraPtr camera;
    string format;
    if(info->read("format", format)){
        if(format == "COLOR"){
            camera = new Camera;
            if(!camera->readSpecifications(info)){
                camera.reset();
            }
        } else if(format == "DEPTH" ||
                  format == "COLOR_DEPTH" ||
                  format == "POINT_CLOUD" ||
                  format == "COLOR_POINT_CLOUD"){
            RangeCameraPtr range = new RangeCamera;
            if(range->readSpecifications(info)){
                camera = range;
            }
        }
    }
    bool result = false;
    if(camera){
        result = loader->readDevice(camera, info);
    }
    return result;
}


struct Registration {
    Registration(){
        StdBodyLoader::registerNodeType("Camera", readCamera);
        StdBodyLoader::registerNodeType("CameraDevice", readCamera);
        StdBodyWriter::registerDeviceWriter<Camera>(
            "Camera",
            [](StdBodyWriter* /* writer */, Mapping* info, const Camera* camera){
                return camera->writeSpecifications(info);
            });
    }
} registration;

}
