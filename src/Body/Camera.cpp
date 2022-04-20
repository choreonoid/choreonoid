#include "Camera.h"
#include "RangeCamera.h"
#include "StdBodyLoader.h"
#include "StdBodyWriter.h"
#include <cnoid/ValueTree>
#include <cnoid/EigenUtil>
#include <cnoid/EigenArchive>
#include <fmt/format.h>
#include <cmath>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

const int VisionSensorStateSize = VisionSensor::visionSensorStateSize();

}

const char* Camera::typeName() const
{
    return "Camera";
}


Camera::Camera()
    : spec(new Spec)
{
    imageType_ = COLOR_IMAGE;
    lensType_ = NORMAL_LENS;
    resolutionX_ = 640;
    resolutionY_ = 480;
    fieldOfView_ = 0.785398;
    nearClipDistance_ = 0.04;
    farClipDistance_ = 200.0;
    frameRate_ = 30.0;
    image_ = std::make_shared<Image>();

    spec->isImageStateClonable = false;
}


Camera::Camera(const Camera& org, bool copyStateOnly)
    : VisionSensor(org, copyStateOnly)
{
    if(!copyStateOnly){
        spec = make_unique<Spec>();
        spec->isImageStateClonable = spec ? spec->isImageStateClonable : false;
    }
    copyCameraStateFrom(org, false, org.isImageStateClonable());
}


void Camera::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(Camera)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyCameraStateFrom(static_cast<const Camera&>(other), true, true);
}


void Camera::copyCameraStateFrom(const Camera& other, bool doCopyVisionSensorState, bool doCopyImage)
{
    if(doCopyVisionSensorState){
        VisionSensor::copyVisionSensorStateFrom(other);
    }

    imageType_ = other.imageType_;
    lensType_ = other.lensType_;
    resolutionX_ = other.resolutionX_;
    resolutionY_ = other.resolutionY_;
    fieldOfView_ = other.fieldOfView_;
    nearClipDistance_ = other.nearClipDistance_;
    farClipDistance_ = other.farClipDistance_;

    if(doCopyImage && !other.image_->empty()){
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
        VisionSensor::forEachActualType(func);
    }
}


void Camera::clearState()
{
    clearImage();
}


void Camera::setImageStateClonable(bool on)
{
    if(spec){
        spec->isImageStateClonable = on;
    }
}


double Camera::horizontalFieldOfView() const {
    if (resolutionX() <= resolutionY()) {
        return fieldOfView();
    } else {
        const double vfov = fieldOfView();
        return std::atan2(resolutionX() / resolutionY() * std::tan(vfov), 1.0);
    }
}


// this function is only available after resolution settings
void Camera::setHorizontalFieldOfView(double hfov) {
    if (resolutionX() <= resolutionY()) {
        setFieldOfView(hfov);
    } else {
        const double scale = static_cast<double>(resolutionY()) / static_cast<double>(resolutionX());
        const double vfov = std::atan2(scale * std::tan(hfov), 1.0);
        setFieldOfView(vfov);
    }
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


void Camera::clearImage()
{
    if(image_.use_count() == 1){
        image_->clear();
    } else {
        image_ = std::make_shared<Image>();
    }
}


int Camera::stateSize() const
{
    return VisionSensorStateSize + 5;
}


const double* Camera::readState(const double* buf)
{
    buf = VisionSensor::readState(buf);
    resolutionX_ = buf[0];
    resolutionY_ = buf[1];
    nearClipDistance_ = buf[2];
    farClipDistance_ = buf[3];
    fieldOfView_ = buf[4];
    return buf + 5;
}


double* Camera::writeState(double* out_buf) const
{
    out_buf = VisionSensor::writeState(out_buf);
    out_buf[0] = resolutionX_;
    out_buf[1] = resolutionY_;
    out_buf[2] = nearClipDistance_;
    out_buf[3] = farClipDistance_;
    out_buf[4] = fieldOfView_;
    return out_buf + 5;
}


bool Camera::readSpecifications(const Mapping* info)
{
    if(!VisionSensor::readSpecifications(info)){
        return false;
    }
    
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

    return true;
}


bool Camera::writeSpecifications(Mapping* info) const
{
    if(!VisionSensor::writeSpecifications(info)){
        return false;
    }

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
