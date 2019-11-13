/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_CAMERA_H
#define CNOID_BODY_CAMERA_H

#include "Device.h"
#include <cnoid/Image>
#include <memory>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT Camera : public Device
{
public:
    Camera();
    Camera(const Camera& org, bool copyStateOnly = false);

    virtual const char* typeName() override;
    void copyStateFrom(const Camera& other);
    virtual void copyStateFrom(const DeviceState& other) override;
    virtual DeviceState* cloneState() const override;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;
    virtual void clearState() override;

    void setImageStateClonable(bool on) { isImageStateClonable_ = on; }
    bool isImageStateClonable() const { return isImageStateClonable_; }

    enum ImageType { NO_IMAGE, COLOR_IMAGE, GRAYSCALE_IMAGE };
    enum LensType { NORMAL_LENS, FISHEYE_LENS, DUAL_FISHEYE_LENS };

    ImageType imageType() const { return imageType_; }
    void setImageType(ImageType type) { imageType_ = type; }

    LensType lensType() const { return lensType_; }
    void setLensType(LensType type) { lensType_ = type; }

    virtual bool on() const override;
    virtual void on(bool on) override;

    double nearClipDistance() const { return nearClipDistance_; }
    void setNearClipDistance(double d) { nearClipDistance_ = d; }
    double farClipDistance() const { return farClipDistance_; }
    void setFarClipDistance(double d) { farClipDistance_ = d; }

#ifdef CNOID_BACKWARD_COMPATIBILITY
    double nearDistance() const { return nearClipDistance_; }
    void setNearDistance(double d) { nearClipDistance_ = d; }
    double farDistance() const { return farClipDistance_; }
    void setFarDistance(double d) { farClipDistance_ = d; }
#endif
        
    double fieldOfView() const { return fieldOfView_; }
    void setFieldOfView(double f) { fieldOfView_ = f; }

    void setResolution(int x, int y) {
        resolutionX_ = x; resolutionY_ = y;
    }
    void setResolutionX(int x) { resolutionX_ = x; }
    void setResolutionY(int y) { resolutionY_ = y; }
    
    int resolutionX() const { return resolutionX_; }
    int resolutionY() const { return resolutionY_; }

    void setFrameRate(double r) { frameRate_ = r; }
    double frameRate() const { return frameRate_; }

    const Image& image() const;
    const Image& constImage() const { return *image_; }
    Image& image();
    Image& newImage();

    std::shared_ptr<const Image> sharedImage() const { return image_; }

    /**
       Move semantics. If the use_count() of the given shared image pointer is one,
       the data is moved to the Camera object and the ownership of the given pointer is released.
       Otherwise, the data is copied.
    */
    void setImage(std::shared_ptr<Image>& image);

    void clearImage();

    /**
       Time [s] consumed in shooting the current image
    */
    double delay() const { return delay_; }
    void setDelay(double time) { delay_ = time; }

    virtual int stateSize() const override;
    virtual const double* readState(const double* buf) override;
    virtual double* writeState(double* out_buf) const override;

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    bool on_;
    bool isImageStateClonable_; // Non state variable
    ImageType imageType_;
    LensType lensType_;
    int resolutionX_;
    int resolutionY_;
    double nearClipDistance_;
    double farClipDistance_;
    double fieldOfView_;
    double frameRate_;
    double delay_;
    std::shared_ptr<Image> image_;

    void copyCameraStateFrom(const Camera& other);
};

typedef ref_ptr<Camera> CameraPtr;

}

#endif
