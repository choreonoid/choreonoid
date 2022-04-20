#ifndef CNOID_BODY_CAMERA_H
#define CNOID_BODY_CAMERA_H

#include "VisionSensor.h"
#include <cnoid/Image>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT Camera : public VisionSensor
{
public:
    Camera();
    Camera(const Camera& org, bool copyStateOnly = false);

    virtual const char* typeName() const override;
    virtual void copyStateFrom(const DeviceState& other) override;
    virtual DeviceState* cloneState() const override;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;
    virtual void clearState() override;

    void setImageStateClonable(bool on);
    bool isImageStateClonable() const { return spec ? spec->isImageStateClonable : true; }

    enum ImageType { NO_IMAGE, COLOR_IMAGE, GRAYSCALE_IMAGE };
    enum LensType { NORMAL_LENS, FISHEYE_LENS, DUAL_FISHEYE_LENS };

    ImageType imageType() const { return imageType_; }
    void setImageType(ImageType type) { imageType_ = type; }

    LensType lensType() const { return lensType_; }
    void setLensType(LensType type) { lensType_ = type; }

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
    double horizontalFieldOfView() const;
    void setHorizontalFieldOfView(double hfov);

    void setResolution(int x, int y) {
        resolutionX_ = x; resolutionY_ = y;
    }
    void setResolutionX(int x) { resolutionX_ = x; }
    void setResolutionY(int y) { resolutionY_ = y; }
    
    int resolutionX() const { return resolutionX_; }
    int resolutionY() const { return resolutionY_; }

    const Image& image() const { return *image_; }
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

    virtual int stateSize() const override;
    virtual const double* readState(const double* buf) override;
    virtual double* writeState(double* out_buf) const override;

    bool readSpecifications(const Mapping* info);
    bool writeSpecifications(Mapping* info) const;

protected:
    void copyCameraStateFrom(const Camera& other, bool doCopyVisionSensorState, bool doCopyImage);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    ImageType imageType_;
    LensType lensType_;
    int resolutionX_;
    int resolutionY_;
    double nearClipDistance_;
    double farClipDistance_;
    double fieldOfView_;
    double frameRate_;
    std::shared_ptr<Image> image_;

    struct Spec {
        bool isImageStateClonable;
    };
    std::unique_ptr<Spec> spec;

    void copyCameraStateFrom(const Camera& other);
};

typedef ref_ptr<Camera> CameraPtr;

}

#endif
