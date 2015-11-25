/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_CAMERAS_H
#define CNOID_BODY_CAMERAS_H

#include "VisionSensor.h"
#include <cnoid/Image>
#include <boost/shared_ptr.hpp>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT Camera : public VisionSensor
{
public:
    Camera();
    Camera(const Camera& org, bool copyAll = true);

    virtual const char* typeName();
    void copyStateFrom(const Camera& other);
    virtual void copyStateFrom(const DeviceState& other);
    virtual DeviceState* cloneState() const;
    virtual Device* clone() const;
    virtual void forEachActualType(boost::function<bool(const std::type_info& type)> func);
    virtual void clearState();
    virtual int stateSize() const;
    virtual const double* readState(const double* buf);
    virtual double* writeState(double* out_buf) const;

    void setShotDataAsState(bool on) { isShotDataSetAsState_ = on; }
    bool isShotDataSetAsState() const { return isShotDataSetAsState_; }

    enum ImageType { NO_IMAGE, COLOR_IMAGE, GRAYSCALE_IMAGE };
    ImageType imageType() const { return imageType_; }
    void setImageType(ImageType type) { imageType_ = type; }

    bool on() const { return on_; }
    void on(bool on) { on_ = on; }

    double nearDistance() const { return nearDistance_; }
    void setNearDistance(double d) { nearDistance_ = d; }

    double farDistance() const { return farDistance_; }
    void setFarDistance(double d) { farDistance_ = d; }
        
    double fieldOfView() const { return fieldOfView_; }
    void setFieldOfView(double f) { fieldOfView_ = f; }

    void setResolution(int x, int y) {
        resolutionX_ = x; resolutionY_ = y;
    }
    int resolutionX() const { return resolutionX_; }
    int resolutionY() const { return resolutionY_; }

    void setFrameRate(double r) { frameRate_ = r; }
    double frameRate() const { return frameRate_; }

    const Image& image() const { return *image_; }
    const Image& constImage() const { return *image_; }
    Image& image();
    Image& newImage();

    boost::shared_ptr<const Image> sharedImage() const { return image_; }

    /**
       Move semantics. If the use_count() of the given shared image pointer is one,
       the data is moved to the Camera object and the ownership of the given pointer is released.
       Otherwise, the data is copied.
    */
    void setImage(boost::shared_ptr<Image>& image);

private:
    bool on_;
    bool isShotDataSetAsState_;
    ImageType imageType_;
    int resolutionX_;
    int resolutionY_;
    double nearDistance_;
    double farDistance_;
    double fieldOfView_;
    double frameRate_;
    boost::shared_ptr<Image> image_;

    void copyCameraStateFrom(const Camera& other);    
};

typedef ref_ptr<Camera> CameraPtr;


class CNOID_EXPORT RangeCamera : public Camera
{
public:
    RangeCamera();
    RangeCamera(const RangeCamera& org, bool copyAll = true);

    virtual const char* typeName();
    void copyStateFrom(const RangeCamera& other); 
    virtual void copyStateFrom(const DeviceState& other);
    virtual DeviceState* cloneState() const;
    virtual Device* clone() const;
    virtual void forEachActualType(boost::function<bool(const std::type_info& type)> func);
    virtual void clearState();

    int numPoints() const { return points_->size(); }

    typedef std::vector<Vector3f> PointData;

    const PointData& points() const { return *points_; }
    const PointData& constPoints() const { return *points_; }
    PointData& points();
    PointData& newPoints();

    bool isOrganized() const { return isOrganized_; }
    void setOrganized(bool on);
            
    boost::shared_ptr<const PointData> sharedPoints() const { return points_; }

    /**
       Move semantics. If the use_count() of the given shared point data pointer is one,
       the data is moved to the Camera object and the ownership of the given pointer is released.
       Otherwise, the data is copied.
    */
    void setPoints(boost::shared_ptr<PointData>& points);

private:
    boost::shared_ptr< std::vector<Vector3f> > points_;
    bool isOrganized_;

    void copyRangeCameraStateFrom(const RangeCamera& other);    
};

typedef ref_ptr<RangeCamera> RangeCameraPtr;
};

#endif
