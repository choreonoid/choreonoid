/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_SCENE_CAMERAS_H
#define CNOID_UTIL_SCENE_CAMERAS_H

#include "SceneGraph.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT SgCamera : public SgPreprocessed
{
protected:
    SgCamera(int classId);
    SgCamera(const SgCamera& org);
        
public:
    static Isometry3 positionLookingFor(const Vector3& eye, const Vector3& direction, const Vector3& up);
    static Isometry3 positionLookingAt(const Vector3& eye, const Vector3& center, const Vector3& up);

    template<class Scalar, int Mode, int Options>
    static Eigen::Matrix<Scalar, 3, 1> right(const Eigen::Transform<Scalar, 3, Mode, Options>& T){
        return T.linear().col(0);
    }
    template<class Scalar, int Mode, int Options>
    static Eigen::Matrix<Scalar, 3, 1> direction(const Eigen::Transform<Scalar, 3, Mode, Options>& T){
        return -T.linear().col(2);
    }
    template<class Scalar, int Mode, int Options>
    static Eigen::Matrix<Scalar, 3, 1> up(const Eigen::Transform<Scalar, 3, Mode, Options>& T){
        return T.linear().col(1);
    }

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

private:
    double nearClipDistance_;
    double farClipDistance_;
};


class CNOID_EXPORT SgPerspectiveCamera : public SgCamera
{
protected:
    SgPerspectiveCamera(int classId);
    
public:
    SgPerspectiveCamera();
    SgPerspectiveCamera(const SgPerspectiveCamera& org);

    double fieldOfView() const { return fieldOfView_; }
    void setFieldOfView(double fov) { fieldOfView_ = fov; }

    static double fovy(double aspectRatio, double fieldOfView);
        
    double fovy(double aspectRatio) const {
        return SgPerspectiveCamera::fovy(aspectRatio, fieldOfView_);
    }

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    double fieldOfView_;
};


class CNOID_EXPORT SgOrthographicCamera : public SgCamera
{
protected:
    SgOrthographicCamera(int classId);
    
public:
    SgOrthographicCamera();
    SgOrthographicCamera(const SgOrthographicCamera& org);

    double height() const { return height_; }
    void setHeight(double h) { height_ = h; }

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    double height_;
};

}

#endif
