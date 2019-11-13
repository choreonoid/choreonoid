/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "SceneCameras.h"

using namespace std;
using namespace cnoid;


SgCamera::SgCamera(int polymorhicId)
    : SgPreprocessed(polymorhicId)
{
    nearClipDistance_ = 0.04;
    farClipDistance_ = 200.0;
}


SgCamera::SgCamera(const SgCamera& org)
    : SgPreprocessed(org)
{
    nearClipDistance_ = org.nearClipDistance_;
    farClipDistance_ = org.farClipDistance_;
}


Affine3 SgCamera::positionLookingFor(const Vector3& eye, const Vector3& direction, const Vector3& up)
{
    Vector3 d = direction.normalized();
    Vector3 c = d.cross(up).normalized();
    Vector3 u = c.cross(d);
    Affine3 C;
    C.linear() << c, u, -d;
    C.translation() = eye;
    return C;
}


Affine3 SgCamera::positionLookingAt(const Vector3& eye, const Vector3& center, const Vector3& up)
{
    return positionLookingFor(eye, (center - eye), up);
}


SgPerspectiveCamera::SgPerspectiveCamera(int polymorhicId)
    : SgCamera(polymorhicId)
{
    fieldOfView_ = 0.785398;
}


SgPerspectiveCamera::SgPerspectiveCamera()
    : SgPerspectiveCamera(findPolymorphicId<SgPerspectiveCamera>())
{

}


SgPerspectiveCamera::SgPerspectiveCamera(const SgPerspectiveCamera& org)
    : SgCamera(org)
{
    fieldOfView_ = org.fieldOfView_;
}


Referenced* SgPerspectiveCamera::doClone(CloneMap*) const
{
    return new SgPerspectiveCamera(*this);
}


/**
   @param aspectRatio width / height
*/
double SgPerspectiveCamera::fovy(double aspectRatio, double fieldOfView)
{
    if(aspectRatio >= 1.0){
        return fieldOfView;
    } else {
        return 2.0 * atan(tan(fieldOfView / 2.0) / aspectRatio);
    }
}


SgOrthographicCamera::SgOrthographicCamera(int polymorhicId)
    : SgCamera(polymorhicId)
{
    height_ = 2.0;
}


SgOrthographicCamera::SgOrthographicCamera()
    : SgOrthographicCamera(findPolymorphicId<SgOrthographicCamera>())
{

}


SgOrthographicCamera::SgOrthographicCamera(const SgOrthographicCamera& org)
    : SgCamera(org)
{
    height_ = org.height_;
}


Referenced* SgOrthographicCamera::doClone(CloneMap*) const
{
    return new SgOrthographicCamera(*this);
}


namespace {

struct NodeTypeRegistration {
    NodeTypeRegistration() {
        SgNode::registerType<SgCamera, SgPreprocessed>();
        SgNode::registerType<SgPerspectiveCamera, SgCamera>();
        SgNode::registerType<SgOrthographicCamera, SgCamera>();
    }
} registration;

}
