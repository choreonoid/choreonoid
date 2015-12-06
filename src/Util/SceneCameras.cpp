/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "SceneCameras.h"
#include "SceneVisitor.h"

using namespace std;
using namespace cnoid;


SgCamera::SgCamera()
{
    nearDistance_ = 0.01;
    farDistance_ = 100.0;
}


SgCamera::SgCamera(const SgCamera& org)
    : SgPreprocessed(org)
{
    nearDistance_ = org.nearDistance_;
    farDistance_ = org.farDistance_;
}


void SgCamera::accept(SceneVisitor& visitor)
{
    visitor.visitCamera(this);
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


SgPerspectiveCamera::SgPerspectiveCamera()
{
    fieldOfView_ = 0.785398;
}


SgPerspectiveCamera::SgPerspectiveCamera(const SgPerspectiveCamera& org)
    : SgCamera(org)
{
    fieldOfView_ = org.fieldOfView_;
}


SgObject* SgPerspectiveCamera::clone(SgCloneMap& cloneMap) const
{
    return new SgPerspectiveCamera(*this);
}


void SgPerspectiveCamera::accept(SceneVisitor& visitor)
{
    visitor.visitCamera(this);
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


SgOrthographicCamera::SgOrthographicCamera()
{
    height_ = 2.0;
}


SgOrthographicCamera::SgOrthographicCamera(const SgOrthographicCamera& org)
    : SgCamera(org)
{
    height_ = org.height_;
}


SgObject* SgOrthographicCamera::clone(SgCloneMap& cloneMap) const
{
    return new SgOrthographicCamera(*this);
}


void SgOrthographicCamera::accept(SceneVisitor& visitor)
{
    visitor.visitCamera(this);
}
