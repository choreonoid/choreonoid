/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "SceneCameras.h"
#include "SceneNodeClassRegistry.h"

using namespace std;
using namespace cnoid;


SgCamera::SgCamera(int classId)
    : SgPreprocessed(classId)
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


Isometry3 SgCamera::positionLookingFor(const Vector3& eye, const Vector3& direction, const Vector3& up)
{
    Vector3 d = direction.normalized();
    Vector3 c = d.cross(up).normalized();
    Vector3 u = c.cross(d);
    Isometry3 C;
    C.linear() << c, u, -d;
    C.translation() = eye;
    return C;
}


Isometry3 SgCamera::positionLookingAt(const Vector3& eye, const Vector3& center, const Vector3& up)
{
    return positionLookingFor(eye, (center - eye), up);
}


SgPerspectiveCamera::SgPerspectiveCamera(int classId)
    : SgCamera(classId)
{
    fieldOfView_ = 0.785398;
}


SgPerspectiveCamera::SgPerspectiveCamera()
    : SgPerspectiveCamera(findClassId<SgPerspectiveCamera>())
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


SgOrthographicCamera::SgOrthographicCamera(int classId)
    : SgCamera(classId)
{
    height_ = 2.0;
}


SgOrthographicCamera::SgOrthographicCamera()
    : SgOrthographicCamera(findClassId<SgOrthographicCamera>())
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

struct NodeClassRegistration {
    NodeClassRegistration() {
        SceneNodeClassRegistry::instance()
            .registerClass<SgCamera, SgPreprocessed>()
            .registerClass<SgPerspectiveCamera, SgCamera>()
            .registerClass<SgOrthographicCamera, SgCamera>();
    }
} registration;

}
