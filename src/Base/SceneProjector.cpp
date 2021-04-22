/**
   @author Shin'ichiro Nakaoka
*/

#include "SceneProjector.h"
#include "SceneWidget.h"

using namespace cnoid;


SceneProjector::~SceneProjector()
{
    
}


ScenePlaneProjector::ScenePlaneProjector()
{
    normal_ << 1.0, 0.0, 0.0;
    d_ = 0.0;
}


ScenePlaneProjector::ScenePlaneProjector(const Vector3& normal, const Vector3& point)
{
    setPlane(normal, point);
}


void ScenePlaneProjector::setPlane(const Vector3& normal, const Vector3& point)
{
    normal_ = normal;
    d_ = -normal.dot(point);
}


bool ScenePlaneProjector::project(const SceneWidgetEvent* event, Vector3& out_projected) const
{
    Vector3 nearPoint, farPoint;
    SceneWidget* sceneWidget = event->sceneWidget();
    if(sceneWidget->unproject(event->x(), event->y(), 0.0, nearPoint) &&
       sceneWidget->unproject(event->x(), event->y(), 1.0, farPoint)){
        return calcPlaneLineIntersection(nearPoint, farPoint, out_projected);
    }
    return false;
}


bool ScenePlaneProjector::calcPlaneLineIntersection
(const Vector3& lineStart, const Vector3& lineEnd, Vector3& out_isect) const
{
    const Vector3& n = normal_;
    
    const double dx = lineEnd.x() - lineStart.x();
    const double dy = lineEnd.y() - lineStart.y();
    const double dz = lineEnd.z() - lineStart.z();

    const double denominator = (n.x() * dx + n.y() * dy + n.z() * dz);
    if(!denominator) {
        return false;
    }

    const double c = (n.x() * lineStart.x() + n.y() * lineStart.y() + n.z() * lineStart.z() + d_) / denominator;

    out_isect.x() = lineStart.x() - dx * c;
    out_isect.y() = lineStart.y() - dy * c;
    out_isect.z() = lineStart.z() - dz * c;

    return true;
}


SceneCylinderProjector::SceneCylinderProjector()
{
    center_ << 0.0, 0.0, 0.0;
    radius_ = 1.0;
    height_ = 1.0;
    rotation_.setIdentity();
    isFrontSurfaceProjection = true;
}


SceneCylinderProjector::SceneCylinderProjector
(const Vector3& center, double radius, double height, const Quaternion& rotation)
{
    setCylinder(center, radius, height, rotation);
    rotation_ = rotation;
    isFrontSurfaceProjection = true;
}


void SceneCylinderProjector::setCylinder
(const Vector3& center, double radius, double height, const Quaternion& rotation)
{
    center_ = center;
    radius_ = radius;
    height_ = height;
    rotation_ = rotation;
}


bool SceneCylinderProjector::initializeProjection(const SceneWidgetEvent* event)
{
    bool initialized = false;
    Vector3 front, back;
    if(project(event, front, back)){
        double d1 = (front - event->point()).norm();
        double d2 = (back - event->point()).norm();
        isFrontSurfaceProjection = (d1 <= d2);
        initialized = true;
    }
    return initialized;
}


bool SceneCylinderProjector::project(const SceneWidgetEvent* event, Vector3& out_projected) const
{
    Vector3 dummy;
    if(isFrontSurfaceProjection){
        return project(event, out_projected, dummy);
    } else {
        return project(event, dummy, out_projected);
    }
}


bool SceneCylinderProjector::project
(const SceneWidgetEvent* event, Vector3& out_isecFront, Vector3& out_isecBack) const
{
    Vector3 nearPoint, farPoint;
    SceneWidget* sceneWidget = event->sceneWidget();
    if(sceneWidget->unproject(event->x(), event->y(), 0.0, nearPoint) &&
       sceneWidget->unproject(event->x(), event->y(), 1.0, farPoint)){
        return calcCylinderLineIntersection(nearPoint, farPoint, out_isecFront, out_isecBack);
    }
    return false;
}


bool SceneCylinderProjector::calcUnitCylinderLineIntersection
(const Vector3& lineStart, const Vector3& lineEnd, Vector3& out_isectFront, Vector3& out_isectBack) const
{
    const Vector3 dir = (lineEnd - lineStart).normalized();
    const double a = dir[0] * dir[0] + dir[1] * dir[1];
    const double b = 2.0 * (lineStart[0] * dir[0] + lineStart[1] * dir[1]);
    const double c = lineStart[0] * lineStart[0] + lineStart[1] * lineStart[1] - 1.0;

    const double d = b * b - 4.0 * a * c;
    if(d < 0.0){
        return false;
    }

    const double dSqroot = sqrt(d);
    double t0, t1;
    if(b > 0.0){
        t0 = -(2.0 * c) / (dSqroot + b);
        t1 = -(dSqroot + b) / (2.0 * a);
    } else {
        t0 = (2.0 * c) / (dSqroot - b);
        t1 = (dSqroot - b) / (2.0 * a);
    }

    out_isectFront = lineStart + dir * t0;
    out_isectBack = lineStart + dir * t1;

    return true;
}


bool SceneCylinderProjector::calcCylinderLineIntersection
(const Vector3d& lineStart, const Vector3& lineEnd, Vector3& out_isectFront, Vector3& out_isectBack) const
{
    // Compute matrix transformation that takes the cylinder to a unit cylinder with Z-axis as it's axis and
    // (0,0,0) as it's center.
    const double rinv = 1.0 / radius_;
    const Affine3 toUnitCylInZ = rotation_.inverse() * Eigen::Scaling(rinv, rinv, rinv) * Translation3(-center_);
                               
    // Transform the lineStart and lineEnd into the unit cylinder space.
    const Vector3 unitCylLineStart = toUnitCylInZ * lineStart;
    const Vector3 unitCylLineEnd   = toUnitCylInZ * lineEnd;

    // Intersect line with unit cylinder.
    Vector3 unitCylIsectFront, unitCylIsectBack;
    if(!calcUnitCylinderLineIntersection(unitCylLineStart, unitCylLineEnd, unitCylIsectFront, unitCylIsectBack)){
        return false;
    }

    // Transform back from unit cylinder space.
    const Affine3 invToUnitCylInZ = toUnitCylInZ.inverse();
    out_isectFront = invToUnitCylInZ * unitCylIsectFront;
    out_isectBack = invToUnitCylInZ * unitCylIsectBack;    

    return true;
}
