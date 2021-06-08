/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_SCENE_PROJECTOR_H
#define CNOID_BASE_SCENE_PROJECTOR_H

#include "SceneWidgetEventHandler.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT SceneProjector
{
public:
    virtual ~SceneProjector();
    virtual bool project(const SceneWidgetEvent* event, Vector3& out_projected) const = 0;
};


class CNOID_EXPORT ScenePlaneProjector : public SceneProjector
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ScenePlaneProjector();
    ScenePlaneProjector(const Vector3& normal, const Vector3& point);
    void setPlane(const Vector3& normal, const Vector3& point);

    const Vector3& normal() const { return normal_; }
    double d() const { return d_; }
        
    virtual bool project(const SceneWidgetEvent* event, Vector3& out_projected) const;

private:
    Vector3 normal_;
    double d_;

    bool calcPlaneLineIntersection(
        const Vector3& lineStart, const Vector3& lineEnd, Vector3& out_isect) const;
};


class CNOID_EXPORT SceneCylinderProjector : public SceneProjector
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SceneCylinderProjector();
    SceneCylinderProjector(const Vector3& center, double radius, double height, const Quaternion& rotation);

    void setCylinder(const Vector3& center, double radius, double height, const Quaternion& rotation);

    /**
       This function must be called for the initial event to start the projection
       to determine whether the surface the user is dragging is the front or back of the cylinder.
    */
    bool initializeProjection(const SceneWidgetEvent* event);
        
    virtual bool project(const SceneWidgetEvent* event, Vector3& out_projected) const;

protected:
    Vector3 center_;
    double radius_;
    double height_;
    Quaternion rotation_;
    bool isFrontSurfaceProjection;

    bool project(
        const SceneWidgetEvent* event, Vector3& out_isecFront, Vector3& out_isecBack) const;
    bool calcUnitCylinderLineIntersection(
        const Vector3& lineStart, const Vector3& lineEnd, Vector3& out_isectFront, Vector3& out_isectBack) const;
    bool calcCylinderLineIntersection(
        const Vector3d& lineStart, const Vector3& lineEnd, Vector3& out_isectFront, Vector3& out_isectBack) const;
};

}

#endif
