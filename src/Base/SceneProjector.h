/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_SCENE_PROJECT_H
#define CNOID_BASE_SCENE_PROJECT_H

#include "SceneWidgetEditable.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT SceneProjector
{
public:
    virtual ~SceneProjector();
    virtual bool project(const SceneWidgetEvent& event, Vector3& out_projected) const = 0;
};


class CNOID_EXPORT ScenePlaneProjector : public SceneProjector
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    ScenePlaneProjector();
    ScenePlaneProjector(const Vector3& normal, const Vector3& point);
    void setPlane(const Vector3& normal, const Vector3& point);

    const Vector3& normal() const { return normal_; }
    double d() const { return d_; }
        
    virtual bool project(const SceneWidgetEvent& event, Vector3& out_projected) const;

private:
    Vector3 normal_;
    double d_;

    bool calcPlaneLineIntersection(
        const Vector3& lineStart, const Vector3& lineEnd, Vector3& out_isect) const;
};


class CNOID_EXPORT SceneCylinderProjector : public SceneProjector
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    SceneCylinderProjector();
    SceneCylinderProjector(const Vector3& center, double radius, double height, const Quat& rotation);

    void setCylinder(const Vector3& center, double radius, double height, const Quat& rotation);
        
    virtual bool project(const SceneWidgetEvent& event, Vector3& out_projected) const;

protected:
    Vector3 center_;
    double radius_;
    double height_;
    Quat rotation_;

    bool calcUnitCylinderLineIntersection(
        const Vector3& lineStart, const Vector3& lineEnd, Vector3& out_isectFront, Vector3& out_isectBack) const;
    bool calcCylinderLineIntersection(
        const Vector3d& lineStart, const Vector3& lineEnd, Vector3& out_isectFront, Vector3& out_isectBack) const;
};

}

#endif
