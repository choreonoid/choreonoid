/**
   @author Shizuko Hattori
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_SCENE_MARKER_H
#define CNOID_UTIL_SCENE_MARKER_H

#include "SceneShape.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT CrossMarker : public SgPosTransform
{
public:
    CrossMarker(double size, const Vector3f& color, double lineWidth = 1.0);
    void setSize(double size);

private:
    SgVertexArrayPtr vertices;
    double size_;
};
typedef ref_ptr<CrossMarker> CrossMarkerPtr;


class CNOID_EXPORT SphereMarker : public SgPosTransform
{
public:
    SphereMarker();
    SphereMarker(double radius, const Vector3f& color, float transparency = 0.0);
    void setRadius(double r);
    void setColor(const Vector3f& c);
private:
    void initialize(double radius, const Vector3f& color, float transparency);
    SgScaleTransformPtr scale;
    SgMaterialPtr material;
};
typedef ref_ptr<SphereMarker> SphereMarkerPtr;


class CNOID_EXPORT BoundingBoxMarker : public SgGroup
{
public:
    BoundingBoxMarker(const BoundingBox& bbox, const Vector3f& color, float transparency, double width);
    BoundingBoxMarker(const BoundingBox& bbox, const Vector3f& color, float transparency);

private:
    void create(const BoundingBox& bbox, const Vector3f& color, float transparency, double width);
    void addMarker(SgShape* shape, double x, double y, double z);
};
typedef ref_ptr<BoundingBoxMarker> BoundingBoxMarkerPtr;

}

#endif
