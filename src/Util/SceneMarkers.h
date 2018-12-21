/**
   @author Shizuko Hattori
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_SCENE_MARKERS_H
#define CNOID_UTIL_SCENE_MARKERS_H

#include "SceneDrawables.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT SceneMarker : public SgPosTransform
{
public:
    SceneMarker();

    enum MarkerType {
        NO_MARKER,
        CROSS_MARKER,
        SPHERE_MARKER,
        AXES_MARKER,
        N_MARKER_TYPES
    };

    int markerType() const { return markerType_; }
    void setMarkerType(int type){ markerType_ = type; }
    double markerSize() const { return markerSize_; }
    void setMarkerSize(double size){ markerSize_ = size; }
    void updateMarker(bool doNotify = false);
        
    const Vector3f& color() const;
    void setColor(const Vector3f& c);
    float emission() const;
    void setEmission(float r);
    double transparency() const;
    void setTransparency(float t);

private:
    int markerType_;
    float markerSize_;
    SgMaterialPtr material;
    float emission_;

    void setCross();
    void setSphere();
    void setAxes();
};

typedef ref_ptr<SceneMarker> SceneMarkerPtr;


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
