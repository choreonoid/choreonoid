#ifndef CNOID_BASE_COORDINATE_AXES_OVERLAY_H
#define CNOID_BASE_COORDINATE_AXES_OVERLAY_H

#include "SceneDrawables.h"
#include "exportdecl.h"

namespace cnoid {

class SceneRenderer;

class CNOID_EXPORT CoordinateAxesOverlay : public SgViewportOverlay
{
    const double length = 15;
    const double width = 4;
    SgPosTransformPtr topTransform;
    SgPosTransformPtr axisTransforms[3];
    int superClassId;
    
public:
    enum CoordinateSystem { RightHanded, LeftHanded };
    CoordinateAxesOverlay(CoordinateSystem coordinateSystem = RightHanded);
    void setCoordinateSystem(CoordinateSystem coordinateSystem);
    virtual void calcViewVolume(double viewportWidth, double viewportHeight, ViewVolume& io_volume) override;
    void render(SceneRenderer* renderer);
};

}

#endif
