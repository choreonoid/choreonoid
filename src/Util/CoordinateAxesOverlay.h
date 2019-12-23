/**
   @author Shin'ichiro Nakaoka
*/

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
    SgPosTransformPtr axesTransform;
    int superClassId;
    
public:
    CoordinateAxesOverlay();
    virtual void calcViewVolume(double viewportWidth, double viewportHeight, ViewVolume& io_volume) override;
    void render(SceneRenderer* renderer);
};

}

#endif
