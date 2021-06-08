/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_SCENE_DRAG_PROJECTOR_H
#define CNOID_BASE_SCENE_DRAG_PROJECTOR_H

#include "SceneWidgetEvent.h"
#include "exportdecl.h"

namespace cnoid {

class SceneDragProjectorImpl;

class CNOID_EXPORT SceneDragProjector
{
public:
    SceneDragProjector();
    virtual ~SceneDragProjector();
        
    enum DragMode { DRAG_NONE, DRAG_ROTATION, DRAG_TRANSLATION };

    int dragMode() const;
    bool isDragging() const;

    void resetDragMode();
        
    void setInitialPosition(const Isometry3& T);
    void setInitialTranslation(const Vector3& p);
    void setInitialRotation(const Matrix3& R);
    const Isometry3& initialPosition() const;

    // for 1-D rotation
    void setRotationAxis(const Vector3& axis);
    const Vector3& rotationAxis() const;

    // for 1-D translation
    void setTranslationAxis(const Vector3& axis);
    const Vector3& translationAxis() const;
        
    // for 2-D translation
    void setTranslationPlaneNormal(const Vector3& normal);
    void setTranslationAlongViewPlane();

    bool startRotation(SceneWidgetEvent* event);
    bool startTranslation(SceneWidgetEvent* event);

    bool drag(SceneWidgetEvent* event);
    bool dragRotation(SceneWidgetEvent* event);
    bool dragTranslation(SceneWidgetEvent* event);

    const Vector3& projectedPoint() const;
    const Isometry3& position() const;
    const Matrix3& rotationMatrix() const;
    double rotationAngle() const;
    const AngleAxis& rotationAngleAxis() const;
    const Vector3& translation() const;
        
private:
    SceneDragProjectorImpl* impl;
};

}

#endif
