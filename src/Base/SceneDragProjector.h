/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_SCENE_DRAG_PROJECTOR_H
#define CNOID_BASE_SCENE_DRAG_PROJECTOR_H

#include "SceneWidgetEditable.h"
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
        
    void setInitialPosition(const Affine3& T);
    void setInitialTranslation(const Vector3& p);
    void setInitialRotation(const Matrix3& R);
    const Affine3& initialPosition() const;

    // for 1-D rotation
    void setRotationAxis(const Vector3& axis);
    const Vector3& rotationAxis() const;

    // for 1-D translation
    void setTranslationAxis(const Vector3& axis);
    const Vector3& translationAxis() const;
        
    // for 2-D translation
    void setTranslationPlaneNormal(const Vector3& normal);
    void setTranslationAlongViewPlane();

    bool startRotation(const SceneWidgetEvent& event);
    bool startTranslation(const SceneWidgetEvent& event);

    bool drag(const SceneWidgetEvent& event);
    bool dragRotation(const SceneWidgetEvent& event);
    bool dragTranslation(const SceneWidgetEvent& event);

    const Vector3& projectedPoint() const;
    const Affine3& position() const;
    const Matrix3& rotationMatrix() const;
    double rotationAngle() const;
    const AngleAxis& rotationAngleAxis() const;
    const Vector3& translation() const;
        
private:
    SceneDragProjectorImpl* impl;
};

}

#endif
