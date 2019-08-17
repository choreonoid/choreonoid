/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_ROTATIN_DRAGGER_H
#define CNOID_BASE_ROTATIN_DRAGGER_H

#include "SceneDragger.h"
#include "SceneDragProjector.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT RotationDragger : public SceneDragger
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RotationDragger();
    RotationDragger(const RotationDragger& org, SgCloneMap* cloneMap = nullptr);

    virtual SgObject* doClone(SgCloneMap* cloneMap) const override;

    enum Axis { RX = 1, RY = 2, RZ = 4 };

    void setDraggableAxes(int axisSet);
    int draggableAxes() const { return draggableAxes_; }

    void setRadius(double r);

    SignalProxy<void()> sigRotationStarted() {
        return sigRotationStarted_;
    }
    /**
       \todo The rotation parameter should be removed.
    */
    SignalProxy<void(const AngleAxis& rotation)> sigRotationDragged() {
        return sigRotationDragged_;
    }
    SignalProxy<void()> sigRotationFinished() {
        return sigRotationFinished_;
    }

    bool isDragging() const;
    const AngleAxis& draggedAngleAxis() const;
    Affine3 draggedPosition() const;

    virtual bool onButtonPressEvent(const SceneWidgetEvent& event) override;
    virtual bool onButtonReleaseEvent(const SceneWidgetEvent& event) override;
    virtual bool onPointerMoveEvent(const SceneWidgetEvent& event) override;
    virtual void onPointerLeaveEvent(const SceneWidgetEvent& event) override;

private:
    int draggableAxes_;
    SgScaleTransformPtr scale;
    SceneDragProjector dragProjector;
    Signal<void()> sigRotationStarted_;
    Signal<void(const AngleAxis& rotation)> sigRotationDragged_;
    Signal<void()> sigRotationFinished_;
};
    
typedef ref_ptr<RotationDragger> RotationDraggerPtr;

}

#endif
