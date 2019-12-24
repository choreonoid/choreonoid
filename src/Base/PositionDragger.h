/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_POSITION_DRAGGER_H
#define CNOID_BASE_POSITION_DRAGGER_H

#include "SceneWidgetEditable.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT PositionDragger : public SgPosTransform, public SceneWidgetEditable
{
public:
    enum Axis { TX = 1 << 0, TY = 1 << 1, TZ = 1 << 2,
                TRANSLATION_AXES = (TX | TY | TZ),
                RX = 1 << 3, RY = 1 << 4, RZ = 1 << 5,
                ROTATION_AXES = (RX | RY | RZ),
                ALL_AXES = (TX | TY | TZ | RX | RY | RZ)
    };

    PositionDragger();
    PositionDragger(int axisSet);
    PositionDragger(const PositionDragger& org);

    void setDraggableAxes(int axisSet);
    int draggableAxes() const;
    SignalProxy<void(int axisSet)> sigDraggableAxesChanged();

    double handleSize() const;
    void setHandleSize(double s);
    double rotationHandleSizeRatio() const;
    void setRotationHandlerSizeRatio(double r);

    //! \deprecated. Use the setHandleSize and setRotationHandlerSizeRatio functions.
    void setRadius(double r, double translationAxisRatio = 2.0f);
    //! \deprecated. Use the handleSize and rotationHandleSizeRatio function.
    double radius() const;
    
    void adjustSize();
    void adjustSize(const BoundingBox& bb);

    void setOverlayMode(bool on);
    bool isOverlayMode() const;

    bool isContainerMode() const;
    void setContainerMode(bool on);

    void setContentsDragEnabled(bool on);
    bool isContentsDragEnabled() const;

    enum DisplayMode { DisplayAlways, DisplayInEditMode, DisplayInFocus, DisplayNever };
    DisplayMode displayMode() const;
    void setDisplayMode(DisplayMode mode);

    void setUndoEnabled(bool on);
    bool isUndoEnabled() const;
    void storeCurrentPositionToHistory();

    bool isDragEnabled() const;
    void setDragEnabled(bool on);
    bool isDragging() const;
    Affine3 draggedPosition() const;
    const Vector3& draggedTranslation() const;
    const AngleAxis& draggedAngleAxis() const;

    SignalProxy<void()> sigDragStarted();
    SignalProxy<void()> sigPositionDragged();
    SignalProxy<void()> sigDragFinished();

    virtual bool onButtonPressEvent(const SceneWidgetEvent& event) override;
    virtual bool onButtonReleaseEvent(const SceneWidgetEvent& event) override;
    virtual bool onPointerMoveEvent(const SceneWidgetEvent& event) override;
    virtual void onPointerLeaveEvent(const SceneWidgetEvent& event) override;
    virtual void onFocusChanged(const SceneWidgetEvent& event, bool on) override;
    virtual void onSceneModeChanged(const SceneWidgetEvent& event) override;
    virtual bool onUndoRequest() override;
    virtual bool onRedoRequest() override;

    // Thw following functions are deprecated. Use displayMode and setDisplayMode instead.
    void setDraggerAlwaysShown(bool on);
    bool isDraggerAlwaysShown() const;
    void setDraggerAlwaysHidden(bool on);
    bool isDraggerAlwaysHidden() const;

protected:
    PositionDragger(const PositionDragger& org, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    class Impl;
    Impl* impl;
};
    
typedef ref_ptr<PositionDragger> PositionDraggerPtr;

}

#endif
