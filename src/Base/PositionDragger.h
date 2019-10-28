/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_POSITION_DRAGGER_H
#define CNOID_BASE_POSITION_DRAGGER_H

#include "SceneDragger.h"
#include "exportdecl.h"

namespace cnoid {

class TranslationDragger;
class RotationDragger;

/**
   \todo Since the draggable axis set can be specified for PositoinDragger now,
   the TranslationDragger class and the RotationDragger class should be removed
   and their implementations should be integrated into the PositionDragger class.
*/
class CNOID_EXPORT PositionDragger : public SceneDragger
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PositionDragger();
    PositionDragger(const PositionDragger& org);
    PositionDragger(const PositionDragger& org, CloneMap* cloneMap);

    enum Axis { TX = 1 << 0, TY = 1 << 1, TZ = 1 << 2,
                TRANSLATION_AXES = (TX | TY | TZ),
                RX = 1 << 3, RY = 1 << 4, RZ = 1 << 5,
                ROTATION_AXES = (RX | RY | RZ),
                ALL_AXES = (TX | TY | TZ | RX | RY | RZ)
    };

    void setDraggableAxes(int axisSet);
    int draggableAxes() const;
    SignalProxy<void(int axisSet)> sigDraggableAxesChanged();

    void setRadius(double r, double translationAxisRatio = 2.0f);
    void adjustSize();
    void adjustSize(const BoundingBox& bb);
    void setContentsDragEnabled(bool on);
    bool isContentsDragEnabled() const;

    enum DisplayMode { DisplayAlways, DisplayInEditMode, DisplayInFocus, DisplayNever };
    DisplayMode displayMode() const;
    void setDisplayMode(DisplayMode mode);

    // Thw following functions are deprecated. Use displayMode and setDisplayMode instead.
    void setDraggerAlwaysShown(bool on);
    bool isDraggerAlwaysShown() const;
    void setDraggerAlwaysHidden(bool on);
    bool isDraggerAlwaysHidden() const;

    void setUndoEnabled(bool on);
    bool isUndoEnabled() const;
    void storeCurrentPositionToHistory();

    TranslationDragger* translationDragger();
    RotationDragger* rotationDragger();

    SignalProxy<void()> sigDragStarted();
    SignalProxy<void()> sigPositionDragged();
    SignalProxy<void()> sigDragFinished();

    virtual bool isDragEnabled() const override;
    virtual void setDragEnabled(bool on) override;

    virtual bool isDragging() const override;
    virtual Affine3 draggedPosition() const override;

    virtual bool onButtonPressEvent(const SceneWidgetEvent& event) override;
    virtual bool onButtonReleaseEvent(const SceneWidgetEvent& event) override;
    virtual bool onPointerMoveEvent(const SceneWidgetEvent& event) override;
    virtual void onPointerLeaveEvent(const SceneWidgetEvent& event) override;
    virtual void onFocusChanged(const SceneWidgetEvent& event, bool on) override;
    virtual void onSceneModeChanged(const SceneWidgetEvent& event) override;
    virtual bool onUndoRequest() override;
    virtual bool onRedoRequest() override;

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    class Impl;
    Impl* impl;
};
    
typedef ref_ptr<PositionDragger> PositionDraggerPtr;

}

#endif
