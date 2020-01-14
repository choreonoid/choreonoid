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
    enum AxisBit {
        TX = 1 << 0, TY = 1 << 1, TZ = 1 << 2,
        TranslationAxes = (TX | TY | TZ),
        RX = 1 << 3, RY = 1 << 4, RZ = 1 << 5,
        RotationAxes = (RX | RY | RZ),
        AllAxes = (TX | TY | TZ | RX | RY | RZ),

        // deprecated
        TRANSLATION_AXES = TranslationAxes,
        ROTATION_AXES = RotationAxes,
        ALL_AXES = AllAxes
    };

    enum HandleType {
        StandardHandle = 0,
        PositiveOnlyHandle = 1,
        WideHandle = 2,
    };
        
    PositionDragger(int axes = AllAxes, int handleType = StandardHandle);
    PositionDragger(const PositionDragger& org) = delete;

    void setDraggableAxes(int axisBitSet);
    int draggableAxes() const;
    SignalProxy<void(int axisBitSet)> sigDraggableAxesChanged();

    double handleSize() const;
    void setHandleSize(double s);

    double rotationHandleSizeRatio() const;
    void setRotationHandleSizeRatio(double r);

    //! \deprecated. Use the setHandleSize and setRotationHandlerSizeRatio functions.
    void setRadius(double r, double translationAxisRatio = 2.0f);
    //! \deprecated. Use the handleSize and rotationHandleSizeRatio function.
    double radius() const;
    
    bool adjustSize();
    bool adjustSize(const BoundingBox& bb);

    void setTransparency(float t);
    float transparency() const;

    void setOverlayMode(bool on);
    bool isOverlayMode() const;

    void setConstantPixelSizeMode(bool on, double pixelSizeRatio = 1.0);
    bool isConstantPixelSizeMode() const;
    
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

    class Impl;

private:
    Impl* impl;
};
    
typedef ref_ptr<PositionDragger> PositionDraggerPtr;

}

#endif
