/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_SCENE_DRAGGER_H
#define CNOID_BASE_SCENE_DRAGGER_H

#include "SceneWidgetEditable.h"
#include "SceneDragProjector.h"
#include <cnoid/SceneShape>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT TranslationDragger : public SgPosTransform, public SceneWidgetEditable
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    TranslationDragger(bool setDefaultAxes = true);
    TranslationDragger(const TranslationDragger& org);
    TranslationDragger(const TranslationDragger& org, SgCloneMap& cloneMap);

    virtual SgObject* clone(SgCloneMap& cloneMap) const;

    void addCustomAxis(int axis, SgNode* node);
    void clearCustomAxes();

    double radius() const;
    void setRadius(double r);
    
    bool isContainerMode() const;
    void setContainerMode(bool on);

    SignalProxy<void()> sigTranslationStarted() {
        return sigTranslationStarted_;
    }
    SignalProxy<void()> sigTranslationDragged() {
        return sigTranslationDragged_;
    }
    SignalProxy<void()> sigTranslationFinished() {
        return sigTranslationFinished_;
    }

    bool isDragging() const;
    const Affine3& draggedPosition() const;
    const Vector3& draggedTranslation() const;

    virtual bool onButtonPressEvent(const SceneWidgetEvent& event);
    virtual bool onButtonReleaseEvent(const SceneWidgetEvent& event);
    virtual bool onPointerMoveEvent(const SceneWidgetEvent& event);
    virtual void onPointerLeaveEvent(const SceneWidgetEvent& event);
        
private:
    SgScaleTransformPtr defaultAxesScale;
    SgGroupPtr customAxes;
    SgMaterialPtr axisMaterials[3];
    SceneDragProjector dragProjector;
    double axisCylinderNormalizedRadius;
    bool isContainerMode_;
    Signal<void()> sigTranslationStarted_;
    Signal<void()> sigTranslationDragged_;
    Signal<void()> sigTranslationFinished_;
};

typedef ref_ptr<TranslationDragger> TranslationDraggerPtr;


class CNOID_EXPORT RotationDragger : public SgPosTransform, public SceneWidgetEditable
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    RotationDragger();
    RotationDragger(const RotationDragger& org);
    RotationDragger(const RotationDragger& org, SgCloneMap& cloneMap);

    virtual SgObject* clone(SgCloneMap& cloneMap) const;

    void setRadius(double r);
    void setContainerMode(bool on);
    bool isContainerMode() const;

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
    const Affine3& draggedPosition() const;

    virtual bool onButtonPressEvent(const SceneWidgetEvent& event);
    virtual bool onButtonReleaseEvent(const SceneWidgetEvent& event);
    virtual bool onPointerMoveEvent(const SceneWidgetEvent& event);
    virtual void onPointerLeaveEvent(const SceneWidgetEvent& event);
        
private:
    SgScaleTransformPtr scale;
    SceneDragProjector dragProjector;
    bool isContainerMode_;
    Signal<void()> sigRotationStarted_;
    Signal<void(const AngleAxis& rotation)> sigRotationDragged_;
    Signal<void()> sigRotationFinished_;
};
    
typedef ref_ptr<RotationDragger> RotationDraggerPtr;

class CNOID_EXPORT PositionDragger : public SgPosTransform, public SceneWidgetEditable
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    PositionDragger();
    PositionDragger(const PositionDragger& org);
    PositionDragger(const PositionDragger& org, SgCloneMap& cloneMap);

    virtual SgObject* clone(SgCloneMap& cloneMap) const;

    void setRadius(double r, double translationAxisRatio = 2.0f);
    void adjustSize();
    void adjustSize(const BoundingBox& bb);
    void setContainerMode(bool on);
    bool isContainerMode() const;
    void setContentsDragEnabled(bool on);
    bool isContentsDragEnabled() const;
    void setDraggerAlwaysShown(bool on);
    bool isDraggerAlwaysShown() const;
    void setDraggerAlwaysHidden(bool on);
    bool isDraggerAlwaysHidden() const;

    TranslationDragger* translationDragger() { return translationDragger_; }
    RotationDragger* rotationDragger() { return rotationDragger_; }

    SignalProxy<void()> sigDragStarted() {
        return sigDragStarted_;
    }
    SignalProxy<void()> sigPositionDragged() {
        return sigPositionDragged_;
    }
    SignalProxy<void()> sigDragFinished() {
        return sigDragFinished_;
    }

    bool isDragging() const;
    Affine3 draggedPosition() const;

    virtual bool onButtonPressEvent(const SceneWidgetEvent& event);
    virtual bool onButtonReleaseEvent(const SceneWidgetEvent& event);
    virtual bool onPointerMoveEvent(const SceneWidgetEvent& event);
    virtual void onPointerLeaveEvent(const SceneWidgetEvent& event);
    virtual void onFocusChanged(const SceneWidgetEvent& event, bool on);
    virtual void onSceneModeChanged(const SceneWidgetEvent& event);
        
private:
    TranslationDraggerPtr translationDragger_;
    RotationDraggerPtr rotationDragger_;
    SceneDragProjector dragProjector;
    bool isContainerMode_;
    bool isContentsDragEnabled_;
    bool isDraggerAlwaysShown_;
    bool isDraggerAlwaysHidden_;
    bool isDraggerShown;
    Signal<void()> sigDragStarted_;
    Signal<void()> sigPositionDragged_;
    Signal<void()> sigDragFinished_;

    void initalizeDraggers();
    void onSubDraggerDragged();
    void showDragMarkers(bool on);
};
    
typedef ref_ptr<PositionDragger> PositionDraggerPtr;
}

#endif
