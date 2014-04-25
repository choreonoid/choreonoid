/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_SCENE_DRAGGER_H_INCLUDED
#define CNOID_BASE_SCENE_DRAGGER_H_INCLUDED

#include "SceneWidgetEditable.h"
#include "SceneDragProjector.h"
#include <cnoid/SceneShape>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT TranslationDragger : public SgPosTransform, public SceneWidgetEditable
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    TranslationDragger();
    TranslationDragger(const TranslationDragger& org);
    TranslationDragger(const TranslationDragger& org, SgCloneMap& cloneMap);

    virtual SgObject* clone(SgCloneMap& cloneMap) const;

    double radius() const;
    void setRadius(double r);
    bool isContainerMode() const;
    void setContainerMode(bool on);

    SignalProxy< boost::signal<void()> > sigTranslationStarted() {
        return sigTranslationStarted_;
    }
    SignalProxy< boost::signal<void()> > sigTranslationDragged() {
        return sigTranslationDragged_;
    }
    SignalProxy< boost::signal<void()> > sigTranslationFinished() {
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
    SgScaleTransformPtr scale;
    SgPosTransformPtr arrows[6];
    SceneDragProjector dragProjector;
    double axisCylinderNormalizedRadius;
    bool isContainerMode_;
    boost::signal<void()> sigTranslationStarted_;
    boost::signal<void()> sigTranslationDragged_;
    boost::signal<void()> sigTranslationFinished_;
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

    SignalProxy< boost::signal<void()> > sigRotationStarted() {
        return sigRotationStarted_;
    }
    /**
       \todo The rotation parameter should be removed.
    */
    SignalProxy< boost::signal<void(const AngleAxis& rotation)> > sigRotationDragged() {
        return sigRotationDragged_;
    }
    SignalProxy< boost::signal<void()> > sigRotationFinished() {
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
    SgShapePtr belts[3];
    SceneDragProjector dragProjector;
    bool isContainerMode_;
    boost::signal<void()> sigRotationStarted_;
    boost::signal<void(const AngleAxis& rotation)> sigRotationDragged_;
    boost::signal<void()> sigRotationFinished_;
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
    void setContainerMode(bool on);
    bool isContainerMode() const;
    void setDraggerAlwaysShown(bool on);
    bool isDraggerAlwaysShown() const;

    TranslationDragger* translationDragger() { return translationDragger_; }
    RotationDragger* rotationDragger() { return rotationDragger_; }

    SignalProxy< boost::signal<void()> > sigDragStarted() {
        return sigDragStarted_;
    }
    SignalProxy< boost::signal<void()> > sigPositionDragged() {
        return sigPositionDragged_;
    }
    SignalProxy< boost::signal<void()> > sigDragFinished() {
        return sigDragFinished_;
    }

    Affine3 draggedPosition() const;

    virtual bool onButtonPressEvent(const SceneWidgetEvent& event);
    virtual bool onButtonReleaseEvent(const SceneWidgetEvent& event);
    virtual bool onPointerMoveEvent(const SceneWidgetEvent& event);
    virtual void onPointerLeaveEvent(const SceneWidgetEvent& event);
    virtual void onFocusChanged(const SceneWidgetEvent& event, bool on);
        
private:
    TranslationDraggerPtr translationDragger_;
    RotationDraggerPtr rotationDragger_;
    SceneDragProjector dragProjector;
    bool isContainerMode_;
    bool isDraggerAlwaysShown_;
    boost::signal<void()> sigDragStarted_;
    boost::signal<void()> sigPositionDragged_;
    boost::signal<void()> sigDragFinished_;

    void initalizeDraggers();
    void onPositionDragged();
};
    
typedef ref_ptr<PositionDragger> PositionDraggerPtr;
}

#endif
