/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_TRANSLATION_DRAGGER_H
#define CNOID_BASE_TRANSLATION_DRAGGER_H

#include "SceneDragger.h"
#include "SceneDragProjector.h"
#include <cnoid/SceneDrawables>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT TranslationDragger : public SceneDragger
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TranslationDragger(bool setDefaultAxes = true);
    TranslationDragger(const TranslationDragger& org);
    TranslationDragger(const TranslationDragger& org, SgCloneMap& cloneMap);

    virtual SgObject* clone(SgCloneMap& cloneMap) const;

    enum Axis { TX = 1, TY = 2, TZ = 4 };

    void setDraggableAxes(int axisSet);
    int draggableAxes() const { return draggableAxes_; }

    void addCustomAxis(int axis, SgNode* node);
    void clearCustomAxes();

    double radius() const;
    void setRadius(double r);
    
    SignalProxy<void()> sigTranslationStarted() {
        return sigTranslationStarted_;
    }
    SignalProxy<void()> sigTranslationDragged() {
        return sigTranslationDragged_;
    }
    SignalProxy<void()> sigTranslationFinished() {
        return sigTranslationFinished_;
    }

    virtual bool isDragging() const;
    virtual Affine3 draggedPosition() const;

    const Vector3& draggedTranslation() const;

    virtual bool onButtonPressEvent(const SceneWidgetEvent& event);
    virtual bool onButtonReleaseEvent(const SceneWidgetEvent& event);
    virtual bool onPointerMoveEvent(const SceneWidgetEvent& event);
    virtual void onPointerLeaveEvent(const SceneWidgetEvent& event);
        
private:
    int draggableAxes_;
    SgScaleTransformPtr defaultAxesScale;
    SgGroupPtr customAxes;
    SgMaterialPtr axisMaterials[3];
    SceneDragProjector dragProjector;
    double axisCylinderNormalizedRadius;
    Signal<void()> sigTranslationStarted_;
    Signal<void()> sigTranslationDragged_;
    Signal<void()> sigTranslationFinished_;
};

typedef ref_ptr<TranslationDragger> TranslationDraggerPtr;

}

#endif
