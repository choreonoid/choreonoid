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
    TranslationDragger(const TranslationDragger& org, CloneMap* cloneMap = nullptr);

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

    virtual bool isDragEnabled() const override;
    virtual void setDragEnabled(bool on) override;

    virtual bool isDragging() const override;
    virtual Affine3 draggedPosition() const override;

    const Vector3& draggedTranslation() const;

    virtual bool onButtonPressEvent(const SceneWidgetEvent& event) override;
    virtual bool onButtonReleaseEvent(const SceneWidgetEvent& event) override;
    virtual bool onPointerMoveEvent(const SceneWidgetEvent& event) override;
    virtual void onPointerLeaveEvent(const SceneWidgetEvent& event) override;

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    int draggableAxes_;
    SgScaleTransformPtr defaultAxesScale;
    SgGroupPtr customAxes;
    SgMaterialPtr axisMaterials[3];
    SceneDragProjector dragProjector;
    bool isDragEnabled_;
    double axisCylinderNormalizedRadius;
    Signal<void()> sigTranslationStarted_;
    Signal<void()> sigTranslationDragged_;
    Signal<void()> sigTranslationFinished_;
};

typedef ref_ptr<TranslationDragger> TranslationDraggerPtr;

}

#endif
