/**
   @author Shin'ichiro Nakaoka
*/

#include "SceneDragger.h"
#include <cnoid/SceneWidget>
#include <cnoid/SceneRenderer>
#include <cnoid/SceneUtil>
#include <cnoid/MeshGenerator>
#include <cnoid/MeshExtractor>
#include <cnoid/EigenUtil>
#include <boost/bind.hpp>
#include <iostream>

using namespace std;
using namespace cnoid;

namespace {

const char* axisNames[3] = { "x", "y", "z" };

/**
   \note This node is not inserted the node path obtained by SceneWidgetEvent::nodePath()
*/
class ViewpointDependentRenderingSelector : public SgGroup
{
    Vector3 axis;
    double thresh;
    
public:
    ViewpointDependentRenderingSelector(){
        axis = Vector3::UnitX();
        thresh = cos(radian(45.0));
    }

    ViewpointDependentRenderingSelector(const ViewpointDependentRenderingSelector& org, SgCloneMap& cloneMap)
        : SgGroup(org, cloneMap) {
        axis = org.axis;
        thresh = org.thresh;
    }

    virtual SgObject* clone(SgCloneMap& cloneMap) const {
        return new ViewpointDependentRenderingSelector(*this, cloneMap);
    }

    void setAxis(const Vector3& axis){
        this->axis = axis;
    }

    void setSwitchAngle(double rad){
        thresh = cos(rad);
    }

    virtual void accept(SceneVisitor& visitor) {
        SceneRenderer* renderer = dynamic_cast<SceneRenderer*>(&visitor);
        if(!renderer){
            visitor.visitGroup(this);
        } else {
            const Affine3& C = renderer->currentCameraPosition();
            const Affine3& M = renderer->currentModelTransform();
            bool isPerspetiveCamera = (renderer->projectionMatrix()(3, 3) == 0.0);
            if(isPerspetiveCamera){
                double d = fabs((C.translation() - M.translation()).normalized().dot((M.linear() * axis).normalized()));
                if(d > thresh){
                    if(numChildren() > 0){
                        child(0)->accept(visitor);
                    }
                } else {
                    if(numChildren() > 1){
                        child(1)->accept(visitor);
                    }
                }
            } else {

            }
        }
    }
};

}


TranslationDragger::TranslationDragger(bool setDefaultAxes)
{
    draggableAxes_ = TX | TY | TZ;
    
    axisCylinderNormalizedRadius = 0.04;

    defaultAxesScale = new SgScaleTransform;
    customAxes = new SgGroup;

    for(int i=0; i < 3; ++i){
        SgMaterial* material = new SgMaterial;
        Vector3f color(0.2f, 0.2f, 0.2f);
        color[i] = 1.0f;
        material->setDiffuseColor(Vector3f::Zero());
        material->setEmissiveColor(color);
        material->setAmbientIntensity(0.0f);
        material->setTransparency(0.6f);
        axisMaterials[i] = material;
    }

    if(setDefaultAxes){
        MeshGenerator meshGenerator;
        SgMeshPtr mesh = meshGenerator.generateArrow(1.8, 0.08, 0.1, 2.5);
        for(int i=0; i < 3; ++i){
            SgShape* shape = new SgShape;
            shape->setMesh(mesh);
            shape->setMaterial(axisMaterials[i]);
            
            SgPosTransform* arrow = new SgPosTransform;
            arrow->addChild(shape);
            if(i == 0){
                arrow->setRotation(AngleAxis(-PI / 2.0, Vector3::UnitZ()));
            } else if(i == 2){
                arrow->setRotation(AngleAxis( PI / 2.0, Vector3::UnitX()));
            }
            SgInvariantGroup* invariant = new SgInvariantGroup;
            invariant->setName(axisNames[i]);
            invariant->addChild(arrow);
            SgSwitch* axis = new SgSwitch;
            axis->addChild(invariant);
            defaultAxesScale->addChild(axis);
        }
        addChild(defaultAxesScale);
    }

    isContainerMode_ = false;
}


TranslationDragger::TranslationDragger(const TranslationDragger& org)
{
    draggableAxes_ = org.draggableAxes_;
    defaultAxesScale = new SgScaleTransform;
    defaultAxesScale->setScale(org.defaultAxesScale->scale());
    org.defaultAxesScale->copyChildrenTo(defaultAxesScale);
    addChild(defaultAxesScale);

    axisCylinderNormalizedRadius = org.axisCylinderNormalizedRadius;
    isContainerMode_ = org.isContainerMode_;
}


TranslationDragger::TranslationDragger(const TranslationDragger& org, SgCloneMap& cloneMap)
    : SgPosTransform(org, cloneMap)
{
    draggableAxes_ = org.draggableAxes_;
    defaultAxesScale = getChild<SgScaleTransform>(0);
    axisCylinderNormalizedRadius = org.axisCylinderNormalizedRadius;
    isContainerMode_ = org.isContainerMode_;
}


SgObject* TranslationDragger::clone(SgCloneMap& cloneMap) const
{
    return new TranslationDragger(*this, cloneMap);
}


void TranslationDragger::setDraggableAxes(int axisSet)
{
    if(axisSet != draggableAxes_){
        for(int i=0; i < 3; ++i){
            SgSwitch* axis = dynamic_cast<SgSwitch*>(defaultAxesScale->child(i));
            if(axis){
                axis->setTurnedOn(axisSet & (1 << i));
            }
        }
        draggableAxes_ = axisSet;
        defaultAxesScale->notifyUpdate();
    }
}


void TranslationDragger::addCustomAxis(int axis, SgNode* node)
{
    SgInvariantGroup* invariant = new SgInvariantGroup;
    invariant->setName(axisNames[axis]);
    invariant->addChild(node);
    customAxes->addChild(invariant);
    addChildOnce(customAxes);
}


void TranslationDragger::clearCustomAxes()
{
    customAxes->clearChildren();
}


double TranslationDragger::radius() const
{
    return defaultAxesScale->scale().x();
}


void TranslationDragger::setRadius(double r)
{
    defaultAxesScale->setScale(r);
}


void TranslationDragger::setContainerMode(bool on)
{
    isContainerMode_ = on;
}


bool TranslationDragger::isContainerMode() const
{
    return isContainerMode_;
}


bool TranslationDragger::isDragging() const
{
    return dragProjector.isDragging();
}


const Vector3& TranslationDragger::draggedTranslation() const
{
    return dragProjector.translation();
}


const Affine3& TranslationDragger::draggedPosition() const
{
    return dragProjector.position();
}


static bool detectAxisFromNodePath(const SgNodePath& path, SgNode* topNode, int& out_axis, int& out_indexOfTopNode)
{
    for(size_t i=0; i < path.size(); ++i){
        const SgNode* node = path[i];
        if(node == topNode){
            out_indexOfTopNode = i;
        }
        if(out_indexOfTopNode >= 0){
            const string& name = node->name();
            if(!name.empty()){
                for(int j=0; j < 3; ++j){
                     if(name == axisNames[j]){
                        out_axis = j;
                        return true;
                    }
                }
            }
        }
    }
    return false;
}


bool TranslationDragger::onButtonPressEvent(const SceneWidgetEvent& event)
{
    int axis;
    int indexOfTopNode;
    const SgNodePath& path = event.nodePath();
    if(detectAxisFromNodePath(path, this, axis, indexOfTopNode)){
        SgNodePath::const_iterator axisIter = path.begin() + indexOfTopNode + 1;
        const Affine3 T_global = calcTotalTransform(path.begin(), axisIter);
        dragProjector.setInitialPosition(T_global);
        const Affine3 T_axis = calcTotalTransform(axisIter, path.end());
        const Vector3 p_local = (T_global * T_axis).inverse() * event.point();
        if(p_local.norm() < 2.0 * axisCylinderNormalizedRadius){
            dragProjector.setTranslationAlongViewPlane();
        } else {
            dragProjector.setTranslationAxis(T_global.linear().col(axis));
        }
        if(dragProjector.startTranslation(event)){
            sigTranslationStarted_();
            return true;
        }
    }
    return false;
}


bool TranslationDragger::onButtonReleaseEvent(const SceneWidgetEvent& event)
{
    if(dragProjector.isDragging()){
        sigTranslationFinished_();
        dragProjector.resetDragMode();
        return true;
    }
    return false;
}


bool TranslationDragger::onPointerMoveEvent(const SceneWidgetEvent& event)
{
    if(dragProjector.dragTranslation(event)){
        if(isContainerMode_){
            setPosition(dragProjector.position());
            notifyUpdate();
        }
        sigTranslationDragged_();
        return true;
    }
    return false;
}


void TranslationDragger::onPointerLeaveEvent(const SceneWidgetEvent& event)
{
    if(dragProjector.isDragging()){
        sigTranslationFinished_();
        dragProjector.resetDragMode();
    }
}


RotationDragger::RotationDragger()
{
    draggableAxes_ = RX | RY | RZ;

    MeshGenerator meshGenerator;
    meshGenerator.setDivisionNumber(36);
    SgMeshPtr beltMesh1 = meshGenerator.generateDisc(1.0, 1.0 - 0.2);
    meshGenerator.setDivisionNumber(24);
    SgMeshPtr beltMesh2 = meshGenerator.generateCylinder(1.0, 0.2, false, false);

    scale = new SgScaleTransform;
    
    // make dragger belts
    for(int i=0; i < 3; ++i){
        
        SgMaterial* material = new SgMaterial;
        Vector3f color(0.2f, 0.2f, 0.2f);
        color[i] = 1.0f;
        material->setDiffuseColor(Vector3f::Zero());
        material->setEmissiveColor(color);
        material->setAmbientIntensity(0.0f);
        material->setTransparency(0.6f);

        SgShape* beltShape1 = new SgShape;
        beltShape1->setMesh(beltMesh1);
        beltShape1->setMaterial(material);

        SgShape* beltShape2 = new SgShape;
        beltShape2->setMesh(beltMesh2);
        beltShape2->setMaterial(material);
        
        ViewpointDependentRenderingSelector* selector = new ViewpointDependentRenderingSelector;
        
        SgPosTransform* transform1 = new SgPosTransform;
        if(i == 0){ // x-axis
            selector->setAxis(Vector3::UnitX());
            transform1->setRotation(AngleAxis(PI / 2.0, Vector3::UnitY()));
        } else if(i == 1){ // y-axis
            selector->setAxis(Vector3::UnitY());
            transform1->setRotation(AngleAxis(-PI / 2.0, Vector3::UnitX()));
        } else if(i == 2) { // z-axis
            selector->setAxis(Vector3::UnitZ());
        }
        transform1->addChild(beltShape1);
        SgInvariantGroup* belt1 = new SgInvariantGroup;
        belt1->setName(axisNames[i]);
        belt1->addChild(transform1);
        selector->addChild(belt1);

        SgPosTransform* transform2 = new SgPosTransform;
        if(i == 0){ // x-axis
            transform2->setRotation(AngleAxis(-PI / 2.0, Vector3::UnitZ()));
        } else if(i == 2) { // z-axis
            transform2->setRotation(AngleAxis(PI / 2.0, Vector3::UnitX()));
        }
        transform2->addChild(beltShape2);
        SgInvariantGroup* belt2 = new SgInvariantGroup;
        belt2->setName(axisNames[i]);
        belt2->addChild(transform2);
        selector->addChild(belt2);

        SgSwitch* axis = new SgSwitch;
        axis->addChild(selector);

        scale->addChild(axis);
    }

    addChild(scale);

    isContainerMode_ = false;
}


RotationDragger::RotationDragger(const RotationDragger& org)
{
    draggableAxes_ = org.draggableAxes_;
    scale = new SgScaleTransform;
    scale->setScale(org.scale->scale());
    org.scale->copyChildrenTo(scale);
    addChild(scale);
    isContainerMode_ = org.isContainerMode_;
}


RotationDragger::RotationDragger(const RotationDragger& org, SgCloneMap& cloneMap)
    : SgPosTransform(org, cloneMap)
{
    draggableAxes_ = org.draggableAxes_;
    scale = getChild<SgScaleTransform>(0);
    isContainerMode_ = org.isContainerMode_;
}


SgObject* RotationDragger::clone(SgCloneMap& cloneMap) const
{
    return new RotationDragger(*this, cloneMap);
}


void RotationDragger::setDraggableAxes(int axisSet)
{
    if(axisSet != draggableAxes_){
        for(int i=0; i < 3; ++i){
            SgSwitch* axis = dynamic_cast<SgSwitch*>(scale->child(i));
            if(axis){
                axis->setTurnedOn(axisSet & (1 << i));
            }
        }
        draggableAxes_ = axisSet;
        scale->notifyUpdate();
    }
}


void RotationDragger::setRadius(double r)
{
    scale->setScale(r);
}


void RotationDragger::setContainerMode(bool on)
{
    isContainerMode_ = on;
}


bool RotationDragger::isContainerMode() const
{
    return isContainerMode_;
}


bool RotationDragger::isDragging() const
{
    return dragProjector.isDragging();
}


const AngleAxis& RotationDragger::draggedAngleAxis() const
{
    return dragProjector.rotationAngleAxis();
}


const Affine3& RotationDragger::draggedPosition() const
{
    return dragProjector.position();
}


bool RotationDragger::onButtonPressEvent(const SceneWidgetEvent& event)
{
    int axis;
    int indexOfTopNode;
    if(detectAxisFromNodePath(event.nodePath(), this, axis, indexOfTopNode)){
        Affine3 T = calcTotalTransform(event.nodePath(), this);
        dragProjector.setInitialPosition(T);
        dragProjector.setRotationAxis(T.linear().col(axis));
        if(dragProjector.startRotation(event)){
            sigRotationStarted_();
            return true;
        }
    }
    return false;
}


bool RotationDragger::onButtonReleaseEvent(const SceneWidgetEvent& event)
{
    if(dragProjector.isDragging()){
        sigRotationFinished_();
        dragProjector.resetDragMode();
        return true;
    }
    return false;
}


bool RotationDragger::onPointerMoveEvent(const SceneWidgetEvent& event)
{
    if(dragProjector.dragRotation(event)){
        if(isContainerMode_){
            setPosition(dragProjector.position());
            notifyUpdate();
        }
        sigRotationDragged_(dragProjector.rotationAngleAxis());
        return true;
    }
    return false;
}


void RotationDragger::onPointerLeaveEvent(const SceneWidgetEvent& event)
{
    if(dragProjector.isDragging()){
        sigRotationFinished_();
        dragProjector.resetDragMode();
    }
}


PositionDragger::PositionDragger()
{
    translationDragger_ = new TranslationDragger;
    rotationDragger_ = new RotationDragger;
    draggableAxes_ = TX | TY | TZ | RX | RY | RZ;

    initalizeDraggers();
    
    isDraggerAlwaysShown_ = false;
    isDraggerAlwaysHidden_ = false;
    isContainerMode_ = false;
    isContentsDragEnabled_ = true;
    isUndoEnabled_ = false;    
}


PositionDragger::PositionDragger(const PositionDragger& org)
{
    translationDragger_ = new TranslationDragger(*org.translationDragger_);
    rotationDragger_ = new RotationDragger(*org.rotationDragger_);
    draggableAxes_ = org.draggableAxes_;

    initalizeDraggers();

    isContainerMode_ = org.isContainerMode_;
    isDraggerAlwaysShown_ = org.isDraggerAlwaysShown_;
    isDraggerAlwaysHidden_ = org.isDraggerAlwaysHidden_;
    isContentsDragEnabled_ = org.isContentsDragEnabled_;
    isUndoEnabled_ = org.isUndoEnabled_;
}


PositionDragger::PositionDragger(const PositionDragger& org, SgCloneMap& cloneMap)
{
    translationDragger_ = new TranslationDragger(*org.translationDragger_, cloneMap);
    rotationDragger_ = new RotationDragger(*org.rotationDragger_, cloneMap);
    draggableAxes_ = org.draggableAxes_;

    initalizeDraggers();

    isContainerMode_ = org.isContainerMode_;
    isDraggerAlwaysShown_ = org.isDraggerAlwaysShown_;
    isDraggerAlwaysHidden_ = org.isDraggerAlwaysHidden_;
    isContentsDragEnabled_ = org.isContentsDragEnabled_;
    isUndoEnabled_ = org.isUndoEnabled_;
}


void PositionDragger::initalizeDraggers()
{
    translationDragger_->sigTranslationStarted().connect(
        boost::bind(&PositionDragger::onSubDraggerDragStarted, this));
    translationDragger_->sigTranslationDragged().connect(
        boost::bind(&PositionDragger::onSubDraggerDragged, this));
    translationDragger_->sigTranslationFinished().connect(boost::ref(sigDragFinished_));
    
    rotationDragger_->sigRotationStarted().connect(
        boost::bind(&PositionDragger::onSubDraggerDragStarted, this));
    rotationDragger_->sigRotationDragged().connect(
        boost::bind(&PositionDragger::onSubDraggerDragged, this));
    rotationDragger_->sigRotationFinished().connect(boost::ref(sigDragFinished_));
}


void PositionDragger::setDraggableAxes(int axisSet)
{
    if(axisSet != draggableAxes_){
        int translationAxes = axisSet & (TX | TY | TZ);
        translationDragger_->setDraggableAxes(translationAxes);
        int rotationAxes = (axisSet & (RX | RY | RZ)) >> 3;
        rotationDragger_->setDraggableAxes(rotationAxes);
        draggableAxes_ = axisSet;
        sigDraggableAxesChanged_(axisSet);
    }
}


SgObject* PositionDragger::clone(SgCloneMap& cloneMap) const
{
    return new PositionDragger(*this, cloneMap);
}


void PositionDragger::setRadius(double r, double translationAxisRatio)
{
    translationDragger_->setRadius(r * translationAxisRatio);
    rotationDragger_->setRadius(r);
}


void PositionDragger::adjustSize(const BoundingBox& bb)
{
    if(!bb.empty()){
        Vector3 s = bb.size() / 2.0;
        std::sort(s.data(), s.data() + 3);
        double a = Vector2(s[0], s[1]).norm() * 1.1;
        double r = std::max(a, s[2] * 1.2);
        setRadius(r);
    }
}


void PositionDragger::adjustSize()
{
    BoundingBox bb;
    for(int i=0; i < numChildren(); ++i){
        SgNode* node = child(i);
        if(node != translationDragger_ && node != rotationDragger_){
            bb.expandBy(node->boundingBox());
        }
    }
    adjustSize(bb);
}


void PositionDragger::setContainerMode(bool on)
{
    isContainerMode_ = on;
}


bool PositionDragger::isContainerMode() const
{
    return isContainerMode_;
}


void PositionDragger::setContentsDragEnabled(bool on)
{
    isContentsDragEnabled_ = on;
}


bool PositionDragger::isContentsDragEnabled() const
{
    return isContentsDragEnabled_;
}


void PositionDragger::setDraggerAlwaysShown(bool on)
{
    if(on){
        isDraggerAlwaysHidden_ = false;
    }
    bool changed = (on != isDraggerAlwaysShown_);
    isDraggerAlwaysShown_ = on;
    if(on && changed){
        showDragMarkers(true);
    }
}


bool PositionDragger::isDraggerAlwaysShown() const
{
    return isDraggerAlwaysShown_;
}


void PositionDragger::setDraggerAlwaysHidden(bool on)
{
    if(on){
        isDraggerAlwaysShown_ = false;
    }
    bool changed = (on != isDraggerAlwaysHidden_);
    isDraggerAlwaysHidden_ = on;
    if(on && changed){
        showDragMarkers(false);
    }
}


bool PositionDragger::isDraggerAlwaysHidden() const
{
    return isDraggerAlwaysHidden_;
}


void PositionDragger::showDragMarkers(bool on)
{
    if(isDraggerAlwaysHidden_){
        on = false;
    } else if(isDraggerAlwaysShown_){
        on = true;
    }
    
    if(on){
        addChildOnce(translationDragger_, true);
        addChildOnce(rotationDragger_, true);
    } else {
        removeChild(translationDragger_, true);
        removeChild(rotationDragger_, true);
    }
}    


bool PositionDragger::isDragging() const
{
    return (translationDragger_->isDragging() ||
            rotationDragger_->isDragging() ||
            dragProjector.isDragging());
}


Affine3 PositionDragger::draggedPosition() const
{
    if(rotationDragger_->isDragging()){
        return rotationDragger_->draggedPosition();
    } else if(translationDragger_->isDragging()){
        return translationDragger_->draggedPosition();
    } else if(dragProjector.isDragging()){
        return dragProjector.position();
    } else {
        return T();
    }
}


void PositionDragger::onSubDraggerDragStarted()
{
    storeCurrentPositionToHistory();
    sigDragStarted_();
}


void PositionDragger::onSubDraggerDragged()
{
    if(isContainerMode_){
        if(isContentsDragEnabled_){
            setPosition(draggedPosition());
            notifyUpdate();
            sigPositionDragged_();
        }
    } else {
        sigPositionDragged_();
    }
}


bool PositionDragger::onButtonPressEvent(const SceneWidgetEvent& event)
{
    if(isContainerMode_ && isContentsDragEnabled_){
        if(!isDraggerAlwaysShown_){
            showDragMarkers(true);
        }
        dragProjector.setInitialPosition(T());
        dragProjector.setTranslationAlongViewPlane();
        if(dragProjector.startTranslation(event)){
            storeCurrentPositionToHistory();
            sigDragStarted_();
            return true;
        }
    }
    return false;
}


bool PositionDragger::onButtonReleaseEvent(const SceneWidgetEvent& event)
{
    if(isContainerMode_ && isContentsDragEnabled_){
        if(dragProjector.isDragging()){
            sigDragFinished_();
            dragProjector.resetDragMode();
            return true;
        }
    }
    return false;
}


bool PositionDragger::onPointerMoveEvent(const SceneWidgetEvent& event)
{
    if(isContainerMode_ && isContentsDragEnabled_){
        if(dragProjector.drag(event)){
            setPosition(dragProjector.position());
            notifyUpdate();
            sigPositionDragged_();
            return true;
        }
    }
    return false;
}


void PositionDragger::onPointerLeaveEvent(const SceneWidgetEvent& event)
{
    if(isContainerMode_ && isContentsDragEnabled_){
        dragProjector.resetDragMode();
    }
}


void PositionDragger::onFocusChanged(const SceneWidgetEvent& event, bool on)
{
    if(isContainerMode_){
        showDragMarkers(on || isDraggerAlwaysShown_);
    }
}


void PositionDragger::onSceneModeChanged(const SceneWidgetEvent& event)
{
    if(!event.sceneWidget()->isEditMode()){
        showDragMarkers(false);
    }
}


void PositionDragger::storeCurrentPositionToHistory()
{
    if(isUndoEnabled_){
        history.push_back(position());
        if(history.size() > 10){
            history.pop_front();
        }
    }
}


void PositionDragger::setUndoEnabled(bool on)
{
    isUndoEnabled_ = on;
}


bool PositionDragger::isUndoEnabled() const
{
    return isUndoEnabled_;
}


bool PositionDragger::onUndoRequest()
{
    if(!history.empty()){
        const Affine3& T = history.back();
        setPosition(T);
        history.pop_back();
        notifyUpdate();
    }
}


bool PositionDragger::onRedoRequest()
{

}
