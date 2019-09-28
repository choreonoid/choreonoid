/**
   @author Shin'ichiro Nakaoka
*/

#include "TranslationDragger.h"
#include <cnoid/SceneUtil>
#include <cnoid/MeshGenerator>
#include <cnoid/MeshExtractor>
#include <cnoid/EigenUtil>

using namespace std;
using namespace cnoid;

namespace {

const char* axisNames[3] = { "x", "y", "z" };

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
        SgMeshPtr mesh = meshGenerator.generateArrow(0.04, 1.8, 0.1, 0.18);
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
}


TranslationDragger::TranslationDragger(const TranslationDragger& org, SgCloneMap* cloneMap)
    : SceneDragger(org, cloneMap)
{
    draggableAxes_ = org.draggableAxes_;

    if(cloneMap){
        defaultAxesScale = getChild<SgScaleTransform>(0);
    } else {
        defaultAxesScale = new SgScaleTransform;
        defaultAxesScale->setScale(org.defaultAxesScale->scale());
        org.defaultAxesScale->copyChildrenTo(defaultAxesScale);
        addChild(defaultAxesScale);
    }
        
    axisCylinderNormalizedRadius = org.axisCylinderNormalizedRadius;
}


SgObject* TranslationDragger::doClone(SgCloneMap* cloneMap) const
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


bool TranslationDragger::isDragging() const
{
    return dragProjector.isDragging();
}


const Vector3& TranslationDragger::draggedTranslation() const
{
    return dragProjector.translation();
}


Affine3 TranslationDragger::draggedPosition() const
{
    return dragProjector.position();
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


bool TranslationDragger::onButtonReleaseEvent(const SceneWidgetEvent&)
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
        if(isContainerMode()){
            setPosition(dragProjector.position());
            notifyUpdate();
        }
        sigTranslationDragged_();
        return true;
    }
    return false;
}


void TranslationDragger::onPointerLeaveEvent(const SceneWidgetEvent&)
{
    if(dragProjector.isDragging()){
        sigTranslationFinished_();
        dragProjector.resetDragMode();
    }
}
