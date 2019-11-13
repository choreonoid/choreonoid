/**
   @author Shin'ichiro Nakaoka
*/

#include "RotationDragger.h"
#include <cnoid/SceneDrawables>
#include <cnoid/SceneWidget>
#include <cnoid/SceneRenderer>
#include <cnoid/SceneUtil>
#include <cnoid/MeshGenerator>
#include <cnoid/EigenUtil>

using namespace std;
using namespace cnoid;

namespace {

const char* axisNames[3] = { "x", "y", "z" };

/**
   \note This node is not inserted the node path obtained by SceneWidgetEvent::nodePath()
*/
class SgViewpointDependentSelector : public SgGroup
{
    Vector3 axis;
    double thresh;
    
public:
    SgViewpointDependentSelector()
        : SgGroup(findPolymorphicId<SgViewpointDependentSelector>()) {
        axis = Vector3::UnitX();
        thresh = cos(radian(45.0));
    }

    SgViewpointDependentSelector(const SgViewpointDependentSelector& org, CloneMap* cloneMap)
        : SgGroup(org, cloneMap) {
        axis = org.axis;
        thresh = org.thresh;
    }

    virtual Referenced* doClone(CloneMap* cloneMap) const override {
        return new SgViewpointDependentSelector(*this, cloneMap);
    }

    void setAxis(const Vector3& axis){
        this->axis = axis;
    }

    void setSwitchAngle(double rad){
        thresh = cos(rad);
    }

    void render(SceneRenderer* renderer) {
        const Affine3& C = renderer->currentCameraPosition();
        const Affine3& M = renderer->currentModelTransform();
        double d = fabs((C.translation() - M.translation()).normalized().dot((M.linear() * axis).normalized()));
        if(d > thresh){
            if(numChildren() > 0){
                renderer->renderNode(child(0));
            }
        } else {
            if(numChildren() > 1){
                renderer->renderNode(child(1));
            }
        }
    }
};


struct NodeTypeRegistration {
    NodeTypeRegistration() {
        SgNode::registerType<SgViewpointDependentSelector, SgGroup>();

        SceneRenderer::addExtension(
            [](SceneRenderer* renderer){
                auto functions = renderer->renderingFunctions();
                functions->setFunction<SgViewpointDependentSelector>(
                    [=](SgNode* node){
                        static_cast<SgViewpointDependentSelector*>(node)->render(renderer);
                    });
            });
    }
} registration;

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
        
        SgViewpointDependentSelector* selector = new SgViewpointDependentSelector;
        
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

    isDragEnabled_ = true;
}


RotationDragger::RotationDragger(const RotationDragger& org, CloneMap* cloneMap)
    : SceneDragger(org, cloneMap)
{
    draggableAxes_ = org.draggableAxes_;

    if(cloneMap){
        scale = getChild<SgScaleTransform>(0);
    } else {
        scale = new SgScaleTransform;
        scale->setScale(org.scale->scale());
        org.scale->copyChildrenTo(scale);
        addChild(scale);
    }

    isDragEnabled_ = org.isDragEnabled_;
}


Referenced* RotationDragger::doClone(CloneMap* cloneMap) const
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


bool RotationDragger::isDragEnabled() const
{
    return isDragEnabled_;
}


void RotationDragger::setDragEnabled(bool on)
{
    isDragEnabled_ = on;
}


bool RotationDragger::isDragging() const
{
    return dragProjector.isDragging();
}


const AngleAxis& RotationDragger::draggedAngleAxis() const
{
    return dragProjector.rotationAngleAxis();
}


Affine3 RotationDragger::draggedPosition() const
{
    return dragProjector.position();
}


bool RotationDragger::onButtonPressEvent(const SceneWidgetEvent& event)
{
    if(!isDragEnabled_){
        return false;
    }

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


bool RotationDragger::onButtonReleaseEvent(const SceneWidgetEvent&)
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
        if(isContainerMode()){
            setPosition(dragProjector.position());
            notifyUpdate();
        }
        sigRotationDragged_(dragProjector.rotationAngleAxis());
        return true;
    }
    return false;
}


void RotationDragger::onPointerLeaveEvent(const SceneWidgetEvent&)
{
    if(dragProjector.isDragging()){
        sigRotationFinished_();
        dragProjector.resetDragMode();
    }
}
