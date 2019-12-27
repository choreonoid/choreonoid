/**
   @author Shin'ichiro Nakaoka
*/

#include "PositionDragger.h"
#include "SceneDragProjector.h"
#include "SceneWidget.h"
#include <cnoid/SceneNodeClassRegistry>
#include <cnoid/SceneRenderer>
#include <cnoid/SceneUtil>
#include <cnoid/MeshGenerator>
#include <cnoid/EigenUtil>
#include <cnoid/CloneMap>
#include <deque>
#include <array>

using namespace std;
using namespace cnoid;

namespace {

const char* axisNames[6] = { "tx", "ty", "tz", "rx", "ry", "rz" };

constexpr double axisCylinderRadius = 0.04;

/**
   \note This node is not inserted the node path obtained by SceneWidgetEvent::nodePath()
*/
class SgViewpointDependentSelector : public SgGroup
{
    Vector3 axis;
    double thresh;
    
public:
    SgViewpointDependentSelector()
        : SgGroup(findClassId<SgViewpointDependentSelector>()) {
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


struct NodeClassRegistration {
    NodeClassRegistration() {
        SceneNodeClassRegistry::instance().registerClass<SgViewpointDependentSelector, SgGroup>();

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

namespace cnoid {

class PositionDragger::Impl
{
public:
    PositionDragger* self;
    int draggableAxes;
    Signal<void(int axisSet)> sigDraggableAxesChanged;
    DisplayMode displayMode;
    bool isOverlayMode;
    bool isContainerMode;
    bool isDragEnabled;
    bool isContentsDragEnabled;
    bool isUndoEnabled;
    SgGroupPtr axisGroup;
    SgOverlayPtr overlay;
    SgScaleTransformPtr translationAxisScale;
    SgScaleTransformPtr rotationAxisScale;
    double rotationHandleSizeRatio;
    array<SgMaterialPtr, 3> axisMaterials;
    SceneDragProjector dragProjector;
    Signal<void()> sigDragStarted;
    Signal<void()> sigPositionDragged;
    Signal<void()> sigDragFinished;
    std::deque<Affine3> history;

    Impl(PositionDragger* self, int axisSet);
    Impl(PositionDragger* self, const Impl& org);
    Impl(PositionDragger* self, const Impl& org, CloneMap* cloneMap);
    void createDraggers();
    void createTranslationDragger(MeshGenerator& meshGenerator);
    void createRotationDragger(MeshGenerator& meshGenerator);
    void setDraggableAxes(int axisSet);
    void setOverlayMode(bool on);
    void showDragMarkers(bool on);
    bool onButtonPressEvent(const SceneWidgetEvent& event);
    bool onTranslationDraggerPressed(const SceneWidgetEvent& event, int axis, int topNodeIndex);
    bool onRotationDraggerPressed(const SceneWidgetEvent& event, int axis);
    void storeCurrentPositionToHistory();
};

}


PositionDragger::PositionDragger()
    : PositionDragger(ALL_AXES)
{

}


PositionDragger::PositionDragger(int axisSet)
{
    impl = new Impl(this, axisSet);
}


PositionDragger::Impl::Impl(PositionDragger* self, int axisSet)
    : self(self)
{
    draggableAxes = axisSet;
    rotationHandleSizeRatio = 0.5;
    axisGroup = self;

    createDraggers();

    displayMode = DisplayInFocus;
    isOverlayMode = false;
    isContainerMode = false;
    isDragEnabled = true;
    isContentsDragEnabled = true;
    isUndoEnabled = false;    
}


PositionDragger::PositionDragger(const PositionDragger& org)
{
    impl = new Impl(this, *org.impl, nullptr);
}


PositionDragger::PositionDragger(const PositionDragger& org, CloneMap* cloneMap)
    : SgPosTransform(org, cloneMap)
{
    impl = new Impl(this, *org.impl, cloneMap);
}


Referenced* PositionDragger::doClone(CloneMap* cloneMap) const
{
    if(cloneMap){
        return new PositionDragger(*this, cloneMap);
    } else {
        return new PositionDragger(*this);
    }
}


PositionDragger::Impl::Impl(PositionDragger* self, const Impl& org, CloneMap* cloneMap)
    : self(self)
{
    draggableAxes = org.draggableAxes;
    rotationHandleSizeRatio = org.rotationHandleSizeRatio;
    isOverlayMode = org.isOverlayMode;
    isContainerMode = org.isContainerMode;
    isDragEnabled = org.isDragEnabled;
    isContentsDragEnabled = org.isContentsDragEnabled;
    isUndoEnabled = org.isUndoEnabled;

    axisGroup = self;
    if(cloneMap){
        axisGroup = cloneMap->findClone(org.axisGroup);
        translationAxisScale = cloneMap->getClone(org.translationAxisScale);
        rotationAxisScale = cloneMap->getClone(org.rotationAxisScale);
        for(int i=0; i < 3; ++i){
            axisMaterials[i] = cloneMap->getClone(org.axisMaterials[i]);
        }
        displayMode = org.displayMode;
    } else {
        createDraggers();
        displayMode = DisplayInFocus;
        self->setDisplayMode(org.displayMode);
    }
}


void PositionDragger::Impl::createDraggers()
{
    for(int i=0; i < 3; ++i){
        auto material = new SgMaterial;
        Vector3f color(0.2f, 0.2f, 0.2f);
        color[i] = 1.0f;
        material->setDiffuseColor(Vector3f::Zero());
        material->setEmissiveColor(color);
        material->setAmbientIntensity(0.0f);
        material->setTransparency(0.6f);
        axisMaterials[i] = material;
    }

    MeshGenerator meshGenerator;
    createTranslationDragger(meshGenerator);
    createRotationDragger(meshGenerator);
}

    
void PositionDragger::Impl::createTranslationDragger(MeshGenerator& meshGenerator)
{
    translationAxisScale = new SgScaleTransform;
    
    SgMeshPtr mesh = meshGenerator.generateArrow(0.04, 1.8, 0.1, 0.18);
    for(int i=0; i < 3; ++i){
        auto shape = new SgShape;
        shape->setMesh(mesh);
        shape->setMaterial(axisMaterials[i]);
            
        auto arrow = new SgPosTransform;
        arrow->addChild(shape);
        if(i == 0){
            arrow->setRotation(AngleAxis(-PI / 2.0, Vector3::UnitZ()));
        } else if(i == 2){
            arrow->setRotation(AngleAxis( PI / 2.0, Vector3::UnitX()));
        }
        arrow->setName(axisNames[i]);

        auto axis = new SgSwitch;
        axis->addChild(arrow);
        axis->setTurnedOn(draggableAxes & (1 << i));
        translationAxisScale->addChild(axis);
    }

    axisGroup->addChild(translationAxisScale);
}


void PositionDragger::Impl::createRotationDragger(MeshGenerator& meshGenerator)
{
    rotationAxisScale = new SgScaleTransform;

    meshGenerator.setDivisionNumber(36);
    auto beltMesh1 = meshGenerator.generateDisc(1.0, 1.0 - 0.2);
    meshGenerator.setDivisionNumber(24);
    auto beltMesh2 = meshGenerator.generateCylinder(1.0, 0.2, false, false);

    for(int i=0; i < 3; ++i){
        auto material = axisMaterials[i];
        
        auto beltShape1 = new SgShape;
        beltShape1->setMesh(beltMesh1);
        beltShape1->setMaterial(material);

        auto beltShape2 = new SgShape;
        beltShape2->setMesh(beltMesh2);
        beltShape2->setMaterial(material);
        
        auto selector = new SgViewpointDependentSelector;
        
        auto belt1 = new SgPosTransform;
        if(i == 0){ // x-axis
            selector->setAxis(Vector3::UnitX());
            belt1->setRotation(AngleAxis(PI / 2.0, Vector3::UnitY()));
        } else if(i == 1){ // y-axis
            selector->setAxis(Vector3::UnitY());
            belt1->setRotation(AngleAxis(-PI / 2.0, Vector3::UnitX()));
        } else if(i == 2) { // z-axis
            selector->setAxis(Vector3::UnitZ());
        }
        belt1->addChild(beltShape1);
        belt1->setName(axisNames[i + 3]);
        selector->addChild(belt1);

        auto belt2 = new SgPosTransform;
        if(i == 0){ // x-axis
            belt2->setRotation(AngleAxis(-PI / 2.0, Vector3::UnitZ()));
        } else if(i == 2) { // z-axis
            belt2->setRotation(AngleAxis(PI / 2.0, Vector3::UnitX()));
        }
        belt2->addChild(beltShape2);
        belt2->setName(axisNames[i + 3]);
        selector->addChild(belt2);

        auto axis = new SgSwitch;
        axis->addChild(selector);
        axis->setTurnedOn(draggableAxes & (1 << (i + 3)));

        rotationAxisScale->addChild(axis);
    }

    axisGroup->addChild(rotationAxisScale);
}


void PositionDragger::setDraggableAxes(int axisSet)
{
    impl->setDraggableAxes(axisSet);
}


void PositionDragger::Impl::setDraggableAxes(int axisSet)
{
    if(axisSet != draggableAxes){
        for(int i=0; i < 3; ++i){
            if(auto axis = dynamic_cast<SgSwitch*>(translationAxisScale->child(i))){
                axis->setTurnedOn(axisSet & (1 << i));
            }
            if(auto axis = dynamic_cast<SgSwitch*>(rotationAxisScale->child(i))){
                axis->setTurnedOn(axisSet & (1 << (i + 3)));
            }
        }
        draggableAxes = axisSet;
        self->notifyUpdate();
    }
}


int PositionDragger::draggableAxes() const
{
    return impl->draggableAxes;
}


SignalProxy<void(int axisSet)> PositionDragger::sigDraggableAxesChanged()
{
    return impl->sigDraggableAxesChanged;
}


double PositionDragger::handleSize() const
{
    return impl->translationAxisScale->scale().x();
}


void PositionDragger::setHandleSize(double s)
{
    impl->translationAxisScale->setScale(s);
    impl->rotationAxisScale->setScale(s * impl->rotationHandleSizeRatio);
}
    

double PositionDragger::rotationHandleSizeRatio() const
{
    return impl->rotationHandleSizeRatio;
}


void PositionDragger::setRotationHandlerSizeRatio(double r)
{
    impl->rotationHandleSizeRatio = r;
    impl->rotationAxisScale->setScale(handleSize() * r);
}


void PositionDragger::setRadius(double r, double translationAxisRatio)
{
    impl->rotationHandleSizeRatio = 1.0 / translationAxisRatio;
    setHandleSize(2.0 * r);
}


double PositionDragger::radius() const
{
    return handleSize() * impl->rotationHandleSizeRatio;
}


void PositionDragger::adjustSize(const BoundingBox& bb)
{
    if(!bb.empty()){
        Vector3 s = bb.size() / 2.0;
        std::sort(s.data(), s.data() + 3);
        double a = Vector2(s[0], s[1]).norm() * 1.1;
        double r = std::max(a, s[2] * 1.2);
        setHandleSize(2.0 * r);
    }
}


void PositionDragger::adjustSize()
{
    BoundingBox bb;
    for(int i=0; i < numChildren(); ++i){
        auto node = child(i);
        if(node != impl->translationAxisScale && node != impl->rotationAxisScale){
            bb.expandBy(node->boundingBox());
        }
    }
    adjustSize(bb);
}


void PositionDragger::setOverlayMode(bool on)
{
    impl->setOverlayMode(on);
}


void PositionDragger::Impl::setOverlayMode(bool on)
{
    if(on != isOverlayMode){
        double transparency;
        if(on){
            if(!overlay){
                overlay = new SgOverlay;
            }
            self->moveChildrenTo(overlay);
            self->addChild(overlay);
            axisGroup = overlay;
            transparency = 0.0f;
        } else {
            self->removeChild(overlay);
            overlay->moveChildrenTo(self);
            axisGroup = self;
            transparency = 1.0f;
        }
        for(int i=0; i < 3; ++i){
            axisMaterials[i]->setTransparency(transparency);
        }
        isOverlayMode = on;
    }
}


bool PositionDragger::isOverlayMode() const
{
    return impl->isOverlayMode;
}


bool PositionDragger::isContainerMode() const
{
    return impl->isContainerMode;
}


void PositionDragger::setContainerMode(bool on)
{
    impl->isContainerMode = on;
}


void PositionDragger::setContentsDragEnabled(bool on)
{
    impl->isContentsDragEnabled = on;
}


bool PositionDragger::isContentsDragEnabled() const
{
    return impl->isContentsDragEnabled;
}


PositionDragger::DisplayMode PositionDragger::displayMode() const
{
    return impl->displayMode;
}


void PositionDragger::setDisplayMode(DisplayMode mode)
{
    if(mode != impl->displayMode){
        impl->displayMode = mode;
        if(mode == DisplayAlways){
            impl->showDragMarkers(true);
        } else if(mode == DisplayNever){
            impl->showDragMarkers(false);
        }
    }
}


void PositionDragger::Impl::showDragMarkers(bool on)
{
    if(displayMode == DisplayNever){
        on = false;
    } else if(displayMode == DisplayAlways){
        on = true;
    }
    
    if(on){
        axisGroup->addChildOnce(translationAxisScale, true);
        axisGroup->addChildOnce(rotationAxisScale, true);
    } else {
        axisGroup->removeChild(translationAxisScale, true);
        axisGroup->removeChild(rotationAxisScale, true);
    }
}    


void PositionDragger::setDraggerAlwaysShown(bool on)
{
    if(on){
        setDisplayMode(DisplayAlways);
    }
}


bool PositionDragger::isDraggerAlwaysShown() const
{
    return (impl->displayMode == DisplayAlways);
}


void PositionDragger::setDraggerAlwaysHidden(bool on)
{
    if(on){
        setDisplayMode(DisplayNever);
    }
}


bool PositionDragger::isDraggerAlwaysHidden() const
{
    return (impl->displayMode == DisplayNever);
}


bool PositionDragger::isDragEnabled() const
{
    return impl->isDragEnabled;
}


void PositionDragger::setDragEnabled(bool on)
{
    impl->isDragEnabled = on;
}


bool PositionDragger::isDragging() const
{
    return impl->dragProjector.isDragging();
}


Affine3 PositionDragger::draggedPosition() const
{
    if(impl->dragProjector.isDragging()){
        return impl->dragProjector.position();
    } else {
        return T();
    }
}


const Vector3& PositionDragger::draggedTranslation() const
{
    return impl->dragProjector.translation();
}


const AngleAxis& PositionDragger::draggedAngleAxis() const
{
    return impl->dragProjector.rotationAngleAxis();
}


SignalProxy<void()> PositionDragger::sigDragStarted()
{
    return impl->sigDragStarted;
}


SignalProxy<void()> PositionDragger::sigPositionDragged()
{
    return impl->sigPositionDragged;
}


SignalProxy<void()> PositionDragger::sigDragFinished()
{
    return impl->sigDragFinished;
}


static bool detectAxisFromNodePath
(const SgNodePath& path, SgNode* topNode, int& out_axis, int& out_topNodeIndex)
{
    out_topNodeIndex = -1;
    
    for(size_t i=0; i < path.size(); ++i){
        if(path[i] == topNode){
            out_topNodeIndex = i;
            for(size_t j=i+1; j < path.size(); ++j){
                auto& name = path[j]->name();
                if(!name.empty()){
                    for(int k=0; k < 6; ++k){
                        if(name == axisNames[k]){
                            out_axis = k;
                            return true;
                        }
                    }
                }
            }
            break;
        }
    }
    return false;
}



bool PositionDragger::onButtonPressEvent(const SceneWidgetEvent& event)
{
    return impl->onButtonPressEvent(event);
}


bool PositionDragger::Impl::onButtonPressEvent(const SceneWidgetEvent& event)
{
    bool processed = false;

    if(!isDragEnabled){
        return processed;
    }

    int axis;
    int topNodeIndex;

    if(!::detectAxisFromNodePath(event.nodePath(), self, axis, topNodeIndex)){
        
        if(self->isContainerMode() && isContentsDragEnabled){
            if(displayMode == DisplayInFocus){
                showDragMarkers(true);
            }
            dragProjector.setInitialPosition(self->T());
            dragProjector.setTranslationAlongViewPlane();
            if(dragProjector.startTranslation(event)){
                processed = true;
            }
        }
    } else {
        int axisBit = 1 << axis;
        if(axisBit & TRANSLATION_AXES){
            processed = onTranslationDraggerPressed(event, axis, topNodeIndex);
        } else if(axisBit & ROTATION_AXES){
            processed = onRotationDraggerPressed(event, axis);
        }
    }

    if(processed){
        storeCurrentPositionToHistory();
        sigDragStarted();
    } else {
        dragProjector.resetDragMode();
    }
    
    return processed;
}


bool PositionDragger::Impl::onTranslationDraggerPressed(const SceneWidgetEvent& event, int axis, int topNodeIndex)
{
    bool processed = false;

    auto& path = event.nodePath();
    auto axisIter = path.begin() + topNodeIndex + 1;
    const Affine3 T_global = calcTotalTransform(path.begin(), axisIter);
    const Affine3 T_axis = calcTotalTransform(axisIter, path.end());
    const Vector3 p_local = (T_global * T_axis).inverse() * event.point();
    
    dragProjector.setInitialPosition(T_global);
    
    if(p_local.norm() < 2.0 * axisCylinderRadius){
        dragProjector.setTranslationAlongViewPlane();
    } else {
        dragProjector.setTranslationAxis(T_global.linear().col(axis));
    }
    if(dragProjector.startTranslation(event)){
        processed = true;
    }
    
    return processed;
}


bool PositionDragger::Impl::onRotationDraggerPressed(const SceneWidgetEvent& event, int axis)
{
    bool processed = false;
    const Affine3 T = calcTotalTransform(event.nodePath(), self);
    dragProjector.setInitialPosition(T);
    dragProjector.setRotationAxis(T.linear().col(axis - 3));
    if(dragProjector.startRotation(event)){
        processed = true;
    }
    return processed;
}


bool PositionDragger::onButtonReleaseEvent(const SceneWidgetEvent&)
{
    if(impl->dragProjector.isDragging()){
        impl->sigDragFinished();
        impl->dragProjector.resetDragMode();
        return true;
    }
    return false;
}


bool PositionDragger::onPointerMoveEvent(const SceneWidgetEvent& event)
{
    if(impl->dragProjector.drag(event)){
        if(isContainerMode()){
            setPosition(impl->dragProjector.position());
            notifyUpdate();
        }
        impl->sigPositionDragged();
        return true;
    }
    return false;
}


void PositionDragger::onPointerLeaveEvent(const SceneWidgetEvent&)
{
    if(impl->dragProjector.isDragging()){
        impl->sigDragFinished();
        impl->dragProjector.resetDragMode();
    }
}


void PositionDragger::onFocusChanged(const SceneWidgetEvent&, bool on)
{
    if(isContainerMode()){
        if(impl->displayMode == DisplayInFocus){
            impl->showDragMarkers(on);
        }
    }
}


void PositionDragger::onSceneModeChanged(const SceneWidgetEvent& event)
{
    if(event.sceneWidget()->isEditMode()){
        if(impl->displayMode == DisplayInEditMode){
            impl->showDragMarkers(true);
        }
    } else {
        if(impl->displayMode == DisplayInEditMode || impl->displayMode == DisplayInFocus){
            impl->showDragMarkers(false);
        }
    }
}


void PositionDragger::storeCurrentPositionToHistory()
{
    impl->storeCurrentPositionToHistory();
}


void PositionDragger::Impl::storeCurrentPositionToHistory()
{
    if(isUndoEnabled){
        history.push_back(self->position());
        if(history.size() > 10){
            history.pop_front();
        }
    }
}


void PositionDragger::setUndoEnabled(bool on)
{
    impl->isUndoEnabled = on;
}


bool PositionDragger::isUndoEnabled() const
{
    return impl->isUndoEnabled;
}


bool PositionDragger::onUndoRequest()
{
    if(!impl->history.empty()){
        const Affine3& T = impl->history.back();
        setPosition(T);
        impl->history.pop_back();
        notifyUpdate();
    }
    return true;
}


bool PositionDragger::onRedoRequest()
{
    return true;
}
