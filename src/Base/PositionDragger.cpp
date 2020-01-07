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

const char* AxisNames[6] = { "tx", "ty", "tz", "rx", "ry", "rz" };

const Vector3f AxisColors[3] = {
    { 1.0f, 0.2f, 0.2f },
    { 0.2f, 1.0f, 0.2f },
    { 0.2f, 0.2f, 1.0f }
};

const Vector3f HighlightedAxisColors[3] = {
    { 1.0f, 0.4f, 0.4f },
    { 0.5f, 1.0f, 0.4f },
    { 0.4f, 0.5f, 1.0f }
};


constexpr float DefaultTransparency = 0.4f;


class SgPickingSwitch : public SgGroup
{
    int switchBoundaryIndex;
    
public:
    SgPickingSwitch() : SgGroup(findClassId<SgPickingSwitch>())
    {
        switchBoundaryIndex = 0;
    }

    void setBoundaryIndex(int index)
    {
        switchBoundaryIndex = index;
    }
    
    void render(SceneRenderer* renderer)
    {
        if(!renderer->isPicking()){
            const int n = std::min(switchBoundaryIndex, numChildren());
            for(int i=0; i < n; ++i){
                renderer->renderNode(child(i));
            }
        } else {
            const int n = numChildren();
            for(int i=switchBoundaryIndex; i < n; ++i){
                renderer->renderNode(child(i));
            }
        }
    }
};
    

/**
   \note This node is not inserted the node path obtained by SceneWidgetEvent::nodePath()
*/
class SgViewpointDependentSelector : public SgGroup
{
    Vector3 axis;
    double thresh;
    
public:
    SgViewpointDependentSelector()
        : SgGroup(findClassId<SgViewpointDependentSelector>())
    {
        axis = Vector3::UnitX();
        thresh = cos(radian(45.0));
    }

    void setAxis(const Vector3& axis)
    {
        this->axis = axis;
    }

    void setSwitchAngle(double rad)
    {
        thresh = cos(rad);
    }

    void render(SceneRenderer* renderer)
    {
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


void registerPickingSwitch(SceneNodeClassRegistry& registry)
{
    registry.registerClass<SgPickingSwitch, SgGroup>();

    SceneRenderer::addExtension(
        [](SceneRenderer* renderer){
            auto functions = renderer->renderingFunctions();
            functions->setFunction<SgPickingSwitch>(
                [=](SgNode* node){
                    static_cast<SgPickingSwitch*>(node)->render(renderer);
                });
        });
}


void registerViewpointDependentSelector(SceneNodeClassRegistry& registry)
{
    registry.registerClass<SgViewpointDependentSelector, SgGroup>();

    SceneRenderer::addExtension(
        [](SceneRenderer* renderer){
            auto functions = renderer->renderingFunctions();
            functions->setFunction<SgViewpointDependentSelector>(
                [=](SgNode* node){
                    static_cast<SgViewpointDependentSelector*>(node)->render(renderer);
                });
        });
}

}

namespace cnoid {

class PositionDragger::Impl
{
public:
    PositionDragger* self;
    int draggableAxes;
    SgSwitchPtr axisSwitch[6];
    int handleType;
    double handleWidth;
    Signal<void(int axisSet)> sigDraggableAxesChanged;
    DisplayMode displayMode;
    bool isOverlayMode;
    bool isConstantPixelSizeMode;
    bool isContainerMode;
    bool isDragEnabled;
    bool isContentsDragEnabled;
    bool isUndoEnabled;
    SgGroupPtr axisGroup;
    SgOverlayPtr overlay;
    SgAutoScalePtr autoScale;
    SgScaleTransformPtr translationAxisScale;
    SgScaleTransformPtr rotationAxisScale;
    double rotationHandleSizeRatio;
    array<SgMaterialPtr, 6> axisMaterials;
    float transparency;
    int highlightedAxisSet;
    SceneDragProjector dragProjector;
    Signal<void()> sigDragStarted;
    Signal<void()> sigPositionDragged;
    Signal<void()> sigDragFinished;
    std::deque<Affine3> history;

    Impl(PositionDragger* self, int mode, int axes);
    void createDraggers();
    void createTranslationAxisArrows(MeshGenerator& meshGenerator);
    void createRotationAxisRings(MeshGenerator& meshGenerator);
    void createRotationAxisDiscs(MeshGenerator& meshGenerator);
    void setDraggableAxes(int axisSet);
    void setMaterialParameters(int axisSet, float t, bool isHighlighted);
    void highlightAxes(int axisSet);
    void showDragMarkers(bool on);
    bool onButtonPressEvent(const SceneWidgetEvent& event);
    bool onTranslationDraggerPressed(const SceneWidgetEvent& event, int axis, int topNodeIndex);
    bool onRotationDraggerPressed(const SceneWidgetEvent& event, int axis);
    void storeCurrentPositionToHistory();
};

}


PositionDragger::PositionDragger(int mode, int axes)
{
    impl = new Impl(this, mode, axes);
}


PositionDragger::Impl::Impl(PositionDragger* self, int axes, int handleType)
    : self(self),
      draggableAxes(axes),
      handleType(handleType)
{
    auto& registry = SceneNodeClassRegistry::instance();
    
    if(handleType != WideHandle){
        if(!registry.hasRegistration<SgPickingSwitch>()){
            registerPickingSwitch(registry);
        }
        handleWidth = 0.04;

    } else {
        if(!registry.hasRegistration<SgViewpointDependentSelector>()){
            registerViewpointDependentSelector(registry);
        }
        handleWidth = 0.08;
    }

    for(int i=0; i < 6; ++i){
        axisSwitch[i] = new SgSwitch(draggableAxes & (1 << i));
    }
    
    rotationHandleSizeRatio = 0.6;
    axisGroup = self;

    createDraggers();

    displayMode = DisplayInFocus;
    isOverlayMode = false;
    isConstantPixelSizeMode = false;
    isContainerMode = false;
    isDragEnabled = true;
    isContentsDragEnabled = true;
    isUndoEnabled = false;
}


void PositionDragger::Impl::createDraggers()
{
    transparency = DefaultTransparency;
    highlightedAxisSet = 0;

    for(int i=0; i < 3; ++i){
        auto material = new SgMaterial;
        material->setDiffuseColor(Vector3f::Zero());
        material->setEmissiveColor(AxisColors[i]);
        material->setAmbientIntensity(0.0f);
        material->setTransparency(1.0f - (1.0f - transparency) / 2.0f);
        axisMaterials[i] = material;

        auto rotationAxisMaterial = new SgMaterial(*material);
        if(handleType == WideHandle){
            rotationAxisMaterial->setTransparency(transparency);
        }
        axisMaterials[i+3] = rotationAxisMaterial;
    }

    MeshGenerator meshGenerator;

    createTranslationAxisArrows(meshGenerator);

    if(handleType == WideHandle){
        createRotationAxisDiscs(meshGenerator);
    } else {
        createRotationAxisRings(meshGenerator);
    }
}

    
void PositionDragger::Impl::createTranslationAxisArrows(MeshGenerator& meshGenerator)
{
    translationAxisScale = new SgScaleTransform;

    double endLength = handleWidth * 2.5;
    double stickLength = 1.0 - endLength;
    if(!(handleType == PositiveOnlyHandle)){
        stickLength *= 2.0;
    }
    SgMeshPtr mesh = meshGenerator.generateArrow(handleWidth / 2.0, stickLength, handleWidth * 1.25, endLength);
    
    for(int i=0; i < 3; ++i){
        auto shape = new SgShape;
        shape->setMesh(mesh);
        shape->setMaterial(axisMaterials[i]);
            
        auto arrow = new SgPosTransform;
        arrow->addChild(shape);
        if(i == 0){
            arrow->setRotation(AngleAxis(-PI / 2.0, Vector3::UnitZ()));
        } else if(i == 2){
            arrow->setRotation(AngleAxis(PI / 2.0, Vector3::UnitX()));
        }
        if(handleType == PositiveOnlyHandle){
            arrow->translation()[i] = stickLength / 2.0;
        }
        arrow->setName(AxisNames[i]);

        auto axis = new SgSwitchableGroup(axisSwitch[i]);
        axis->addChild(arrow);
        translationAxisScale->addChild(axis);
    }

    axisGroup->addChild(translationAxisScale);
}


void PositionDragger::Impl::createRotationAxisRings(MeshGenerator& meshGenerator)
{
    rotationAxisScale = new SgScaleTransform;

    meshGenerator.setDivisionNumber(36);
    double radius = handleWidth / 2.0 / rotationHandleSizeRatio;
    double endAngle = (handleType == PositiveOnlyHandle) ? (PI / 2.0) : 2.0 * PI;
    auto ringMesh = meshGenerator.generateTorus(1.0, radius, 0.0, endAngle);

    for(int i=0; i < 3; ++i){
        auto material = axisMaterials[i+3];
        
        auto ringShape = new SgShape;
        ringShape->setMesh(ringMesh);
        ringShape->setMaterial(material);

        auto ring = new SgPosTransform;
        if(i == 0){ // x-axis
            ring->setRotation(AngleAxis(PI / 2.0, Vector3::UnitZ()));
        } else if(i == 2) { // z-axis
            ring->setRotation(AngleAxis(-PI / 2.0, Vector3::UnitX()));
        }
        ring->addChild(ringShape);
        ring->setName(AxisNames[i + 3]);

        auto axis = new SgSwitchableGroup(axisSwitch[i + 3]);
        axis->addChild(ring);

        rotationAxisScale->addChild(axis);
    }

    axisGroup->addChild(rotationAxisScale);
}


void PositionDragger::Impl::createRotationAxisDiscs(MeshGenerator& meshGenerator)
{
    rotationAxisScale = new SgScaleTransform;

    SgMesh* mesh[2];
    double width = handleWidth / rotationHandleSizeRatio;
    meshGenerator.setDivisionNumber(36);
    mesh[0] = meshGenerator.generateDisc(1.0, 1.0 - width);
    meshGenerator.setDivisionNumber(24);
    mesh[1] = meshGenerator.generateCylinder(1.0, width, false, false);

    for(int i=0; i < 3; ++i){
        auto selector = new SgViewpointDependentSelector;
        Vector3 a = Vector3::Zero();
        a(i) = 1.0;
        selector->setAxis(a);
        for(int j=0; j < 2; ++j){
            auto shape = new SgShape;
            shape->setMesh(mesh[j]);
            shape->setMaterial(axisMaterials[i+3]);
            auto disc = new SgPosTransform;
            if(i == 0){ // x-axis
                disc->setRotation(AngleAxis(-PI / 2.0, Vector3::UnitZ()));
            } else if(i == 2) { // z-axis
                disc->setRotation(AngleAxis(PI / 2.0, Vector3::UnitX()));
            }
            disc->addChild(shape);
            disc->setName(AxisNames[i + 3]);
            selector->addChild(disc);
        }
        auto axis = new SgSwitchableGroup(axisSwitch[i + 3]);
        axis->addChild(selector);
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
        for(int i=0; i < 6; ++i){
            axisSwitch[i]->setTurnedOn(axisSet & (1 << i), true);
        }
        draggableAxes = axisSet;
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


void PositionDragger::setRotationHandleSizeRatio(double r)
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


bool PositionDragger::adjustSize(const BoundingBox& bb)
{
    if(bb.empty()){
        return false;
    }

    Vector3 s = bb.size() / 2.0;
    std::sort(s.data(), s.data() + 3);
    double r = Vector2(s[0], s[1]).norm();
    if(!impl->isOverlayMode){
        r *= 1.05;
    }
    setHandleSize(r / impl->rotationHandleSizeRatio);
    return true;
}


bool PositionDragger::adjustSize()
{
    BoundingBox bb;
    for(int i=0; i < numChildren(); ++i){
        auto node = child(i);
        if(node != impl->translationAxisScale && node != impl->rotationAxisScale){
            bb.expandBy(node->boundingBox());
        }
    }
    return adjustSize(bb);
}


void PositionDragger::setTransparency(float t)
{
    impl->setMaterialParameters(AllAxes, t, false);
    impl->transparency = t;
    impl->highlightedAxisSet = 0;
}


void PositionDragger::Impl::setMaterialParameters(int axisSet, float t, bool isHighlighted)
{
    float t2 = (t == 0.0f) ? 0.0f : (1.0f - (1.0f - t) / 2.0f);
    
    for(int i=0; i < 6; ++i){
        int axis = i < 3 ? i : i - 3;
        if(i == 3){
            if(handleType == WideHandle){
                t2 = t;
            }
        }
        if(axisSet & (1 << i)){
            bool updated = false;
            auto& material = axisMaterials[i];
            if(t != material->transparency()){
                material->setTransparency(t2);
                updated = true;
            }
            if(transparency == 0.0f){
                const Vector3f& color = isHighlighted ? HighlightedAxisColors[axis] : AxisColors[axis];
                if(color != material->emissiveColor()){
                    material->setEmissiveColor(color);
                    updated = true;
                }
            }
            if(updated){
                material->notifyUpdate();
            }
        }
    }
}
    

float PositionDragger::transparency() const
{
    return impl->transparency;
}


void PositionDragger::Impl::highlightAxes(int axisSet)
{
    if(axisSet != highlightedAxisSet){
        if(highlightedAxisSet){
            setMaterialParameters(highlightedAxisSet, transparency, false);
        }
        if(axisSet){
            setMaterialParameters(axisSet, 0.0f, true);
        }
        highlightedAxisSet = axisSet;
    }
}


void PositionDragger::setOverlayMode(bool on)
{
    if(on != impl->isOverlayMode){
        if(on){
            if(!impl->overlay){
                impl->overlay = new SgOverlay;
            }
            insertChainedGroup(impl->overlay);
        } else {
            removeChainedGroup(impl->overlay);
        }
        impl->axisGroup = lastChainedGroup();
        impl->isOverlayMode = on;
    }
}


bool PositionDragger::isOverlayMode() const
{
    return impl->isOverlayMode;
}


void PositionDragger::setConstantPixelSizeMode(bool on, double pixelSizeRatio)
{
    if(impl->autoScale){
        impl->autoScale->setPixelSizeRatio(pixelSizeRatio);
    }
    if(on != impl->isConstantPixelSizeMode){
        if(on){
            if(!impl->autoScale){
                impl->autoScale = new SgAutoScale(pixelSizeRatio);
            }
            insertChainedGroup(impl->autoScale);
        } else {
            removeChainedGroup(impl->autoScale);
        }
        impl->axisGroup = lastChainedGroup();
        impl->isConstantPixelSizeMode = on;
    }
}


bool PositionDragger::isConstantPixelSizeMode() const
{
    return impl->isConstantPixelSizeMode;
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
                        if(name == AxisNames[k]){
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
    
    if(p_local.norm() < handleWidth){
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
    if(!impl->dragProjector.isDragging()){
        int axisSet = 0;
        int axis;
        int topNodeIndex;
        if(detectAxisFromNodePath(event.nodePath(), this, axis, topNodeIndex)){
            axisSet = 1 << axis;
        }
        impl->highlightAxes(axisSet);

    } else if(impl->dragProjector.drag(event)){
        if(isContainerMode()){
            setPosition(impl->dragProjector.position());
            notifyUpdate();
        }
        impl->sigPositionDragged();
    }
    
    return true;
}


void PositionDragger::onPointerLeaveEvent(const SceneWidgetEvent&)
{
    if(impl->dragProjector.isDragging()){
        impl->sigDragFinished();
        impl->dragProjector.resetDragMode();
    }

    impl->highlightAxes(0);
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
