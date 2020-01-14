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
#include <bitset>
#include <deque>
#include <array>
#include <unordered_map>

#include <iostream>

using namespace std;
using namespace cnoid;

namespace {

constexpr int NumHandleVariants = 5;

typedef bitset<6> AxisBitSet;

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

constexpr double StdUnitHandleWidth = 0.02;
constexpr double StdPixelHandleWidth = 6.0;
constexpr double MaxPixelHandleWidthCorrectionRatio = 3.0;
constexpr double StdRotationHandleSizeRatio = 0.6;
constexpr double WideUnitHandleWidth = 0.08;
constexpr double WideRotationHandleSizeRatio = 0.5;
constexpr float DefaultTransparency = 0.4f;


class SgHandleVariantSelector : public SgGroup
{
    PositionDragger::Impl* dragger;

public:
    SgHandleVariantSelector(PositionDragger::Impl* dragger);
    void render(SceneRenderer* renderer);
};
    

/**
   \note This node is not inserted the node path obtained by SceneWidgetEvent::nodePath()
*/
class SgViewpointDependentSelector : public SgGroup
{
    Vector3 axis;
    double thresh;

public:
    SgViewpointDependentSelector();
    void setAxis(const Vector3& axis);
    void setSwitchAngle(double rad);
    void render(SceneRenderer* renderer);
};


void registerPickingShapeSelector(SceneNodeClassRegistry& registry)
{
    registry.registerClass<SgHandleVariantSelector, SgGroup>();

    SceneRenderer::addExtension(
        [](SceneRenderer* renderer){
            auto functions = renderer->renderingFunctions();
            functions->setFunction<SgHandleVariantSelector>(
                [=](SgNode* node){
                    static_cast<SgHandleVariantSelector*>(node)->render(renderer);
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
    unordered_map<double, SgNodePtr> handleVariantMap;
    AxisBitSet draggableAxisBitSet;
    SgSwitchPtr axisSwitch[6];
    int handleType;
    double handleSize;
    double unitHandleWidth;
    Signal<void(int axisBitSet)> sigDraggableAxesChanged;
    DisplayMode displayMode;
    bool isOverlayMode;
    bool isConstantPixelSizeMode;
    bool isContainerMode;
    bool isDragEnabled;
    bool isContentsDragEnabled;
    bool isUndoEnabled;
    SgSwitchableGroupPtr topSwitch;
    SgOverlayPtr overlay;
    SgAutoScalePtr autoScale;
    double rotationHandleSizeRatio;
    array<SgMaterialPtr, 6> axisMaterials;
    float transparency;
    AxisBitSet highlightedAxisBitSet;
    MeshGenerator meshGenerator;
    SceneDragProjector dragProjector;
    Signal<void()> sigDragStarted;
    Signal<void()> sigPositionDragged;
    Signal<void()> sigDragFinished;
    std::deque<Affine3> history;

    Impl(PositionDragger* self, int mode, int axes);
    SgNode* createHandle(double widthRatio);
    SgNode* createTranslationHandle(double widthRatio);
    SgNode* createRotationRingHandle(double widthRatio);
    SgNode* createRotationDiscHandle(double widthRatio);
    double calcWidthRatio(double pixelSizeRatio);
    SgNode* getOrCreateHandleVariant(double pixelSizeRatio, bool isForPicking);
    void clearHandleVariants();
    void setDraggableAxes(AxisBitSet axisBitSet);
    void setMaterialParameters(AxisBitSet axisBitSet, float t, bool isHighlighted);
    void highlightAxes(AxisBitSet axisBitSet);
    void showDragMarkers(bool on);
    AxisBitSet detectTargetAxes(const SceneWidgetEvent& event);
    bool onButtonPressEvent(const SceneWidgetEvent& event);
    bool onTranslationDraggerPressed(const SceneWidgetEvent& event, AxisBitSet axisBitSet);
    bool onRotationDraggerPressed(const SceneWidgetEvent& event, AxisBitSet axisBitSet);
    void storeCurrentPositionToHistory();
};

}


SgHandleVariantSelector::SgHandleVariantSelector(PositionDragger::Impl* dragger)
    : SgGroup(findClassId<SgHandleVariantSelector>()),
      dragger(dragger)
{

}


void SgHandleVariantSelector::render(SceneRenderer* renderer)
{
    double pixelSizeRatio = -1.0;
    bool isForPicking = false;
    if(!dragger->isConstantPixelSizeMode){
        pixelSizeRatio = renderer->projectedPixelSizeRatio(
            renderer->currentModelTransform().translation());
        isForPicking = renderer->isRenderingPickingImage();
    }

    //
    clearChildren();
    
    if(auto node = dragger->getOrCreateHandleVariant(pixelSizeRatio, isForPicking)){
        addChildOnce(node);
        renderer->renderNode(node);
    }
}


SgViewpointDependentSelector::SgViewpointDependentSelector()
    : SgGroup(findClassId<SgViewpointDependentSelector>())
{
    axis = Vector3::UnitX();
    thresh = cos(radian(45.0));
}


void SgViewpointDependentSelector::setAxis(const Vector3& axis)
{
    this->axis = axis;
}


void SgViewpointDependentSelector::setSwitchAngle(double rad)
{
    thresh = cos(rad);
}


void SgViewpointDependentSelector::render(SceneRenderer* renderer)
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


PositionDragger::PositionDragger(int mode, int axes)
{
    impl = new Impl(this, mode, axes);
}


PositionDragger::Impl::Impl(PositionDragger* self, int axes, int handleType)
    : self(self),
      draggableAxisBitSet(axes),
      handleType(handleType)
{
    auto& registry = SceneNodeClassRegistry::instance();

    handleSize = 1.0;
    
    if(!registry.hasRegistration<SgHandleVariantSelector>()){
        registerPickingShapeSelector(registry);
    }

    if(handleType != WideHandle){
        unitHandleWidth = StdUnitHandleWidth;
        rotationHandleSizeRatio = StdRotationHandleSizeRatio;
    } else {
        if(!registry.hasRegistration<SgViewpointDependentSelector>()){
            registerViewpointDependentSelector(registry);
        }
        unitHandleWidth = WideUnitHandleWidth;
        rotationHandleSizeRatio = WideRotationHandleSizeRatio;
    }

    for(int i=0; i < 6; ++i){
        axisSwitch[i] = new SgSwitch(draggableAxisBitSet[i]);
    }
    

    displayMode = DisplayInFocus;
    isOverlayMode = false;
    isConstantPixelSizeMode = false;
    isContainerMode = false;
    isDragEnabled = true;
    isContentsDragEnabled = true;
    isUndoEnabled = false;

    transparency = DefaultTransparency;
    highlightedAxisBitSet.reset();

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

    topSwitch = new SgSwitchableGroup;
    topSwitch->addChild(new SgHandleVariantSelector(this));
    self->addChild(topSwitch);
}


SgNode* PositionDragger::Impl::createHandle(double widthRatio)
{
    auto draggerAxes = new SgGroup;
    
    draggerAxes->addChild(createTranslationHandle(widthRatio));

    if(handleType != WideHandle){
        draggerAxes->addChild(createRotationRingHandle(widthRatio));
    } else {
        draggerAxes->addChild(createRotationDiscHandle(widthRatio));
    }

    return draggerAxes;
}

    
SgNode* PositionDragger::Impl::createTranslationHandle(double widthRatio)
{
    auto scale = new SgScaleTransform(handleSize);

    double endLength = unitHandleWidth * 2.5;
    double extraEndLength = endLength * (widthRatio - 1.0);
    double stickLength = 1.0 - endLength;
    if(!(handleType == PositiveOnlyHandle)){
        stickLength *= 2.0;
    }
    stickLength -= extraEndLength / 2.0;
    int divisionNumber = std::max((int)(24 / widthRatio), 8);
    meshGenerator.setDivisionNumber(divisionNumber);
    SgMeshPtr mesh = meshGenerator.generateArrow(
        (unitHandleWidth / 2.0) * widthRatio,
        stickLength,
        unitHandleWidth * 1.25 * widthRatio,
        endLength * widthRatio);
    
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
        } else {
            arrow->translation()[i] = -extraEndLength / 2.0;
        }
        arrow->setName(AxisNames[i]);

        auto axis = new SgSwitchableGroup(axisSwitch[i]);
        axis->addChild(arrow);
        scale->addChild(axis);
    }

    return scale;
}


SgNode* PositionDragger::Impl::createRotationRingHandle(double widthRatio)
{
    auto scale = new SgScaleTransform(handleSize * rotationHandleSizeRatio);

    double radius = widthRatio * unitHandleWidth / 2.0 / rotationHandleSizeRatio;
    double endAngle = (handleType == PositiveOnlyHandle) ? (PI / 2.0) : 2.0 * PI;
    int divisionNumber = std::max((int)(72 / widthRatio), 24);
    meshGenerator.setDivisionNumber(divisionNumber);
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

        scale->addChild(axis);
    }

    return scale;
}


SgNode* PositionDragger::Impl::createRotationDiscHandle(double widthRatio)
{
    auto scale = new SgScaleTransform(handleSize * rotationHandleSizeRatio);

    SgMesh* mesh[2];
    double width = unitHandleWidth / rotationHandleSizeRatio * widthRatio;
    meshGenerator.setDivisionNumber(36);
    mesh[0] = meshGenerator.generateDisc(1.0, 1.0 - width);
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
        scale->addChild(axis);
    }

    return scale;
}


double PositionDragger::Impl::calcWidthRatio(double pixelSizeRatio)
{
    double widthRatio = 1.0;
    if(pixelSizeRatio >= 0.0){
        double pixelWidth = unitHandleWidth * handleSize * pixelSizeRatio;
        if(pixelWidth > 0.1){
            if(pixelWidth < StdPixelHandleWidth){
                widthRatio = StdPixelHandleWidth / pixelWidth;
            }
            if(widthRatio > MaxPixelHandleWidthCorrectionRatio){
                widthRatio = MaxPixelHandleWidthCorrectionRatio;
            }
        }
    }
    return widthRatio;
}
    

/**
   \todo
    - Make the handles for picking a bit wider
    - Cache the generated handles in handleVariantMap
    - Adjust the width considering the DPI of the display
    - Add the API to customize the width
*/
SgNode* PositionDragger::Impl::getOrCreateHandleVariant(double pixelSizeRatio, bool isForPicking)
{
    double widthRatio;
    if(handleType != WideHandle){
        widthRatio = calcWidthRatio(pixelSizeRatio);
    } else {
        widthRatio = 1.0;
    }
    return createHandle(widthRatio);
}


void PositionDragger::Impl::clearHandleVariants()
{
    handleVariantMap.clear();
}


void PositionDragger::setDraggableAxes(int axisBitSet)
{
    impl->setDraggableAxes(axisBitSet);
}


void PositionDragger::Impl::setDraggableAxes(AxisBitSet axisBitSet)
{
    if(axisBitSet != draggableAxisBitSet){
        for(int i=0; i < 6; ++i){
            axisSwitch[i]->setTurnedOn(axisBitSet[i], true);
        }
        draggableAxisBitSet = axisBitSet;
    }
}


int PositionDragger::draggableAxes() const
{
    return static_cast<int>(impl->draggableAxisBitSet.to_ulong());
}


SignalProxy<void(int axisSet)> PositionDragger::sigDraggableAxesChanged()
{
    return impl->sigDraggableAxesChanged;
}


double PositionDragger::handleSize() const
{
    return impl->handleSize;
}


void PositionDragger::setHandleSize(double s)
{
    if(s != impl->handleSize){
        impl->handleSize = s;
        impl->clearHandleVariants();
    }
}
    

double PositionDragger::rotationHandleSizeRatio() const
{
    return impl->rotationHandleSizeRatio;
}


void PositionDragger::setRotationHandleSizeRatio(double r)
{
    if(r != impl->rotationHandleSizeRatio){
        impl->rotationHandleSizeRatio = r;
        impl->clearHandleVariants();
    }
}


void PositionDragger::setRadius(double r, double translationAxisRatio)
{
    impl->rotationHandleSizeRatio = 1.0 / translationAxisRatio;
    setHandleSize(2.0 * r);
}


double PositionDragger::radius() const
{
    return impl->handleSize * impl->rotationHandleSizeRatio;
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
        if(node != impl->topSwitch){
            bb.expandBy(node->boundingBox());
        }
    }
    return adjustSize(bb);
}


void PositionDragger::setTransparency(float t)
{
    impl->setMaterialParameters(AllAxes, t, false);
    impl->transparency = t;
    impl->highlightedAxisBitSet.reset();
}


void PositionDragger::Impl::setMaterialParameters(AxisBitSet axisBitSet, float t, bool isHighlighted)
{
    float t2 = (t == 0.0f) ? 0.0f : (1.0f - (1.0f - t) / 2.0f);
    
    for(int i=0; i < 6; ++i){
        int axis = i < 3 ? i : i - 3;
        if(i == 3){
            if(handleType == WideHandle){
                t2 = t;
            }
        }
        if(axisBitSet[i]){
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


void PositionDragger::Impl::highlightAxes(AxisBitSet axisBitSet)
{
    if(axisBitSet != highlightedAxisBitSet){
        if(highlightedAxisBitSet.any()){
            setMaterialParameters(highlightedAxisBitSet, transparency, false);
        }
        if(axisBitSet.any()){
            setMaterialParameters(axisBitSet, 0.0f, true);
        }
        highlightedAxisBitSet = axisBitSet;
    }
}


void PositionDragger::setOverlayMode(bool on)
{
    if(on != impl->isOverlayMode){
        if(on){
            if(!impl->overlay){
                impl->overlay = new SgOverlay;
            }
            impl->topSwitch->insertChainedGroup(impl->overlay);
        } else {
            impl->topSwitch->removeChainedGroup(impl->overlay);
        }
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
            impl->topSwitch->insertChainedGroup(impl->autoScale);
        } else {
            impl->topSwitch->removeChainedGroup(impl->autoScale);
        }
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
    
    topSwitch->setTurnedOn(on, true);
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


AxisBitSet PositionDragger::Impl::detectTargetAxes(const SceneWidgetEvent& event)
{
    AxisBitSet axisBitSet(0);

    auto& path = event.nodePath();
    for(size_t i=0; i < path.size(); ++i){
        if(path[i] == self){
            for(size_t j=i+1; j < path.size(); ++j){
                auto& name = path[j]->name();
                if(!name.empty()){
                    for(int k=0; k < 6; ++k){
                        if(name == AxisNames[k]){
                            axisBitSet.set(k);
                            break;
                        }
                    }
                }
            }
            break;
        }
    }

    if((axisBitSet & AxisBitSet(TRANSLATION_AXES)).any()){
        const Affine3 T = calcTotalTransform(event.nodePath(), self);
        const Vector3 p_local = T.inverse() * event.point();
        double width = handleSize * unitHandleWidth * calcWidthRatio(event.pixelSizeRatio());
        if(p_local.norm() < 1.5 * width){
            axisBitSet |= AxisBitSet(TRANSLATION_AXES);
        }
    }

    return axisBitSet;
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

    auto axisBitSet = detectTargetAxes(event);

    if(axisBitSet.none()){
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
        if((axisBitSet & AxisBitSet(TRANSLATION_AXES)).any()){
            processed = onTranslationDraggerPressed(event, axisBitSet);
        } else if((axisBitSet & AxisBitSet(ROTATION_AXES)).any()){
            processed = onRotationDraggerPressed(event, axisBitSet);
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


bool PositionDragger::Impl::onTranslationDraggerPressed(const SceneWidgetEvent& event, AxisBitSet axisBitSet)
{
    bool processed = false;

    const Affine3 T = calcTotalTransform(event.nodePath(), self);
    dragProjector.setInitialPosition(T);
    
    if(axisBitSet.count() == 1){
        for(int i=0; i < 3; ++i){
            if(axisBitSet[i]){
                dragProjector.setTranslationAxis(T.linear().col(i));
                break;
            }
        }
    } else {
        dragProjector.setTranslationAlongViewPlane();
    }        
    
    if(dragProjector.startTranslation(event)){
        processed = true;
    }
    
    return processed;
}


bool PositionDragger::Impl::onRotationDraggerPressed(const SceneWidgetEvent& event, AxisBitSet axisBitSet)
{
    bool processed = false;
    const Affine3 T = calcTotalTransform(event.nodePath(), self);

    if(axisBitSet.count() == 1){
        dragProjector.setInitialPosition(T);
        for(int i=0; i < 3; ++i){
            if(axisBitSet[i + 3]){
                dragProjector.setRotationAxis(T.linear().col(i));
                if(dragProjector.startRotation(event)){
                    processed = true;
                }
                break;
            }
        }
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
        auto axisBitSet = impl->detectTargetAxes(event);
        if(axisBitSet.any()){
            impl->highlightAxes(axisBitSet);
        }
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
