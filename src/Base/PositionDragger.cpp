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
#include <array>
#include <unordered_map>

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
constexpr double StdPixelHandleWidth = 5.0;
constexpr double MaxPixelHandleWidthCorrectionRatio = 5.0;
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


float convertToTubeTransparency(float t)
{
    // Reduce the alpha value by half because the total alpha value of the handle
    // is a composite of the front and back surfaces of the handle tube shape
    return (t == 0.0f) ? 0.0f : (1.0f - (1.0f - t) / 2.0f);
}

}

namespace cnoid {

class PositionDragger::Impl
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    PositionDragger* self;
    map<int, SgNodePtr> widthLevelToHandleShapeMap;
    AxisBitSet draggableAxisBitSet;
    SgSwitchPtr axisSwitch[6];
    int handleType;
    double handleSize;
    double handleWidth;
    double unitHandleWidth;
    Signal<void(int axisBitSet)> sigDraggableAxesChanged;
    DisplayMode displayMode;
    bool isEditMode;
    bool isOverlayMode;
    bool isScreenFixedSizeMode;
    bool isContainerMode;
    bool isDragEnabled;
    bool isContentsDragEnabled;
    bool hasOffset;
    Isometry3 T_parent;
    Isometry3 T_offset;
    SgSwitchableGroupPtr topSwitch;
    SgOverlayPtr overlay;
    SgFixedPixelSizeGroupPtr fixedPixelSizeGroup;
    double rotationHandleSizeRatio;
    array<SgMaterialPtr, 6> axisMaterials;
    float transparency;
    AxisBitSet highlightedAxisBitSet;
    MeshGenerator meshGenerator;
    SceneDragProjector dragProjector;
    Signal<void()> sigDragStarted;
    Signal<void()> sigPositionDragged;
    Signal<void()> sigDragFinished;

    Impl(PositionDragger* self, int mode, int axes);
    SgNode* createHandle(double widthRatio);
    SgNode* createTranslationHandle(double widthRatio);
    SgNode* createRotationRingHandle(double widthRatio);
    SgNode* createRotationDiscHandle(double widthRatio);
    double calcWidthRatio(double pixelSizeRatio);
    SgNode* getOrCreateHandleVariant(double pixelSizeRatio, bool isForPicking);
    void clearHandleVariants();
    void setDraggableAxes(AxisBitSet axisBitSet, SgUpdateRef update);
    void disableScreenFixedSizeMode();
    void setMaterialParameters(AxisBitSet axisBitSet, float t, bool isHighlighted);
    void highlightAxes(AxisBitSet axisBitSet);
    void showDragMarkers(bool on, SgUpdateRef update);
    AxisBitSet detectTargetAxes(SceneWidgetEvent* event);
    bool onButtonPressEvent(SceneWidgetEvent* event);
    bool onTranslationDraggerPressed(
        SceneWidgetEvent* event, const Isometry3& T_global, AxisBitSet axisBitSet);
    bool onRotationDraggerPressed(
        SceneWidgetEvent* event, const Isometry3& T_global, AxisBitSet axisBitSet);
};

}


namespace {

SgHandleVariantSelector::SgHandleVariantSelector(PositionDragger::Impl* dragger)
    : SgGroup(findClassId<SgHandleVariantSelector>()),
      dragger(dragger)
{

}


void SgHandleVariantSelector::render(SceneRenderer* renderer)
{
    double pixelSizeRatio = -1.0;
    bool isForPicking = false;
    if(!dragger->isScreenFixedSizeMode){
        pixelSizeRatio = renderer->projectedPixelSizeRatio(
            renderer->currentModelTransform().translation());
        isForPicking = renderer->isRenderingPickingImage();
    }

    clearChildren();
    if(auto node = dragger->getOrCreateHandleVariant(pixelSizeRatio, isForPicking)){
        addChild(node);
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
    const Isometry3& C = renderer->currentCameraPosition();
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

}


PositionDragger::PositionDragger(int mode, int axes)
{
    setAttribute(Marker | Operable);
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
    

    displayMode = DisplayInEditMode;
    isEditMode = false;
    isOverlayMode = false;
    isScreenFixedSizeMode = false;
    isContainerMode = false;
    isDragEnabled = true;
    isContentsDragEnabled = true;

    hasOffset = false;
    T_offset.setIdentity();

    transparency = DefaultTransparency;
    highlightedAxisBitSet.reset();

    for(int i=0; i < 3; ++i){
        auto material = new SgMaterial;
        material->setDiffuseColor(Vector3f::Zero());
        material->setEmissiveColor(AxisColors[i]);
        material->setAmbientIntensity(0.0f);
        material->setTransparency(convertToTubeTransparency(transparency));
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

    double k = (widthRatio - 1.0) / (MaxPixelHandleWidthCorrectionRatio - 1.0);
    int divisionNumber = 24.0  * (1.0 - k) + 8.0 * k;
    
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
    SgMesh::Cylinder cylinder(1.0, width);
    cylinder.top = cylinder.bottom = false;
    mesh[1] = new SgMesh(cylinder);
    meshGenerator.updateMeshWithPrimitiveInformation(mesh[1]);

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
    

SgNode* PositionDragger::Impl::getOrCreateHandleVariant(double pixelSizeRatio, bool isForPicking)
{
    double widthRatio;
    if(handleType != WideHandle){
        widthRatio = calcWidthRatio(pixelSizeRatio);
    } else {
        widthRatio = 1.0;
    }
    constexpr double resolution = 5.0;
    int widthLevel = std::round(widthRatio * resolution);

    auto p = widthLevelToHandleShapeMap.find(widthLevel);
    if(p != widthLevelToHandleShapeMap.end()){
        return p->second;
    } else {
        auto shape = createHandle(widthLevel / resolution);
        widthLevelToHandleShapeMap[widthLevel] = shape;
        return shape;
    }
}


void PositionDragger::Impl::clearHandleVariants()
{
    widthLevelToHandleShapeMap.clear();
}


void PositionDragger::setOffset(const Isometry3& T)
{
    impl->T_offset = T;
    impl->hasOffset = (T.matrix() != Isometry3::Identity().matrix());
}


void PositionDragger::setDraggableAxes(int axisBitSet, SgUpdateRef update)
{
    impl->setDraggableAxes(axisBitSet, update);
}


void PositionDragger::Impl::setDraggableAxes(AxisBitSet axisBitSet, SgUpdateRef update)
{
    if(axisBitSet != draggableAxisBitSet){
        for(int i=0; i < 6; ++i){
            axisSwitch[i]->setTurnedOn(axisBitSet[i], update);
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
    impl->disableScreenFixedSizeMode();
}


void PositionDragger::Impl::disableScreenFixedSizeMode()
{
    if(isScreenFixedSizeMode){
        topSwitch->removeChainedGroup(fixedPixelSizeGroup);
        isScreenFixedSizeMode = false;
    }
}    


void PositionDragger::setHandleWidthRatio(double w)
{
    if(w != impl->unitHandleWidth){
        impl->unitHandleWidth = w;
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

    if(s[2] > 3.0 * s[1]){
        s[2] = 3.0 * s[1];
    }
    double r = Vector2(s[2], s[1]).norm();
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


void PositionDragger::setPixelSize(int size, int width)
{
    double dsize = size;
    impl->handleSize = 1.0;
    impl->clearHandleVariants();
    impl->unitHandleWidth = width / dsize;

    if(impl->fixedPixelSizeGroup){
        impl->fixedPixelSizeGroup->setPixelSizeRatio(dsize);
    }

    if(!impl->isScreenFixedSizeMode){
        if(!impl->fixedPixelSizeGroup){
            impl->fixedPixelSizeGroup = new SgFixedPixelSizeGroup(dsize);
        }
        impl->topSwitch->insertChainedGroup(impl->fixedPixelSizeGroup);
        impl->isScreenFixedSizeMode = true;
    } else {
        impl->fixedPixelSizeGroup->setPixelSizeRatio(dsize);
    }
}


void PositionDragger::setFixedPixelSizeMode(bool on, double pixelSizeRatio)
{
    if(on){
        double size = impl->handleSize * pixelSizeRatio;
        setPixelSize(size, size * impl->unitHandleWidth);
    } else {
        impl->disableScreenFixedSizeMode();
    }
}


bool PositionDragger::isScreenFixedSizeMode() const
{
    return impl->isScreenFixedSizeMode;
}


bool PositionDragger::isFixedPixelSizeMode() const
{
    return impl->isScreenFixedSizeMode;
}


void PositionDragger::setTransparency(float t)
{
    impl->setMaterialParameters(AllAxes, t, false);
    impl->transparency = t;
    impl->highlightedAxisBitSet.reset();
}


void PositionDragger::Impl::setMaterialParameters(AxisBitSet axisBitSet, float t, bool isHighlighted)
{
    float t2 = convertToTubeTransparency(t);
    
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


void PositionDragger::setDisplayMode(DisplayMode mode, SgUpdateRef update)
{
    if(mode != impl->displayMode){
        impl->displayMode = mode;
        if(mode == DisplayAlways){
            impl->showDragMarkers(true, update);
        } else if(mode == DisplayInEditMode){
            if(impl->isEditMode){
                impl->showDragMarkers(true, update);
            }
        } else if(mode == DisplayNever){
            impl->showDragMarkers(false, update);
        }
    }
}


void PositionDragger::Impl::showDragMarkers(bool on, SgUpdateRef update)
{
    if(displayMode == DisplayNever){
        on = false;
    } else if(displayMode == DisplayAlways){
        on = true;
    }
    
    topSwitch->setTurnedOn(on, update);
}    


void PositionDragger::setDraggerAlwaysShown(bool on, SgUpdateRef update)
{
    if(on){
        setDisplayMode(DisplayAlways, update);
    }
}


bool PositionDragger::isDraggerAlwaysShown() const
{
    return (impl->displayMode == DisplayAlways);
}


void PositionDragger::setDraggerAlwaysHidden(bool on, SgUpdateRef update)
{
    if(on){
        setDisplayMode(DisplayNever, update);
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


Isometry3 PositionDragger::draggingPosition() const
{
    Isometry3 T1;
    bool doNormalization = false;
    if(impl->dragProjector.isDragging()){
        T1 = impl->T_parent.inverse(Eigen::Isometry) * impl->dragProjector.position();
        doNormalization = true;
    } else {
        T1 = T();
    }
    if(impl->hasOffset){
        doNormalization = true;
        T1 = T1 * impl->T_offset.inverse(Eigen::Isometry);
    }
    if(doNormalization){
        normalizeRotation(T1);
    }
    return T1;
}


Isometry3 PositionDragger::globalDraggingPosition() const
{
    Isometry3 T1;
    if(impl->dragProjector.isDragging()){
        T1 = impl->dragProjector.position();
    } else {
        T1 = impl->T_parent * T();
    }
    if(impl->hasOffset){
        T1 = T1 * impl->T_offset.inverse(Eigen::Isometry);
    }
    normalizeRotation(T1);
    return T1;
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


AxisBitSet PositionDragger::Impl::detectTargetAxes(SceneWidgetEvent* event)
{
    AxisBitSet axisBitSet(0);

    auto& path = event->nodePath();
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
        const Isometry3 T_global = calcRelativePosition(event->nodePath(), self);
        const Vector3 p_local = T_global.inverse() * event->point();
        if(!isScreenFixedSizeMode){
            double width = handleSize * unitHandleWidth * calcWidthRatio(event->pixelSizeRatio());
            if(p_local.norm() < 1.5 * width){
                axisBitSet |= AxisBitSet(TRANSLATION_AXES);
            }
        } else {
            double pixelWidth = handleSize * unitHandleWidth * fixedPixelSizeGroup->pixelSizeRatio();
            if(p_local.norm() * event->pixelSizeRatio() < 1.5 * pixelWidth){
                axisBitSet |= AxisBitSet(TRANSLATION_AXES);
            }
        }
    }

    return axisBitSet;
}


bool PositionDragger::onButtonPressEvent(SceneWidgetEvent* event)
{
    return impl->onButtonPressEvent(event);
}


bool PositionDragger::Impl::onButtonPressEvent(SceneWidgetEvent* event)
{
    bool processed = false;

    if(!isDragEnabled){
        return processed;
    }

    auto& path = event->nodePath();
    auto iter = std::find(path.begin(), path.end(), self);
    if(iter != path.end()){
        T_parent = calcRelativePosition(path.begin(), iter);
        const Isometry3 T_global = T_parent * self->T();

        auto axisBitSet = detectTargetAxes(event);
        if(axisBitSet.none()){
            if(self->isContainerMode() && isContentsDragEnabled){
                if(displayMode == DisplayInFocus){
                    SgTmpUpdate update;
                    showDragMarkers(true, update);
                }
                dragProjector.setInitialPosition(T_global);
                dragProjector.setTranslationAlongViewPlane();
                if(dragProjector.startTranslation(event)){
                    processed = true;
                }
            }
        } else {
            if((axisBitSet & AxisBitSet(TRANSLATION_AXES)).any()){
                processed = onTranslationDraggerPressed(event, T_global, axisBitSet);
            } else if((axisBitSet & AxisBitSet(ROTATION_AXES)).any()){
                processed = onRotationDraggerPressed(event, T_global, axisBitSet);
            }
        }
    }

    if(processed){
        sigDragStarted();
    } else {
        dragProjector.resetDragMode();
    }
    
    return processed;
}


bool PositionDragger::Impl::onTranslationDraggerPressed
(SceneWidgetEvent* event, const Isometry3& T_global, AxisBitSet axisBitSet)
{
    bool processed = false;

    dragProjector.setInitialPosition(T_global);
    if(axisBitSet.count() == 1){
        for(int i=0; i < 3; ++i){
            if(axisBitSet[i]){
                dragProjector.setTranslationAxis(T_global.linear().col(i));
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


bool PositionDragger::Impl::onRotationDraggerPressed
(SceneWidgetEvent* event, const Isometry3& T_global, AxisBitSet axisBitSet)
{
    bool processed = false;

    if(axisBitSet.count() == 1){
        dragProjector.setInitialPosition(T_global);
        for(int i=0; i < 3; ++i){
            if(axisBitSet[i + 3]){
                dragProjector.setRotationAxis(T_global.linear().col(i));
                if(dragProjector.startRotation(event)){
                    processed = true;
                }
                break;
            }
        }
    }
    return processed;
}


bool PositionDragger::onButtonReleaseEvent(SceneWidgetEvent*)
{
    if(impl->dragProjector.isDragging()){
        impl->sigDragFinished();
        impl->dragProjector.resetDragMode();
        return true;
    }
    return false;
}


bool PositionDragger::onPointerMoveEvent(SceneWidgetEvent* event)
{
    if(!impl->dragProjector.isDragging()){
        if(impl->isDragEnabled){
            auto axisBitSet = impl->detectTargetAxes(event);
            if(axisBitSet.any()){
                impl->highlightAxes(axisBitSet);
            }
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


void PositionDragger::onPointerLeaveEvent(SceneWidgetEvent*)
{
    if(impl->dragProjector.isDragging()){
        impl->sigDragFinished();
        impl->dragProjector.resetDragMode();
    }

    impl->highlightAxes(0);
}


void PositionDragger::onFocusChanged(SceneWidgetEvent* event, bool on)
{
    if(isContainerMode()){
        if(impl->displayMode == DisplayInFocus){
            SgTmpUpdate update;
            impl->showDragMarkers(on, update);
        }
    }
}


void PositionDragger::onSceneModeChanged(SceneWidgetEvent* event)
{
    SgTmpUpdate update;
    if(event->sceneWidget()->isEditMode()){
        impl->isEditMode = true;
        if(impl->displayMode == DisplayInEditMode){
            impl->showDragMarkers(true, update);
        }
    } else {
        impl->isEditMode = false;
        if(impl->displayMode == DisplayInEditMode || impl->displayMode == DisplayInFocus){
            impl->showDragMarkers(false, update);
        }
    }
}


void PositionDragger::setUndoEnabled(bool /* on */)
{

}


bool PositionDragger::isUndoEnabled() const
{
    return false;
}


void PositionDragger::storeCurrentPositionToHistory()
{

}
