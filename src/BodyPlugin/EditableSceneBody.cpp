/**
   @author Shin'ichiro Nakaoka
*/

#include "EditableSceneBody.h"
#include "BodyItem.h"
#include "BodySelectionManager.h"
#include "KinematicsBar.h"
#include "SimulatorItem.h"
#include <cnoid/JointPath>
#include <cnoid/LinkKinematicsKit>
#include <cnoid/CoordinateFrame>
#include <cnoid/PenetrationBlocker>
#include <cnoid/MenuManager>
#include <cnoid/SceneWidget>
#include <cnoid/SceneEffects>
#include <cnoid/SceneMarkers>
#include <cnoid/SceneDragProjector>
#include <cnoid/PositionDragger>
#include <cnoid/GLSceneRenderer>
#include <cnoid/SceneDevice>
#include <cnoid/LeggedBodyHelper>
#include <cnoid/PinDragIK>
#include <cnoid/EigenUtil>
#include <cnoid/RootItem>
#include <cnoid/ExtensionManager>
#include <cnoid/Archive>
#include <cnoid/ConnectionSet>
#include <cnoid/CheckBoxAction>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

Action* linkVisibilityCheck;

enum LinkOperationType { None, FK, IK, SimInterference };

}

namespace cnoid {

class EditableSceneLink::Impl
{
public:
    EditableSceneLink* self;
    PositionDraggerPtr originMarker;
    SgOutlinePtr outline;
    BoundingBoxMarkerPtr bbMarker;
    bool isPointed;
    bool isColliding;

    Impl(EditableSceneLink* self);
    void showOrigin(bool on);
};

}


EditableSceneLink::EditableSceneLink(Link* link)
    : SceneLink(link)
{
    impl = new Impl(this);
}


EditableSceneLink::Impl::Impl(EditableSceneLink* self)
    : self(self)
{
    isPointed = false;
    isColliding = false;
}


EditableSceneLink::~EditableSceneLink()
{
    delete impl;
}


void EditableSceneLink::showOrigin(bool on)
{
    impl->showOrigin(on);
}


void EditableSceneLink::Impl::showOrigin(bool on)
{
    if(on){
        if(!originMarker){
            originMarker = new PositionDragger(
                PositionDragger::TranslationAxes, PositionDragger::PositiveOnlyHandle);
            originMarker->setOverlayMode(true);
            originMarker->setHandleWidthRatio(0.05);
            originMarker->setConstantPixelSizeMode(true, 48);
            originMarker->setDisplayMode(PositionDragger::DisplayInEditMode);
            originMarker->setTransparency(0.0f);
            originMarker->setDragEnabled(false);
        }
        self->addChildOnce(originMarker, true);
    } else {
        if(originMarker && originMarker->hasParents()){
            self->removeChild(originMarker, true);
        }
    }
}


bool EditableSceneLink::isOriginShown() const
{
    return (impl->originMarker && impl->originMarker->hasParents());
}


void EditableSceneLink::showOutline(bool on)
{
    if(!visualShape()){
        return;
    }
    SgOutlinePtr& outline = impl->outline;
    if(on){
        if(!outline){
            outline = new SgOutline;
            outline->setColor(Vector3f(1.0f, 1.0f, 0.0f));
        }
        if(!outline->hasParents()){
            insertEffectGroup(outline, true);
        }
    } else {
        if(outline && outline->hasParents()){
            removeEffectGroup(outline, true);
        }
    }
}


void EditableSceneLink::showMarker(const Vector3f& color, float transparency)
{
    if(impl->bbMarker){
        removeChild(impl->bbMarker);
    }
    if(visualShape()){
        impl->bbMarker = new BoundingBoxMarker(visualShape()->boundingBox(), color, transparency);
        addChildOnce(impl->bbMarker, true);
    }
}


void EditableSceneLink::hideMarker()
{
    if(impl->bbMarker){
        removeChild(impl->bbMarker, true);
        impl->bbMarker = nullptr;
    }
}


void EditableSceneLink::setColliding(bool on)
{
    if(!impl->isColliding && on){
        if(!impl->isPointed){
            
        }
        impl->isColliding = true;
    } else if(impl->isColliding && !on){
        if(!impl->isPointed){
            
        }
        impl->isColliding = false;
    }
}


namespace cnoid {

class EditableSceneBody::Impl
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Impl(EditableSceneBody* self, BodyItem* bodyItem);
        
    EditableSceneBody* self;
    BodyItemPtr bodyItem;

    SgUpdate modified;

    ScopedConnectionSet connections;
    ScopedConnection connectionToSigCollisionsUpdated;
    vector<bool> collisionLinkBitSet;
    ScopedConnection connectionToSigLinkSelectionChanged;

    enum PointedType { PT_NONE, PT_SCENE_LINK, PT_ZMP };
    EditableSceneLink* pointedSceneLink;
    EditableSceneLink* outlinedLink;

    SgGroupPtr markerGroup;
    CrossMarkerPtr cmMarker;
    CrossMarkerPtr ppcomMarker;
    bool isCmVisible;
    bool isPpcomVisible;
    SgLineSetPtr virtualElasticStringLine;
    SphereMarkerPtr zmpMarker;
    bool isZmpVisible;
    Vector3 orgZmpPos;

    Link* targetLink;
    double orgJointPosition;
        
    LinkTraverse fkTraverse;
    shared_ptr<InverseKinematics> currentIK;
    shared_ptr<PinDragIK> pinDragIK;
    shared_ptr<PenetrationBlocker> penetrationBlocker;
    PositionDraggerPtr positionDragger;
    ScopedConnection kinematicsKitConnection;

    bool isEditMode;
    bool isFocused;

    KinematicsBar* kinematicsBar;

    enum DragMode {
        DRAG_NONE,
        LINK_IK_TRANSLATION,
        LINK_FK_ROTATION,
        LINK_FK_TRANSLATION,
        LINK_VIRTUAL_ELASTIC_STRING,
        LINK_FORCED_POSITION,
        ZMP_TRANSLATION
    };
    DragMode dragMode;
    SceneDragProjector dragProjector;
    bool isDragging;

    weak_ref_ptr<SimulatorItem> activeSimulatorItem;
    Vector3 pointedLinkLocalPoint;
    enum { NO_FORCED_POSITION, MOVE_FORCED_POSITION, KEEP_FORCED_POSITION };
    int forcedPositionMode;

    EditableSceneLink* editableSceneLink(int index){
        return static_cast<EditableSceneLink*>(self->sceneLink(index));
    }

    double calcLinkMarkerRadius(SceneLink* sceneLink) const;
    void onSceneGraphConnection(bool on);
    void updateModel();
    void onBodyItemUpdated();
    void onKinematicStateChanged();

    void onCollisionsUpdated();
    void onCollisionLinkHighlightModeChanged();
    void changeCollisionLinkHighlightMode(bool on);
    void onLinkVisibilityCheckToggled();
    void onLinkSelectionChanged(const std::vector<bool>& selection);
    void onLinkOriginsCheckChanged(bool on);

    void showCenterOfMass(bool on);
    void showPpcom(bool on);
    void showZmp(bool on);
    void makeLinkFree(EditableSceneLink* sceneLink);
    void setBaseLink(EditableSceneLink* sceneLink);
    void toggleBaseLink(EditableSceneLink* sceneLink);
    void togglePin(EditableSceneLink* sceneLink, bool toggleTranslation, bool toggleRotation);
    void makeLinkAttitudeLevel();
        
    PointedType findPointedObject(const vector<SgNode*>& path);
    int checkLinkOperationType(SceneLink* sceneLink);
    int checkLinkKinematicsType(Link* link);
    void updateMarkersAndManipulators(bool on);
    void attachPositionDragger(Link* link);

    bool onKeyPressEvent(const SceneWidgetEvent& event);
    bool onKeyReleaseEvent(const SceneWidgetEvent& event);
    bool onButtonPressEvent(const SceneWidgetEvent& event);
    bool onButtonReleaseEvent(const SceneWidgetEvent& event);
    bool onPointerMoveEvent(const SceneWidgetEvent& event);
    void onPointerLeaveEvent(const SceneWidgetEvent& event);
    bool onScrollEvent(const SceneWidgetEvent& event);
    void onContextMenuRequest(const SceneWidgetEvent& event, MenuManager& menuManager);
    void onSceneModeChanged(const SceneWidgetEvent& event);
    bool onUndoRequest();
    bool onRedoRequest();
    void onDraggerDragStarted();
    void onDraggerDragged();
    void onDraggerDragFinished();

    bool initializeIK();
    void startIK(const SceneWidgetEvent& event);
    void dragIK(const SceneWidgetEvent& event);
    void doIK(const Position& position);
    void startFK(const SceneWidgetEvent& event);
    void dragFKRotation(const SceneWidgetEvent& event);
    void dragFKTranslation(const SceneWidgetEvent& event);
    void startLinkOperationDuringSimulation(const SceneWidgetEvent& event);
    void setForcedPositionMode(int mode, bool on);
    void startVirtualElasticString(const SceneWidgetEvent& event);
    void dragVirtualElasticString(const SceneWidgetEvent& event);
    void finishVirtualElasticString();
    void startForcedPosition(const SceneWidgetEvent& event);
    void setForcedPosition(const Position& position);
    void dragForcedPosition(const SceneWidgetEvent& event);
    void finishForcedPosition();
    void startZmpTranslation(const SceneWidgetEvent& event);
    void dragZmpTranslation(const SceneWidgetEvent& event);
    bool finishEditing();
    
    static bool storeProperties(Archive& archive);
    static void restoreProperties(const Archive& archive);
    static void restoreSceneBodyProperties(const Archive& archive);
};

}


static SceneLink* createEditableSceneLink(Link* link)
{
    return new EditableSceneLink(link);
}
    
    
EditableSceneBody::EditableSceneBody(BodyItem* bodyItem)
    : SceneBody(bodyItem->body(), createEditableSceneLink)
{
    setName(body()->name());
    impl = new Impl(this, bodyItem);
}


EditableSceneBody::Impl::Impl(EditableSceneBody* self, BodyItem* bodyItem)
    : self(self),
      bodyItem(bodyItem),
      modified(SgUpdate::MODIFIED),
      kinematicsBar(KinematicsBar::instance())
{
    pointedSceneLink = nullptr;
    outlinedLink = nullptr;
    targetLink = nullptr;

    if(GLSceneRenderer::rendererType() == GLSceneRenderer::GL1_RENDERER){
        /** GL1SceneRenderer does not support the overlay rendering with SgOverlay and use the
            old type dragger to render it correctly. */
        positionDragger = new PositionDragger(PositionDragger::AllAxes, PositionDragger::WideHandle);
        positionDragger->setOverlayMode(false);
    } else {
        positionDragger = new PositionDragger(PositionDragger::AllAxes, PositionDragger::PositiveOnlyHandle);
        positionDragger->setOverlayMode(true);
    }
    positionDragger->setDisplayMode(PositionDragger::DisplayAlways);
    positionDragger->sigDragStarted().connect([&](){ onDraggerDragStarted(); });
    positionDragger->sigPositionDragged().connect([&](){ onDraggerDragged(); });
    positionDragger->sigDragFinished().connect([&](){ onDraggerDragFinished(); });
    
    dragMode = DRAG_NONE;
    isDragging = false;
    isEditMode = false;
    isFocused = false;

    markerGroup = new SgGroup;
    markerGroup->setName("Marker");
    self->addChild(markerGroup);

    double radius = 0.0;
    const int n = self->numSceneLinks();
    for(int i=0; i < n; ++i){
        SceneLink* sLink = self->sceneLink(i);
        BoundingBox bb = sLink->boundingBox();
        double radius0 = bb.size().norm() / 2.0;
        if(radius0 > radius){
            radius = radius0;
        }
    }
    cmMarker = new CrossMarker(radius, Vector3f(0.0f, 1.0f, 0.0f), 2.0);
    cmMarker->setName("centerOfMass");
    isCmVisible = false;
    ppcomMarker = new CrossMarker(radius, Vector3f(1.0f, 0.5f, 0.0f), 2.0);
    ppcomMarker->setName("ProjectionPointCoM");
    isPpcomVisible = false;

    forcedPositionMode = NO_FORCED_POSITION;
    virtualElasticStringLine = new SgLineSet;
    virtualElasticStringLine->getOrCreateVertices()->resize(2);
    virtualElasticStringLine->addLine(0, 1);

    LeggedBodyHelperPtr legged = getLeggedBodyHelper(self->body());
    if(legged->isValid() && legged->numFeet() > 0){
        Link* footLink = legged->footLink(0);
        const double r = calcLinkMarkerRadius(self->sceneLink(footLink->index()));
        zmpMarker = new SphereMarker(r, Vector3f(0.0f, 1.0f, 0.0f), 0.3);
        zmpMarker->setName("ZMP");
        zmpMarker->addChild(new CrossMarker(r * 2.5, Vector3f(0.0f, 1.0f, 0.0f), 2.0f));
    } else {
        zmpMarker = new SphereMarker(0.1, Vector3f(0.0f, 1.0f, 0.0f), 0.3);
    }
    isZmpVisible = false;

    self->sigGraphConnection().connect([&](bool on){ onSceneGraphConnection(on);});
}


double EditableSceneBody::Impl::calcLinkMarkerRadius(SceneLink* sceneLink) const
{
    SgNode* shape = sceneLink->visualShape();
    if(shape){
        const BoundingBox& bb = shape->boundingBox();
        if (bb.empty()) return 1.0; // Is this OK?
        double V = ((bb.max().x() - bb.min().x()) * (bb.max().y() - bb.min().y()) * (bb.max().z() - bb.min().z()));
        return pow(V, 1.0 / 3.0) * 0.6;
    }
    return 1.0;
}


void EditableSceneBody::Impl::onSceneGraphConnection(bool on)
{
    connections.disconnect();
    connectionToSigLinkSelectionChanged.disconnect();

    if(on){
        connections.add(
            bodyItem->sigUpdated().connect(
                [&](){ onBodyItemUpdated(); }));

        onBodyItemUpdated();

        connections.add(
            bodyItem->sigKinematicStateChanged().connect(
                [&](){ onKinematicStateChanged(); }));
            
        onKinematicStateChanged();

        connections.add(
            bodyItem->getLocationProxy()->sigAttributeChanged().connect(
                [&](){
                    bool on = bodyItem->isLocationEditable();
                    if(!on){
                        if(outlinedLink){
                            outlinedLink->showOutline(false);
                            outlinedLink = nullptr;
                        }
                        updateMarkersAndManipulators(false);
                    }
                }));

        connections.add(
            kinematicsBar->sigCollisionVisualizationChanged().connect(
                [&](){ onCollisionLinkHighlightModeChanged(); }));
        
        onCollisionLinkHighlightModeChanged();

        connections.add(
            bodyItem->sigModelUpdated().connect(
                [&](){ updateModel(); }));

        connections.add(
            linkVisibilityCheck->sigToggled().connect(
                [&](bool){ onLinkVisibilityCheckToggled(); }));
        
        onLinkVisibilityCheckToggled();
    }
}


void EditableSceneBody::updateModel()
{
    impl->updateModel();
}


void EditableSceneBody::Impl::updateModel()
{
    pointedSceneLink = nullptr;
    targetLink = nullptr;
    if(outlinedLink){
        outlinedLink->showOutline(false);
        outlinedLink = nullptr;
    }
    isDragging = false;
    dragMode = DRAG_NONE;
    
    self->SceneBody::updateModel();
}


void EditableSceneBody::Impl::onBodyItemUpdated()
{
    updateMarkersAndManipulators(isFocused);
}


void EditableSceneBody::Impl::onKinematicStateChanged()
{
    if(isCmVisible){
        cmMarker->setTranslation(bodyItem->centerOfMass());
    }
    if(isPpcomVisible){
    	Vector3 com = bodyItem->centerOfMass();
    	com(2) = 0.0;
    	ppcomMarker->setTranslation(com);
    }
    if(isZmpVisible){
        zmpMarker->setTranslation(bodyItem->zmp());
    }

    if(activeSimulatorItem){
        if(dragMode == LINK_VIRTUAL_ELASTIC_STRING){
            if(virtualElasticStringLine->hasParents()){
                virtualElasticStringLine->vertices()->at(0)
                    = (targetLink->T() * pointedLinkLocalPoint).cast<Vector3f::Scalar>();
            }
        }
    }

    self->updateLinkPositions(modified);
}


EditableSceneLink* EditableSceneBody::editableSceneLink(int index)
{
    return static_cast<EditableSceneLink*>(sceneLink(index));
}


void EditableSceneBody::Impl::onCollisionsUpdated()
{
    if(bodyItem->collisionLinkBitSet() != collisionLinkBitSet){
        collisionLinkBitSet = bodyItem->collisionLinkBitSet();
        const int n = self->numSceneLinks();
        for(int i=0; i < n; ++i){
            editableSceneLink(i)->setColliding(collisionLinkBitSet[i]);
        }
        self->notifyUpdate(modified);
    }
}


void EditableSceneBody::Impl::onCollisionLinkHighlightModeChanged()
{
    changeCollisionLinkHighlightMode(kinematicsBar->isCollisionLinkHighlihtMode());
}


void EditableSceneBody::Impl::changeCollisionLinkHighlightMode(bool on)
{
    if(!connectionToSigCollisionsUpdated.connected() && on){
        connectionToSigCollisionsUpdated =
            bodyItem->sigCollisionsUpdated().connect(
                [&](){ onCollisionsUpdated(); });
        onCollisionsUpdated();

    } else if(connectionToSigCollisionsUpdated.connected() && !on){
        connectionToSigCollisionsUpdated.disconnect();
        const int n = self->numSceneLinks();
        for(int i=0; i < n; ++i){
            editableSceneLink(i)->setColliding(false);
        }
        self->notifyUpdate(modified);
    }
}


EditableSceneBody::~EditableSceneBody()
{
    delete impl;
}


void EditableSceneBody::setLinkVisibilities(const std::vector<bool>& visibilities)
{
    int i;
    const int m = numSceneLinks();
    const int n = std::min(m, (int)visibilities.size());
    for(i=0; i < n; ++i){
        sceneLink(i)->setVisible(visibilities[i]);
    }
    while(i < m){
        sceneLink(i)->setVisible(false);
        ++i;
    }
    notifyUpdate(impl->modified);
}


void EditableSceneBody::Impl::onLinkVisibilityCheckToggled()
{
    auto bsm = BodySelectionManager::instance();

    if(linkVisibilityCheck->isChecked()){
        connectionToSigLinkSelectionChanged.reset(
            bsm->sigLinkSelectionChanged(bodyItem).connect(
                [&](const std::vector<bool>& selection){
                    onLinkSelectionChanged(selection);
                }));
        onLinkSelectionChanged(bsm->linkSelection(bodyItem));
    } else {
        connectionToSigLinkSelectionChanged.disconnect();
        self->setLinkVisibilities(vector<bool>(self->numSceneLinks(), true));
    }
}


void EditableSceneBody::Impl::onLinkSelectionChanged(const std::vector<bool>& selection)
{
    if(linkVisibilityCheck->isChecked()){
        self->setLinkVisibilities(selection);
    }
}


void EditableSceneBody::Impl::onLinkOriginsCheckChanged(bool on)
{
    for(int i=0; i < self->numSceneLinks(); ++i){
        self->editableSceneLink(i)->showOrigin(on);
    }
}


void EditableSceneBody::Impl::showCenterOfMass(bool on)
{
    isCmVisible = on;
    if(on){
        cmMarker->setTranslation(bodyItem->centerOfMass());
        markerGroup->addChildOnce(cmMarker, true);
    } else {
        markerGroup->removeChild(cmMarker, true);
    }
}


void EditableSceneBody::Impl::showPpcom(bool on)
{
    isPpcomVisible = on;
    if(on){
        Vector3 com = bodyItem->centerOfMass();
        com(2) = 0.0;
        ppcomMarker->setTranslation(com);
        markerGroup->addChildOnce(ppcomMarker, true);
    } else {
        markerGroup->removeChild(ppcomMarker, true);
    }
}


void EditableSceneBody::Impl::showZmp(bool on)
{
    isZmpVisible = on;
    if(on){
        zmpMarker->setTranslation(bodyItem->zmp());
        markerGroup->addChildOnce(zmpMarker, true);
    } else {
        markerGroup->removeChild(zmpMarker, true);
    }
}


void EditableSceneBody::Impl::makeLinkFree(EditableSceneLink* sceneLink)
{
    if(bodyItem->currentBaseLink() == sceneLink->link()){
        bodyItem->setCurrentBaseLink(nullptr);
    }
    bodyItem->pinDragIK()->setPin(sceneLink->link(), PinDragIK::NO_AXES);
    bodyItem->notifyUpdate();
}


void EditableSceneBody::Impl::setBaseLink(EditableSceneLink* sceneLink)
{
    bodyItem->setCurrentBaseLink(sceneLink->link());
    bodyItem->notifyUpdate();
}
    

void EditableSceneBody::Impl::toggleBaseLink(EditableSceneLink* sceneLink)
{
    Link* baseLink = bodyItem->currentBaseLink();
    if(sceneLink->link() != baseLink){
        bodyItem->setCurrentBaseLink(sceneLink->link());
    } else {
        bodyItem->setCurrentBaseLink(nullptr);
    }
    bodyItem->notifyUpdate();
}


void EditableSceneBody::Impl::togglePin(EditableSceneLink* sceneLink, bool toggleTranslation, bool toggleRotation)
{
    auto pin = bodyItem->pinDragIK();

    PinDragIK::AxisSet axes = pin->pinAxes(sceneLink->link());

    if(toggleTranslation && toggleRotation){
        if(axes == PinDragIK::NO_AXES){
            axes = PinDragIK::TRANSFORM_6D;
        } else {
            axes = PinDragIK::NO_AXES;
        }
    } else {
        if(toggleTranslation){
            axes = (PinDragIK::AxisSet)(axes ^ PinDragIK::TRANSLATION_3D);
        }
        if(toggleRotation){
            axes = (PinDragIK::AxisSet)(axes ^ PinDragIK::ROTATION_3D);
        }
    }
        
    pin->setPin(sceneLink->link(), axes);
    bodyItem->notifyUpdate();
}


void EditableSceneBody::Impl::makeLinkAttitudeLevel()
{
    if(pointedSceneLink){
        Link* link = outlinedLink->link();
        auto ik = bodyItem->getCurrentIK(link);
        if(ik){
            const Position& T = link->T();
            const double theta = acos(T(2, 2));
            const Vector3 z(T(0,2), T(1, 2), T(2, 2));
            const Vector3 axis = z.cross(Vector3::UnitZ()).normalized();
            const Matrix3 R2 = AngleAxisd(theta, axis) * T.linear();
            Position T2;
            T2.linear() = R2;
            T2.translation() = link->p();

            bodyItem->beginKinematicStateEdit();
            if(ik->calcInverseKinematics(T2)){
                bool fkDone = ik->calcRemainingPartForwardKinematicsForInverseKinematics();
                bodyItem->notifyKinematicStateChange(!fkDone);
                bodyItem->acceptKinematicStateEdit();
            }
        }
    }
}


EditableSceneBody::Impl::PointedType EditableSceneBody::Impl::findPointedObject(const vector<SgNode*>& path)
{
    PointedType pointedType = PT_NONE;
    pointedSceneLink = nullptr;
    for(size_t i = path.size() - 1; i >= 1; --i){
        pointedSceneLink = dynamic_cast<EditableSceneLink*>(path[i]);
        if(pointedSceneLink){
            pointedType = PT_SCENE_LINK;
            break;
        }
        SphereMarker* marker = dynamic_cast<SphereMarker*>(path[i]);
        if(marker == zmpMarker){
            pointedType = PT_ZMP;
            break;
        }
    }
    return pointedType;
}


int EditableSceneBody::Impl::checkLinkOperationType(SceneLink* sceneLink)
{
    currentIK.reset();
    
    if(!sceneLink){
        return LinkOperationType::None;
    }
    auto link = sceneLink->link();
    if(!link){
        return LinkOperationType::None;
    }

    int type = LinkOperationType::None;
    activeSimulatorItem = SimulatorItem::findActiveSimulatorItemFor(bodyItem);
    if(activeSimulatorItem){
        if(link->body()->isStaticModel() || (link->isRoot() && link->isFixedJoint())){
            type = LinkOperationType::None;
        } else {
            if(checkLinkKinematicsType(link) != LinkOperationType::None){
                type = LinkOperationType::SimInterference;
            }
        }
    } else {
        type = checkLinkKinematicsType(link);
    }

    return type;
}


int EditableSceneBody::Impl::checkLinkKinematicsType(Link* link)
{
    // Check if the link is editable considering the ancestor bodies
    BodyItem* bodyItemChain = bodyItem;
    Link* linkChain = link;
    while(true){
        if(!bodyItemChain->isAttachedToParentBody()){
            if(!bodyItemChain->isLocationEditable() && linkChain->isBodyRoot()){
                return LinkOperationType::None;
            }
            break;
        }
        if(!linkChain->isBodyRoot()){
            break;
        }
        bodyItemChain = bodyItemChain->parentBodyItem();
        linkChain = linkChain->body()->parentBodyLink();
    }

    int mode = kinematicsBar->mode();
    int type = LinkOperationType::None;

    if(mode == KinematicsBar::PresetKinematics){
        currentIK = bodyItem->findPresetIK(link);
        if(currentIK && kinematicsBar->isInverseKinematicsEnabled()){
            type = LinkOperationType::IK;
        } else if(link->isBodyRoot()){
            type = LinkOperationType::IK;
        } else if(kinematicsBar->isForwardKinematicsEnabled()){
            type = LinkOperationType::FK;
        }
    } else if(mode == KinematicsBar::ForwardKinematics){
        auto baseLink = bodyItem->currentBaseLink();
        if(link->isBodyRoot()){
            if(!baseLink || link == baseLink){
                type = LinkOperationType::IK;
            }
        } else {
            if(baseLink && link == baseLink){
                type = LinkOperationType::IK;
            } else if(kinematicsBar->isForwardKinematicsEnabled()){
                type = LinkOperationType::FK;
            }
        }
    } else if(mode == KinematicsBar::InverseKinematics){
        if(!link->isBodyRoot() || bodyItem->isAttachedToParentBody()){
            type = LinkOperationType::IK;
        }
    }
    
    return type;
}


void EditableSceneBody::Impl::updateMarkersAndManipulators(bool on)
{
    Link* baseLink = bodyItem->currentBaseLink();
    auto pin = bodyItem->pinDragIK();

    const int n = self->numSceneLinks();
    for(int i=0; i < n; ++i){
        EditableSceneLink* sceneLink = editableSceneLink(i);
        sceneLink->hideMarker();
        sceneLink->removeChild(positionDragger);

        if(on && isEditMode && !activeSimulatorItem){
            Link* link = sceneLink->link();
            if(link == baseLink){
                sceneLink->showMarker(Vector3f(1.0f, 0.1f, 0.1f), 0.4);
            } else {
                int pinAxes = pin->pinAxes(link);
                if(pinAxes & (PinDragIK::TRANSFORM_6D)){
                    sceneLink->showMarker(Vector3f(1.0f, 1.0f, 0.1f), 0.4);
                }
            }
        }
        if(sceneLink->impl->originMarker){
            sceneLink->impl->originMarker->setDisplayMode(PositionDragger::DisplayInEditMode);
        }
    }

    kinematicsKitConnection.disconnect();

    self->notifyUpdate(modified);
}


void EditableSceneBody::Impl::attachPositionDragger(Link* link)
{
    LinkKinematicsKit* kinematicsKit = nullptr;
    if(link->isBodyRoot() && bodyItem->isAttachedToParentBody()){
        auto parentBodyLink = bodyItem->body()->parentBodyLink();
        if(!parentBodyLink->isBodyRoot()){
            auto parentBodyItem = bodyItem->parentBodyItem();
            kinematicsKit = parentBodyItem->getCurrentLinkKinematicsKit(parentBodyLink);
            if(kinematicsKit){
                positionDragger->setPosition(
                    link->Tb().inverse(Eigen::Isometry) * kinematicsKit->currentOffsetFrame()->T());
            }
        }
    }
    if(!kinematicsKit){
        kinematicsKit = bodyItem->getCurrentLinkKinematicsKit(link);
        if(kinematicsKit){
            positionDragger->setPosition(kinematicsKit->currentOffsetFrame()->T());
        } else {
            positionDragger->setPosition(Affine3::Identity());
        }
    }

    positionDragger->setOffset(positionDragger->T());

    // Even if the connection to sigFrameUpdate is remade, it must
    // first be disconnected to avoid the infinite loop to call the newly
    // connected slot when this function is called by the signal.
    kinematicsKitConnection.disconnect();
    if(kinematicsKit){
        kinematicsKitConnection =
            kinematicsKit->sigFrameUpdate().connect(
                [this, link](){ attachPositionDragger(link); });
    }
    
    auto sceneLink = editableSceneLink(link->index());
    if(!positionDragger->isConstantPixelSizeMode()){
        if(auto shape = sceneLink->visualShape()){
            if(auto transform = dynamic_cast<SgTransform*>(shape)){
                positionDragger->adjustSize(transform->untransformedBoundingBox());
            } else {
                positionDragger->adjustSize(shape->boundingBox());
            }
        }
    }

    positionDragger->notifyUpdate();
    sceneLink->addChildOnce(positionDragger);

    if(sceneLink->impl->originMarker){
        sceneLink->impl->originMarker->setDisplayMode(PositionDragger::DisplayNever);
    }
}


bool EditableSceneBody::onKeyPressEvent(const SceneWidgetEvent& event)
{
    return impl->onKeyPressEvent(event);
}


bool EditableSceneBody::Impl::onKeyPressEvent(const SceneWidgetEvent& event)
{
    if(!outlinedLink){
        return false;
    }

    bool handled = true;

    switch(event.key()){
    case Qt::Key_B:
        toggleBaseLink(outlinedLink);
        break;
        
    case Qt::Key_R:
        togglePin(outlinedLink, false, true);
        break;

    case Qt::Key_T:
        togglePin(outlinedLink, true, false);
        break;

    default:
        handled = false;
        break;
    }
        
    return handled;

}


bool EditableSceneBody::onKeyReleaseEvent(const SceneWidgetEvent& event)
{
    return impl->onKeyReleaseEvent(event);
}


bool EditableSceneBody::Impl::onKeyReleaseEvent(const SceneWidgetEvent& event)
{
    return false;
}


bool EditableSceneBody::onButtonPressEvent(const SceneWidgetEvent& event)
{
    return impl->onButtonPressEvent(event);
}


bool EditableSceneBody::Impl::onButtonPressEvent(const SceneWidgetEvent& event)
{
    if(outlinedLink){
        outlinedLink->showOutline(false);
        outlinedLink = nullptr;
    }
    
    PointedType pointedType = findPointedObject(event.nodePath());

    if(pointedType == PT_ZMP && event.button() == Qt::LeftButton){
        startZmpTranslation(event);
        return true;
    }    

    if(!pointedSceneLink){
        return false;
    }
    targetLink = pointedSceneLink->link();

    int operationType = LinkOperationType::None;
    bool showOutline = false;
    auto bsm = BodySelectionManager::instance();
        
    if(event.button() == Qt::RightButton){
        // The context menu is about to be shown
        bsm->setCurrent(bodyItem, targetLink, true);
        showOutline = true;
    } else {
        operationType = checkLinkOperationType(pointedSceneLink);
        if(operationType != LinkOperationType::None){
            showOutline = true;
        }
    }
    
    if(showOutline){
        pointedSceneLink->showOutline(true);
        outlinedLink = pointedSceneLink;
    }
    if(operationType == LinkOperationType::None){
        return false;
    }

    bool handled = false;
    isDragging = false;

    if(operationType == LinkOperationType::SimInterference){
        startLinkOperationDuringSimulation(event);
        handled = true;
    } else {
        if(event.button() == Qt::LeftButton){
            updateMarkersAndManipulators(true);

            //if(!bodyItem->isAttachedToParentBody()){
            if(true){
                bsm->setCurrent(bodyItem, targetLink, true);
            } else{
                bsm->setCurrent(
                    bodyItem->parentBodyItem(),
                    bodyItem->body()->parentBodyLink(),
                    true);
            }
            if(operationType == LinkOperationType::FK){
                startFK(event);
                handled = true;
            } else if(operationType == LinkOperationType::IK){
                startIK(event);
                handled = true;
            }
        }
    }

    if(dragMode != DRAG_NONE && outlinedLink){
        outlinedLink->showOutline(false);
        self->notifyUpdate(modified);
    }

    return handled;
}


bool EditableSceneBody::onButtonReleaseEvent(const SceneWidgetEvent& event)
{
    return impl->onButtonReleaseEvent(event);
}


bool EditableSceneBody::Impl::onButtonReleaseEvent(const SceneWidgetEvent& event)
{
    bool handled = finishEditing();

    if(outlinedLink){
        outlinedLink->showOutline(true);
        self->notifyUpdate(modified);
    }

    return handled;
}


bool EditableSceneBody::onDoubleClickEvent(const SceneWidgetEvent& event)
{
    if(impl->targetLink){
        if(impl->checkLinkKinematicsType(impl->targetLink) != LinkOperationType::None){
            // prevent returning from the edit mode to the view mode
            return true;
        }
    }
    return false;
            
    // return impl->makePointedLinkCurrent();
}


/*
bool EditableSceneBody::Impl::makePointedLinkCurrent()
{
    if(event.button() == Qt::LeftButton){
        if(findPointedObject(event.nodePath()) == PT_SCENE_LINK){
            BodySelectionManager::instance()->setCurrent(bodyItem, targetLink, true);
            return true
        }
    }
    return false;
}
*/


bool EditableSceneBody::onPointerMoveEvent(const SceneWidgetEvent& event)
{
    return impl->onPointerMoveEvent(event);
}


bool EditableSceneBody::Impl::onPointerMoveEvent(const SceneWidgetEvent& event)
{
    if(dragMode == DRAG_NONE){
        findPointedObject(event.nodePath());
        if(!pointedSceneLink){
            event.updateIndicator("");
        } else {
            if(checkLinkOperationType(pointedSceneLink) != LinkOperationType::None){
                if(pointedSceneLink != outlinedLink){
                    if(outlinedLink){
                        outlinedLink->showOutline(false);
                    }
                    pointedSceneLink->showOutline(true);
                    outlinedLink = pointedSceneLink;
                }
            }
            const Vector3 p = pointedSceneLink->T().inverse() * event.point();
            event.updateIndicator(
                fmt::format("{0} / {1} : ({2:.3f}, {3:.3f}, {4:.3f})",
                            bodyItem->displayName(), pointedSceneLink->link()->name(),
                            p.x(), p.y(), p.z()));
        }
    } else {
        if(!isDragging){
            bodyItem->beginKinematicStateEdit();
            isDragging = true;
        }

        switch(dragMode){

        case LINK_IK_TRANSLATION:
            dragIK(event);
            break;
            
        case LINK_FK_ROTATION:
            dragFKRotation(event);
            break;

        case LINK_FK_TRANSLATION:
            dragFKTranslation(event);
            break;

        case LINK_VIRTUAL_ELASTIC_STRING:
            dragVirtualElasticString(event);
            break;

        case LINK_FORCED_POSITION:
            dragForcedPosition(event);
            break;
            
        case ZMP_TRANSLATION:
            dragZmpTranslation(event);
            break;
            
        default:
            break;
        }
    }

    return true;
}


void EditableSceneBody::onPointerLeaveEvent(const SceneWidgetEvent& event)
{
    return impl->onPointerLeaveEvent(event);
}


void EditableSceneBody::Impl::onPointerLeaveEvent(const SceneWidgetEvent& event)
{
    if(outlinedLink){
        outlinedLink->showOutline(false);
        outlinedLink = nullptr;
    }
}


bool EditableSceneBody::onScrollEvent(const SceneWidgetEvent& event)
{
    return impl->onScrollEvent(event);
}


bool EditableSceneBody::Impl::onScrollEvent(const SceneWidgetEvent& event)
{
    return false;
}


void EditableSceneBody::onFocusChanged(const SceneWidgetEvent& event, bool on)
{
    impl->isFocused = on;
    if(!on){
        impl->updateMarkersAndManipulators(false);
    }
}


void EditableSceneBody::onContextMenuRequest(const SceneWidgetEvent& event, MenuManager& menuManager)
{
    impl->onContextMenuRequest(event, menuManager);
}


void EditableSceneBody::Impl::onContextMenuRequest(const SceneWidgetEvent& event, MenuManager& mm)
{
    PointedType pointedType = findPointedObject(event.nodePath());

    if(pointedType != PT_SCENE_LINK){
        return;
    }

    auto locationLockCheck = mm.addCheckItem(_("Lock location"));
    locationLockCheck->setChecked(!bodyItem->isLocationEditable());
    locationLockCheck->sigToggled().connect(
        [&](bool on){ bodyItem->setLocationEditable(!on); });
                    
    activeSimulatorItem = SimulatorItem::findActiveSimulatorItemFor(bodyItem);
    if(activeSimulatorItem){
        if(pointedSceneLink->link()->isBodyRoot() && bodyItem->isLocationEditable()){
            Action* item1 = mm.addCheckItem(_("Move Forcibly"));
            item1->setChecked(forcedPositionMode == MOVE_FORCED_POSITION);
            item1->sigToggled().connect(
                [&](bool on){ setForcedPositionMode(MOVE_FORCED_POSITION, on); });
                    
            Action* item2 = mm.addCheckItem(_("Hold Forcibly"));
            item2->setChecked(forcedPositionMode == KEEP_FORCED_POSITION);
            item2->sigToggled().connect(
                [&](bool on){ setForcedPositionMode(KEEP_FORCED_POSITION, on); });
                    
            mm.addSeparator();
        }
    } else {
        mm.addItem(_("Set Free"))->sigTriggered().connect(
            [&](){ makeLinkFree(pointedSceneLink); });
        mm.addItem(_("Set Base"))->sigTriggered().connect(
            [&](){ setBaseLink(pointedSceneLink); });
        mm.addItem(_("Set Translation Pin"))->sigTriggered().connect(
            [&](){ togglePin(pointedSceneLink, true, false); });
        mm.addItem(_("Set Rotation Pin"))->sigTriggered().connect(
            [&](){ togglePin(pointedSceneLink, false, true); });
        mm.addItem(_("Set Both Pins"))->sigTriggered().connect(
            [&](){ togglePin(pointedSceneLink, true, true); });
                
        mm.addSeparator();
            
        mm.addItem(_("Level Attitude"))->sigTriggered().connect(
            [&](){ makeLinkAttitudeLevel(); });
            
        mm.addSeparator();
    }

    mm.setPath(_("Markers"));

    auto linkOriginAction = new CheckBoxAction(_("Link Origins"));
    int checkState = Qt::Unchecked;
    int numLinks = self->numSceneLinks();
    int numOriginShowns = 0;
    for(int i=0; i < numLinks; ++i){
        if(self->editableSceneLink(i)->isOriginShown()){
            ++numOriginShowns;
        }
    }
    auto check = linkOriginAction->checkBox();
    check->setTristate();
    if(numOriginShowns == 0){
        check->setCheckState(Qt::Unchecked);
    } else if(numOriginShowns == numLinks){
        check->setCheckState(Qt::Checked);
    } else {
        check->setCheckState(Qt::PartiallyChecked);
    }
    mm.addAction(linkOriginAction);
    check->sigToggled().connect([&](bool on){ onLinkOriginsCheckChanged(on); });
        
    auto item = mm.addCheckItem(_("Center of Mass"));
    item->setChecked(isCmVisible);
    item->sigToggled().connect([&](bool on){ showCenterOfMass(on); });
            
    item = mm.addCheckItem(_("Projection Point of CoM"));
    item->setChecked(isPpcomVisible);
    item->sigToggled().connect([&](bool on){ showPpcom(on); });
            
    item = mm.addCheckItem(_("ZMP"));
    item->setChecked(isZmpVisible);
    item->sigToggled().connect([&](bool on){ showZmp(on); });

    mm.setPath("/");
    mm.addSeparator();
}


void EditableSceneBody::onSceneModeChanged(const SceneWidgetEvent& event)
{
    impl->onSceneModeChanged(event);
}


void EditableSceneBody::Impl::onSceneModeChanged(const SceneWidgetEvent& event)
{
    isEditMode = event.sceneWidget()->isEditMode();

    if(isEditMode){
        if(outlinedLink){
            outlinedLink->showOutline(true);
        }
    } else {
        finishEditing();
        if(outlinedLink){
            outlinedLink->showOutline(false);
            outlinedLink = nullptr;
        }
        updateMarkersAndManipulators(false);
    }
}


bool EditableSceneBody::Impl::finishEditing()
{
    bool finished = false;
    
    isDragging = false;
    finished = true;

    if(dragMode == LINK_VIRTUAL_ELASTIC_STRING){
        finishVirtualElasticString();
    } else if(dragMode == LINK_FORCED_POSITION){
        finishForcedPosition();
    } else if(dragMode != DRAG_NONE){
        bodyItem->acceptKinematicStateEdit();
    } else {
        finished = false;
    }
    
    dragMode = DRAG_NONE;

    return finished;
}


bool EditableSceneBody::onUndoRequest()
{
    return impl->onUndoRequest();
}


bool EditableSceneBody::Impl::onUndoRequest()
{
    return bodyItem->undoKinematicState();
}


bool EditableSceneBody::onRedoRequest()
{
    return impl->onRedoRequest();
}


bool EditableSceneBody::Impl::onRedoRequest()
{
    return bodyItem->redoKinematicState();
}


void EditableSceneBody::Impl::onDraggerDragStarted()
{
    activeSimulatorItem = SimulatorItem::findActiveSimulatorItemFor(bodyItem);
    if(!activeSimulatorItem){
        initializeIK();
    }
}


void EditableSceneBody::Impl::onDraggerDragged()
{
    activeSimulatorItem = SimulatorItem::findActiveSimulatorItemFor(bodyItem);
    if(activeSimulatorItem){
        setForcedPosition(positionDragger->globalDraggingPosition());
    } else {
        Affine3 T = positionDragger->globalDraggingPosition();
        doIK(T);
    }
}


void EditableSceneBody::Impl::onDraggerDragFinished()
{
    activeSimulatorItem = SimulatorItem::findActiveSimulatorItemFor(bodyItem);
    if(activeSimulatorItem){
        finishForcedPosition();
    } else {
        Affine3 T = positionDragger->globalDraggingPosition();
        doIK(T);
    }
}


bool EditableSceneBody::Impl::initializeIK()
{
    if(!currentIK && bodyItem->pinDragIK()->numPinnedLinks() > 0){
        pinDragIK = bodyItem->pinDragIK();
        pinDragIK->setBaseLink(bodyItem->currentBaseLink());
        pinDragIK->setTargetLink(targetLink, kinematicsBar->isPositionDraggerEnabled());
        if(pinDragIK->initialize()){
            currentIK = pinDragIK;
        }
    }
    if(!currentIK){
        currentIK = bodyItem->getCurrentIK(targetLink);
    }
    if(auto jointPath = dynamic_pointer_cast<JointPath>(currentIK)){
        if(!jointPath->hasAnalyticalIK()){
            jointPath->setBestEffortIKmode(true);
        }
    }

    return currentIK? true: false;
}


void EditableSceneBody::Impl::startIK(const SceneWidgetEvent& event)
{
    Body* body = self->body();
    Link* baseLink = bodyItem->currentBaseLink();

    if(initializeIK()){
        if(kinematicsBar->isPositionDraggerEnabled()){
            attachPositionDragger(targetLink);
        }
        dragProjector.setInitialPosition(targetLink->position());
        dragProjector.setTranslationAlongViewPlane();
        if(dragProjector.startTranslation(event)){
            dragMode = LINK_IK_TRANSLATION;
        }
        fkTraverse.find(baseLink ? baseLink : body->rootLink(), true, true);
        if(kinematicsBar->isPenetrationBlockMode()){
            penetrationBlocker = bodyItem->createPenetrationBlocker(targetLink, true);
        } else {
            penetrationBlocker.reset();
        }
    }
}


void EditableSceneBody::Impl::dragIK(const SceneWidgetEvent& event)
{
    if(dragProjector.dragTranslation(event)){
        //Position T = dragProjector.initialPosition();
        Position T;
        T.translation() = dragProjector.position().translation();
        T.linear() = targetLink->R();
        if(penetrationBlocker){
            penetrationBlocker->adjust(T, T.translation() - targetLink->p());
        }
        doIK(T);
    }
}


void EditableSceneBody::Impl::doIK(const Position& position)
{
    if(currentIK){
        if(currentIK->calcInverseKinematics(position) || true /* Best effort */){
            bool fkDone = currentIK->calcRemainingPartForwardKinematicsForInverseKinematics();
            if(!fkDone){
                fkTraverse.calcForwardKinematics();
            }
            bodyItem->notifyKinematicStateChange();
        }
    }
}


void EditableSceneBody::Impl::startFK(const SceneWidgetEvent& event)
{
    dragProjector.setInitialPosition(targetLink->position());
    
    orgJointPosition = targetLink->q();
    
    if(targetLink->isRotationalJoint()){
        dragProjector.setRotationAxis(targetLink->R() * targetLink->a());
        if(dragProjector.startRotation(event)){
            dragMode = LINK_FK_ROTATION;
        }
        
    } else if(targetLink->isSlideJoint()){
        dragProjector.setTranslationAxis(targetLink->R() * targetLink->d());
        if(dragProjector.startTranslation(event)){
            dragMode = LINK_FK_TRANSLATION;
        }
    }
}


void EditableSceneBody::Impl::dragFKRotation(const SceneWidgetEvent& event)
{
    if(dragProjector.dragRotation(event)){
        targetLink->q() = orgJointPosition + dragProjector.rotationAngle();
        bodyItem->notifyKinematicStateChange(true);
    }
}


void EditableSceneBody::Impl::dragFKTranslation(const SceneWidgetEvent& event)
{
    if(dragProjector.dragTranslation(event)){
        targetLink->q() = orgJointPosition + dragProjector.translationAxis().dot(dragProjector.translation());
        bodyItem->notifyKinematicStateChange(true);
    }
}


void EditableSceneBody::Impl::startLinkOperationDuringSimulation(const SceneWidgetEvent& event)
{
    if(event.button() == Qt::LeftButton){
        if(targetLink->isBodyRoot() && (forcedPositionMode != NO_FORCED_POSITION)){
            startForcedPosition(event);
        } else {
            startVirtualElasticString(event);
        }
    }
}


void EditableSceneBody::Impl::setForcedPositionMode(int mode, bool on)
{
    if(on){
        forcedPositionMode = mode;
    } else {
        forcedPositionMode = NO_FORCED_POSITION;
        updateMarkersAndManipulators(isFocused);
    }
    finishForcedPosition();
}


void EditableSceneBody::Impl::startVirtualElasticString(const SceneWidgetEvent& event)
{
    const Vector3& point = event.point();
    dragProjector.setInitialTranslation(point);
    dragProjector.setTranslationAlongViewPlane();
    if(dragProjector.startTranslation(event)){
        pointedLinkLocalPoint = targetLink->T().inverse() * point;
        dragMode = LINK_VIRTUAL_ELASTIC_STRING;
        dragVirtualElasticString(event);
        markerGroup->addChildOnce(virtualElasticStringLine, true);
    }
}


void EditableSceneBody::Impl::dragVirtualElasticString(const SceneWidgetEvent& event)
{
    if(dragMode == LINK_VIRTUAL_ELASTIC_STRING){
        SimulatorItem* simulatorItem = activeSimulatorItem.lock();
        if(simulatorItem && dragProjector.dragTranslation(event)){
            Vector3 p = targetLink->T() * pointedLinkLocalPoint;
            Vector3 d = dragProjector.position().translation() - p;
            double k = 2.0;
            if(event.modifiers() & Qt::ShiftModifier){
                k *= 10.0;
                if(event.modifiers() & Qt::ControlModifier){
                    k *= 10.0;
                }
            }
            Vector3 end = p + k * self->boundingBox().boundingSphereRadius() * d;
            SgVertexArray& points = *virtualElasticStringLine->vertices();
            points[0] = p.cast<Vector3f::Scalar>();
            points[1] = (p + d).cast<Vector3f::Scalar>();
            virtualElasticStringLine->notifyUpdate();
            simulatorItem->setVirtualElasticString(bodyItem, targetLink, pointedLinkLocalPoint, end);
        }
    }
}


void EditableSceneBody::Impl::finishVirtualElasticString()
{
    if(SimulatorItem* simulatorItem = activeSimulatorItem.lock()){
        simulatorItem->clearVirtualElasticStrings();
    }
    markerGroup->removeChild(virtualElasticStringLine, true);
}


void EditableSceneBody::Impl::startForcedPosition(const SceneWidgetEvent& event)
{
    finishForcedPosition();
    updateMarkersAndManipulators(true);
    if(kinematicsBar->isPositionDraggerEnabled() && forcedPositionMode != NO_FORCED_POSITION){
        attachPositionDragger(targetLink);
    }
    dragProjector.setInitialPosition(targetLink->position());
    dragProjector.setTranslationAlongViewPlane();
    if(dragProjector.startTranslation(event)){
        dragMode = LINK_FORCED_POSITION;
    }
}


void EditableSceneBody::Impl::setForcedPosition(const Position& position)
{
    if(SimulatorItem* simulatorItem = activeSimulatorItem.lock()){
        simulatorItem->setForcedPosition(bodyItem, position);
    }
}
    
    
void EditableSceneBody::Impl::dragForcedPosition(const SceneWidgetEvent& event)
{
    if(dragProjector.dragTranslation(event)){
        Position T;
        T.translation() = dragProjector.position().translation();
        T.linear() = targetLink->R();
        setForcedPosition(T);
    }
}


void EditableSceneBody::Impl::finishForcedPosition()
{
    if(forcedPositionMode != KEEP_FORCED_POSITION){
        if(SimulatorItem* simulatorItem = activeSimulatorItem.lock()){
            simulatorItem->clearForcedPositions();
        }
    }
}


void EditableSceneBody::Impl::startZmpTranslation(const SceneWidgetEvent& event)
{
    dragProjector.setInitialTranslation(bodyItem->zmp());
    dragProjector.setTranslationPlaneNormal(Vector3::UnitZ());
    if(dragProjector.startTranslation(event)){
        dragMode = ZMP_TRANSLATION;
    }
}


void EditableSceneBody::Impl::dragZmpTranslation(const SceneWidgetEvent& event)
{
    if(dragProjector.dragTranslation(event)){
        Vector3 p = dragProjector.position().translation();
        p.z() = dragProjector.initialPosition().translation().z();
        bodyItem->setZmp(p);
        bodyItem->notifyKinematicStateChange(true);
    }
}


bool EditableSceneBody::Impl::storeProperties(Archive& archive)
{
    ListingPtr states = new Listing();

    for(auto& bodyItem : RootItem::instance()->descendantItems<BodyItem>()){
        EditableSceneBody* sceneBody = bodyItem->existingSceneBody();
        if(sceneBody){
            ValueNodePtr id = archive.getItemId(bodyItem);
            if(id){
                EditableSceneBody::Impl* impl = sceneBody->impl;
                MappingPtr state = new Mapping();
                state->insert("bodyItem", id);
                state->write("showCenterOfMass", impl->isCmVisible);
                state->write("showPpcom", impl->isPpcomVisible);
                state->write("showZmp", impl->isZmpVisible);
                states->append(state);
            }
        }
    }
    if(!states->empty()){
        archive.insert("editableSceneBodies", states);
    }

    return true;
}
    
    
void EditableSceneBody::Impl::restoreProperties(const Archive& archive)
{
    archive.addPostProcess(
        [&archive](){ restoreSceneBodyProperties(archive); },
        1);
}


void EditableSceneBody::Impl::restoreSceneBodyProperties(const Archive& archive)
{
    Listing& states = *archive.findListing("editableSceneBodies");
    if(states.isValid()){
        for(int i=0; i < states.size(); ++i){
            Mapping* state = states[i].toMapping();
            BodyItem* bodyItem = archive.findItem<BodyItem>(state->find("bodyItem"));
            if(bodyItem){
                EditableSceneBody::Impl* impl = bodyItem->sceneBody()->impl;
                impl->showCenterOfMass(state->get("showCenterOfMass", impl->isCmVisible));
                impl->showPpcom(state->get("showPpcom", impl->isPpcomVisible));
                impl->showZmp(state->get("showZmp", impl->isZmpVisible));
            }
        }
    }
}


void EditableSceneBody::initializeClass(ExtensionManager* ext)
{
    MenuManager& mm = ext->menuManager().setPath("/Options/Scene View");
    linkVisibilityCheck = mm.addCheckItem(_("Show selected links only"));

    ext->setProjectArchiver(
        "EditableSceneBody",
        EditableSceneBody::Impl::storeProperties,
        EditableSceneBody::Impl::restoreProperties);
}
