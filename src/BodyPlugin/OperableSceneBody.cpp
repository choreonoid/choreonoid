#include "OperableSceneBody.h"
#include "BodyItem.h"
#include "BodyItemKinematicsKit.h"
#include "BodySelectionManager.h"
#include "KinematicsBar.h"
#include "SimulatorItem.h"
#include <cnoid/JointPath>
#include <cnoid/LinkedJointHandler>
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
#include <cnoid/stdx/clamp>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

enum LinkOperationType { None, FK, IK, SimInterference };

}

namespace cnoid {

class OperableSceneLink::Impl
{
public:
    OperableSceneLink* self;
    SgUpdate& update;
    SgPolygonDrawStylePtr highlightStyle;
    BoundingBoxMarkerPtr bbMarker;
    CrossMarkerPtr cmMarker;
    bool isOriginShown;
    bool isCenterOfMassShown;
    bool isPointed;
    bool isColliding;

    Impl(OperableSceneBody* sceneBody, OperableSceneLink* self);
    OperableSceneBody* operableSceneBody();
    double calcMarkerRadius() const;
    void showOrigin(bool on);
    void showCenterOfMass(bool on);
};

class OperableSceneBody::Impl
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    OperableSceneBody* self;
    // This must not be a smart pointer to avoid a cycline reference.
    // OperableSceneBody is owned by the corresponding BodyItem object.
    BodyItem* bodyItem;
    LinkedJointHandlerPtr linkedJointHandler;

    SgUpdate update;

    ScopedConnectionSet connections;
    ScopedConnection connectionToSigCollisionsUpdated;
    vector<bool> collisionLinkBitSet;
    ScopedConnection connectionToSigLinkSelectionChanged;

    enum PointedType { PT_NONE, PT_SCENE_LINK, PT_ZMP };
    OperableSceneLink* pointedSceneLink;
    OperableSceneLink* highlightedLink;

    Link* targetLink;
    double orgJointPosition;
        
    LinkTraverse fkTraverse;
    shared_ptr<InverseKinematics> currentIK;
    shared_ptr<PinDragIK> pinDragIK;
    shared_ptr<PenetrationBlocker> penetrationBlocker;
    PositionDraggerPtr positionDragger;
    OperableSceneLink* sceneLinkForPositionDragger;
    ScopedConnection kinematicsKitConnection;

    bool isEditMode;
    bool isFocused;
    bool isSelected;
    bool isHighlightingEnabled;
    bool isVisibleLinkSelectionMode;

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
    bool dragged;

    PositionDraggerPtr linkOriginMarker;
    vector<bool> linkOriginMarkerVisibilities;
    SgHighlightPtr highlight;
    SgGroupPtr markerGroup;
    CrossMarkerPtr cmMarker;
    CrossMarkerPtr cmProjectionMarker;
    SphereMarkerPtr zmpMarker;
    Vector3 orgZmpPos;
    double bodyMarkerRadius;
    bool isCmVisible;
    bool isCmProjectionVisible;
    bool isZmpVisible;

    SgLineSetPtr virtualElasticStringLine;
    weak_ref_ptr<SimulatorItem> activeSimulatorItem;
    Vector3 pointedLinkLocalPoint;
    enum { NO_FORCED_POSITION, MOVE_FORCED_POSITION, KEEP_FORCED_POSITION };
    int forcedPositionMode;

    Impl(OperableSceneBody* self, BodyItem* bodyItem);
    void initialize();
        
    OperableSceneLink* operableSceneLink(int index){
        return static_cast<OperableSceneLink*>(self->sceneLink(index));
    }

    void onSceneGraphConnection(bool on);
    void onBodyItemUpdated();
    void updateSceneModel();
    void onSelectionChanged(bool on);
    void onKinematicStateChanged();
    void onCollisionsUpdated();
    void onCollisionLinkHighlightModeChanged();
    void changeCollisionLinkHighlightMode(bool on);
    void updateVisibleLinkSelectionMode(bool isActive);
    void onLinkOriginsCheckToggled(bool on);
    void onLinkCmsCheckToggled(bool on);
    void enableHighlight(bool on);
    void calcBodyMarkerRadius();
    double calcLinkMarkerRadius(SceneLink* sceneLink) const;
    void ensureCmMarker();
    void ensureCmProjectionMarker();
    LeggedBodyHelper* checkLeggedBody();
    bool ensureZmpMarker();
    void showCenterOfMass(bool on);
    void showCmProjection(bool on);
    void showZmp(bool on);
    void makeLinkFree(OperableSceneLink* sceneLink);
    void setBaseLink(OperableSceneLink* sceneLink);
    void toggleBaseLink(OperableSceneLink* sceneLink);
    void togglePin(OperableSceneLink* sceneLink, bool toggleTranslation, bool toggleRotation);
    void makeLinkAttitudeLevel(OperableSceneLink* sceneLink);
        
    PointedType findPointedObject(const SgNodePath& path);
    int checkLinkOperationType(SceneLink* sceneLink, bool doUpdateIK);
    int checkLinkKinematicsType(Link* link, bool doUpdateIK);
    void updateMarkersAndManipulators(bool on);
    void createPositionDragger();
    void attachPositionDragger(Link* link);
    void detachPositionDragger();
    void adjustPositionDraggerSize(Link* link, OperableSceneLink* sceneLink);

    bool onKeyPressEvent(SceneWidgetEvent* event);
    bool onKeyReleaseEvent(SceneWidgetEvent* event);
    bool onButtonPressEvent(SceneWidgetEvent* event);
    bool onButtonReleaseEvent(SceneWidgetEvent* event);
    bool onPointerMoveEvent(SceneWidgetEvent* event);
    void onPointerLeaveEvent(SceneWidgetEvent* event);
    bool onScrollEvent(SceneWidgetEvent* event);
    bool onContextMenuRequest(SceneWidgetEvent* event);
    void onSceneModeChanged(SceneWidgetEvent* event);
    void onDraggerDragStarted();
    void onDraggerDragged();
    void onDraggerDragFinished();

    bool initializeIK();
    void startIK(SceneWidgetEvent* event);
    void dragIK(SceneWidgetEvent* event);
    void doIK(const Isometry3& position);
    void startFK(SceneWidgetEvent* event);
    void dragFKRotation(SceneWidgetEvent* event);
    void dragFKTranslation(SceneWidgetEvent* event);
    void startLinkOperationDuringSimulation(SceneWidgetEvent* event);
    void setForcedPositionMode(int mode, bool on);
    void startVirtualElasticString(SceneWidgetEvent* event);
    void dragVirtualElasticString(SceneWidgetEvent* event);
    void finishVirtualElasticString();
    void startForcedPosition(SceneWidgetEvent* event);
    void setForcedPosition(const Isometry3& position);
    void dragForcedPosition(SceneWidgetEvent* event);
    void finishForcedPosition();
    void startZmpTranslation(SceneWidgetEvent* event);
    void dragZmpTranslation(SceneWidgetEvent* event);
    bool finishEditing();
    
    static bool storeProperties(Archive& archive);
    static void restoreProperties(const Archive& archive);
    static void restoreSceneBodyProperties(const Archive& archive);
};

}


OperableSceneLink::OperableSceneLink(OperableSceneBody* sceneBody, Link* link)
    : SceneLink(sceneBody, link)
{
    impl = new Impl(sceneBody, this);
}


OperableSceneLink::Impl::Impl(OperableSceneBody* sceneBody, OperableSceneLink* self)
    : self(self),
      update(sceneBody->impl->update)
{
    isOriginShown = false;
    isCenterOfMassShown = false;
    isPointed = false;
    isColliding = false;
}


OperableSceneLink::~OperableSceneLink()
{
    delete impl;
}


OperableSceneBody* OperableSceneLink::operableSceneBody()
{
    return static_cast<OperableSceneBody*>(sceneBody());
}


OperableSceneBody* OperableSceneLink::Impl::operableSceneBody()
{
    return static_cast<OperableSceneBody*>(self->sceneBody());
}


const OperableSceneBody* OperableSceneLink::operableSceneBody() const
{
    return static_cast<const OperableSceneBody*>(sceneBody());
}


void OperableSceneLink::setVisible(bool on)
{
    SceneLink::setVisible(on);

    bool updated = false;

    if(impl->isOriginShown){
        auto sceneBodyImpl = operableSceneBody()->impl;
        if(on){
            addChildOnce(sceneBodyImpl->linkOriginMarker);
        } else {
            removeChild(sceneBodyImpl->linkOriginMarker);
        }
        updated = true;
    }
    if(impl->isCenterOfMassShown){
        if(on){
            addChildOnce(impl->cmMarker);
        } else {
            removeChild(impl->cmMarker);
        }
        updated = true;
    }
    if(updated){
        notifyUpdate(impl->update.withAction(on ? SgUpdate::Added : SgUpdate::Removed));
    }
}


double OperableSceneLink::Impl::calcMarkerRadius() const
{
    if(auto shape = self->visualShape()){
        const BoundingBox& bb = shape->boundingBox();
        if(bb.empty()){
            return 1.0; // Is this OK?
        }
        double V = ((bb.max().x() - bb.min().x()) * (bb.max().y() - bb.min().y()) * (bb.max().z() - bb.min().z()));
        return pow(V, 1.0 / 3.0) * 0.6;
    }
    return 1.0;
}


void OperableSceneLink::showOrigin(bool on)
{
    impl->showOrigin(on);
}


void OperableSceneLink::Impl::showOrigin(bool on)
{
    if(on != isOriginShown){

        auto sceneBody = operableSceneBody();
        auto& originMarker = sceneBody->impl->linkOriginMarker;
        auto& visibilities = sceneBody->impl->linkOriginMarkerVisibilities;
        int linkIndex = self->link()->index();

        if(on){
            if(!originMarker){
                originMarker = new PositionDragger(
                    PositionDragger::TranslationAxes, PositionDragger::PositiveOnlyHandle);
                originMarker->setOverlayMode(true);
                originMarker->setPixelSize(48, 2);
                originMarker->setDisplayMode(PositionDragger::DisplayInEditMode);
                originMarker->setTransparency(0.0f);
                originMarker->setDragEnabled(false);
            }
            if(self->isVisible()){
                self->addChildOnce(originMarker, update);
            }
            
            if(linkIndex >= visibilities.size()){
                visibilities.resize(linkIndex + 1);
            }
            visibilities[linkIndex] = true;
            
        } else {
            if(self->isVisible()){
                if(originMarker && originMarker->hasParents()){
                    self->removeChild(originMarker, update);
                }
            }
            visibilities[linkIndex] = false;
        }
        isOriginShown = on;
    }
}


bool OperableSceneLink::isOriginShown() const
{
    return impl->isOriginShown;
}


void OperableSceneLink::showCenterOfMass(bool on)
{
    impl->showCenterOfMass(on);
}


void OperableSceneLink::Impl::showCenterOfMass(bool on)
{
    if(on != isCenterOfMassShown){
        if(on){
            if(!cmMarker){
                auto radius = calcMarkerRadius();
                cmMarker = new CrossMarker(radius, Vector3f(0.0f, 1.0f, 0.0f), 2.0);
                cmMarker->setName("CenterOfMass");
            }
            cmMarker->setTranslation(self->link()->centerOfMass());
            if(self->isVisible()){
                self->addChildOnce(cmMarker, update);
            }
            
        } else {
            if(self->isVisible()){
                if(cmMarker && cmMarker->hasParents()){
                    self->removeChild(cmMarker, update);
                }
            }
        }
        isCenterOfMassShown = on;
    }
}


bool OperableSceneLink::isCenterOfMassShown() const
{
    return impl->isCenterOfMassShown;
}


void OperableSceneLink::enableHighlight(bool on)
{
    if(!visualShape()){
        return;
    }
    auto& highlightStyle = impl->highlightStyle;
    if(on){
        if(!highlightStyle){
            highlightStyle = new SgPolygonDrawStyle;
            highlightStyle->setPolygonElements(SgPolygonDrawStyle::Face | SgPolygonDrawStyle::Edge);
            highlightStyle->setEdgeColor(Vector4f(1.0f, 1.0f, 0.0f, 0.75f));
            highlightStyle->setEdgeWidth(0.7f);
        }
        if(!highlightStyle->hasParents()){
            insertEffectGroup(highlightStyle, impl->update);
        }
    } else {
        if(highlightStyle && highlightStyle->hasParents()){
            removeEffectGroup(highlightStyle, impl->update);
        }
    }
}


void OperableSceneLink::showMarker(const Vector3f& color, float transparency)
{
    if(impl->bbMarker){
        removeChild(impl->bbMarker, impl->update);
    }
    if(visualShape()){
        impl->bbMarker = new BoundingBoxMarker(visualShape()->boundingBox(), color, transparency);
        addChildOnce(impl->bbMarker, impl->update);
    }
}


void OperableSceneLink::hideMarker()
{
    if(impl->bbMarker){
        removeChild(impl->bbMarker, impl->update);
        impl->bbMarker = nullptr;
    }
}


void OperableSceneLink::setColliding(bool on)
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


OperableSceneBody::OperableSceneBody(BodyItem* bodyItem)
{
    setAttribute(Operable);
    impl = new Impl(this, bodyItem);
    impl->initialize();
}


OperableSceneBody::Impl::Impl(OperableSceneBody* self, BodyItem* bodyItem)
    : self(self),
      bodyItem(bodyItem),
      kinematicsBar(KinematicsBar::instance())
{

}


void OperableSceneBody::Impl::initialize()
{
    pointedSceneLink = nullptr;
    highlightedLink = nullptr;
    targetLink = nullptr;
    sceneLinkForPositionDragger = nullptr;

    isEditMode = false;
    isFocused = false;
    isSelected = false;
    isHighlightingEnabled = false;
    isVisibleLinkSelectionMode = false;

    self->setBody(bodyItem->body(), [this](Link* link){ return new OperableSceneLink(self, link); });

    dragMode = DRAG_NONE;
    isDragging = false;
    dragged = false;

    markerGroup = new SgGroup;
    markerGroup->setName("Marker");
    self->addChild(markerGroup);

    bodyMarkerRadius = -1.0;
    isCmVisible = false;
    isCmProjectionVisible = false;
    isZmpVisible = false;

    forcedPositionMode = NO_FORCED_POSITION;
    virtualElasticStringLine = new SgLineSet;
    virtualElasticStringLine->getOrCreateVertices()->resize(2);
    virtualElasticStringLine->addLine(0, 1);

    self->sigGraphConnection().connect([this](bool on){ onSceneGraphConnection(on);});
}


BodyItem* OperableSceneBody::bodyItem()
{
    return impl->bodyItem;
}


void OperableSceneBody::Impl::onSceneGraphConnection(bool on)
{
    connections.disconnect();

    if(on){

        connections.add(
            bodyItem->sigSelectionChanged().connect(
                [this](bool on){ onSelectionChanged(on); }));

        onSelectionChanged(bodyItem->isSelected()); 

        connections.add(
            bodyItem->sigUpdated().connect(
                [this](){ onBodyItemUpdated(); }));

        connections.add(
            bodyItem->sigContinuousKinematicUpdateStateChanged().connect(
                [this](bool){ onBodyItemUpdated(); }));

        connections.add(
            bodyItem->sigKinematicStateChanged().connect(
                [this](){ onKinematicStateChanged(); }));
            
        onKinematicStateChanged();

        connections.add(
            kinematicsBar->sigCollisionVisualizationChanged().connect(
                [this](){ onCollisionLinkHighlightModeChanged(); }));
        
        onCollisionLinkHighlightModeChanged();
    }

    updateVisibleLinkSelectionMode(on);
}


void OperableSceneBody::Impl::onBodyItemUpdated()
{
    bool isUserInputBlocked = bodyItem->isDoingContinuousKinematicUpdate() || bodyItem->isLocationLocked();
    if(isUserInputBlocked){
        if(sceneLinkForPositionDragger){
            detachPositionDragger();
        }
    } else if(isFocused){
        updateMarkersAndManipulators(true);
    }

    updateVisibleLinkSelectionMode(true);
}


void OperableSceneBody::updateSceneModel()
{
    impl->updateSceneModel();
}


void OperableSceneBody::Impl::updateSceneModel()
{
    pointedSceneLink = nullptr;
    targetLink = nullptr;
    if(highlightedLink){
        highlightedLink->enableHighlight(false);
        highlightedLink = nullptr;
    }
    dragMode = DRAG_NONE;
    isDragging = false;
    dragged = false;

    self->SceneBody::updateSceneModel();

    // Restore the visibilities of link origin markers
    int n = std::min(self->numSceneLinks(), (int)linkOriginMarkerVisibilities.size());
    for(int i=0; i < n; ++i){
        if(linkOriginMarkerVisibilities[i]){
            self->operableSceneLink(i)->showOrigin(true);
        }
    }
    linkOriginMarkerVisibilities.resize(n);
}


OperableSceneLink* OperableSceneBody::operableSceneLink(int index)
{
    return static_cast<OperableSceneLink*>(sceneLink(index));
}


void OperableSceneBody::Impl::onSelectionChanged(bool on)
{
    isSelected = on;
    enableHighlight(isHighlightingEnabled && isEditMode && isSelected);
}


void OperableSceneBody::Impl::onKinematicStateChanged()
{
    if(isCmVisible){
        cmMarker->setTranslation(bodyItem->centerOfMass());
    }
    if(isCmProjectionVisible){
    	Vector3 com = bodyItem->centerOfMass();
    	com(2) = 0.0;
    	cmProjectionMarker->setTranslation(com);
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

    self->updateLinkPositions(update.withAction(SgUpdate::Modified));
}


void OperableSceneBody::Impl::onCollisionsUpdated()
{
    if(bodyItem->collisionLinkBitSet() != collisionLinkBitSet){
        collisionLinkBitSet = bodyItem->collisionLinkBitSet();
        const int n = self->numSceneLinks();
        for(int i=0; i < n; ++i){
            operableSceneLink(i)->setColliding(collisionLinkBitSet[i]);
        }
        self->notifyUpdate(update.withAction(SgUpdate::Modified));
    }
}


void OperableSceneBody::Impl::onCollisionLinkHighlightModeChanged()
{
    changeCollisionLinkHighlightMode(kinematicsBar->isCollisionLinkHighlihtMode());
}


void OperableSceneBody::Impl::changeCollisionLinkHighlightMode(bool on)
{
    if(!connectionToSigCollisionsUpdated.connected() && on){
        connectionToSigCollisionsUpdated =
            bodyItem->sigCollisionsUpdated().connect(
                [this](){ onCollisionsUpdated(); });
        onCollisionsUpdated();

    } else if(connectionToSigCollisionsUpdated.connected() && !on){
        connectionToSigCollisionsUpdated.disconnect();
        const int n = self->numSceneLinks();
        for(int i=0; i < n; ++i){
            operableSceneLink(i)->setColliding(false);
        }
        self->notifyUpdate(update.withAction(SgUpdate::Modified));
    }
}


OperableSceneBody::~OperableSceneBody()
{
    delete impl;
}


void OperableSceneBody::setLinkVisibilities(const std::vector<bool>& visibilities)
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
    notifyUpdate(impl->update.withAction(SgUpdate::Modified));
}


void OperableSceneBody::Impl::updateVisibleLinkSelectionMode(bool isActive)
{
    bool newMode;
    
    if(isActive){
        newMode = bodyItem->isVisibleLinkSelectionMode();
    } else {
        newMode = false;
    }

    if(newMode != isVisibleLinkSelectionMode){
        
        isVisibleLinkSelectionMode = newMode;

        if(isVisibleLinkSelectionMode){
            auto bsm = BodySelectionManager::instance();
            connectionToSigLinkSelectionChanged.reset(
                bsm->sigLinkSelectionChanged(bodyItem).connect(
                    [this](const std::vector<bool>& selection){
                        self->setLinkVisibilities(selection);
                    }));
            self->setLinkVisibilities(bsm->linkSelection(bodyItem));

        } else {
            connectionToSigLinkSelectionChanged.disconnect();
            if(isActive){
                self->setLinkVisibilities(vector<bool>(self->numSceneLinks(), true));
            }
        }
    }
}


void OperableSceneBody::Impl::onLinkOriginsCheckToggled(bool on)
{
    for(int i=0; i < self->numSceneLinks(); ++i){
        self->operableSceneLink(i)->showOrigin(on);
    }
}


void OperableSceneBody::Impl::onLinkCmsCheckToggled(bool on)
{
    for(int i=0; i < self->numSceneLinks(); ++i){
        self->operableSceneLink(i)->showCenterOfMass(on);
    }
}


void OperableSceneBody::Impl::enableHighlight(bool on)
{
    if(on){
        bool doUpdate = false;
        if(!highlight){
            highlight = new SgBoundingBox;
            highlight->setColor(Vector3f(1.0f, 1.0f, 0.0f));
            highlight->setLineWidth(2.0f);
            doUpdate = true;
        } else {
            doUpdate = !highlight->hasParents();
        }
        if(doUpdate){
            self->insertEffectGroup(highlight, update.withAction(SgUpdate::GeometryModified));

            // The following code cannot support the case where
            // the number of links is changed by model update.
            /*
            if(self->numSceneLinks() == 1){
                self->sceneLink(0)->insertEffectGroup(highlight, update);
            } else{
                self->insertEffectGroup(highlight, update);
            }
            */
        }
    } else {
        if(highlight && highlight->hasParents()){
            self->removeEffectGroup(highlight, update.withAction(SgUpdate::GeometryModified));

            // The following code cannot support the case where
            // the number of links is changed by model update.
            /*
            if(self->numSceneLinks() == 1){
                self->sceneLink(0)->removeEffectGroup(highlight, update);
            } else {
                self->removeEffectGroup(highlight, update);
            }
            */
        }
    }
}


void OperableSceneBody::Impl::calcBodyMarkerRadius()
{
    bodyMarkerRadius = 0.0;
    const int n = self->numSceneLinks();
    for(int i=0; i < n; ++i){
        SceneLink* sLink = self->sceneLink(i);
        BoundingBox bb = sLink->boundingBox();
        double radius0 = bb.size().norm() / 2.0;
        if(radius0 > bodyMarkerRadius){
            bodyMarkerRadius = radius0;
        }
    }
}


void OperableSceneBody::Impl::ensureCmMarker()
{
    if(!cmMarker){
        if(bodyMarkerRadius < 0.0){
            calcBodyMarkerRadius();
        }
        cmMarker = new CrossMarker(bodyMarkerRadius, Vector3f(0.0f, 1.0f, 0.0f), 2.0);
        cmMarker->setName("CenterOfMass");
    }
}


void OperableSceneBody::Impl::ensureCmProjectionMarker()
{
    if(!cmProjectionMarker){
        if(bodyMarkerRadius < 0.0){
            calcBodyMarkerRadius();
        }
        cmProjectionMarker = new CrossMarker(bodyMarkerRadius, Vector3f(1.0f, 0.5f, 0.0f), 2.0);
        cmProjectionMarker->setName("CmProjection");
    }
}


LeggedBodyHelper* OperableSceneBody::Impl::checkLeggedBody()
{
    auto legged = getLeggedBodyHelper(self->body());
    if(!legged->isValid() || legged->numFeet() == 0){
        legged = nullptr;
    }
    return legged;
}


bool OperableSceneBody::Impl::ensureZmpMarker()
{
    if(!zmpMarker){
        if(auto legged = checkLeggedBody()){
            Link* footLink = legged->footLink(0);
            auto sceneLink = self->operableSceneLink(footLink->index());
            double radius = sceneLink->impl->calcMarkerRadius();
            zmpMarker = new SphereMarker(radius, Vector3f(0.0f, 1.0f, 0.0f), 0.3f);
            zmpMarker->addChild(new CrossMarker(radius * 2.5, Vector3f(0.0f, 1.0f, 0.0f), 2.0f));
            zmpMarker->setName("ZMP");
        }
    }
    return (zmpMarker != nullptr);
}


void OperableSceneBody::Impl::showCenterOfMass(bool on)
{
    isCmVisible = on;
    if(on){
        ensureCmMarker();
        cmMarker->setTranslation(bodyItem->centerOfMass());
        markerGroup->addChildOnce(cmMarker, update);
    } else {
        if(cmMarker){
            markerGroup->removeChild(cmMarker, update);
            cmMarker.reset();
            bodyMarkerRadius = -1.0;
        }
    }
}


void OperableSceneBody::Impl::showCmProjection(bool on)
{
    isCmProjectionVisible = on;
    if(on){
        ensureCmProjectionMarker();
        Vector3 com = bodyItem->centerOfMass();
        com(2) = 0.0;
        cmProjectionMarker->setTranslation(com);
        markerGroup->addChildOnce(cmProjectionMarker, update);
    } else {
        if(cmProjectionMarker){
            markerGroup->removeChild(cmProjectionMarker, update);
            cmProjectionMarker.reset();
            bodyMarkerRadius = -1.0;
        }
    }
}


void OperableSceneBody::Impl::showZmp(bool on)
{
    if(on){
        if(ensureZmpMarker()){
            zmpMarker->setTranslation(bodyItem->zmp());
            markerGroup->addChildOnce(zmpMarker, update);
            isZmpVisible = true;
        }
    } else {
        if(zmpMarker){
            markerGroup->removeChild(zmpMarker, update);
            zmpMarker.reset();
        }
        isZmpVisible = false;
    }
}


void OperableSceneBody::Impl::makeLinkFree(OperableSceneLink* sceneLink)
{
    if(bodyItem->currentBaseLink() == sceneLink->link()){
        bodyItem->setCurrentBaseLink(nullptr);
    }
    if(auto pin = bodyItem->checkPinDragIK()){
        pin->setPin(sceneLink->link(), PinDragIK::NO_AXES);
    }
    bodyItem->notifyUpdate();
}


void OperableSceneBody::Impl::setBaseLink(OperableSceneLink* sceneLink)
{
    bodyItem->setCurrentBaseLink(sceneLink->link());
    bodyItem->notifyUpdate();
}
    

void OperableSceneBody::Impl::toggleBaseLink(OperableSceneLink* sceneLink)
{
    Link* baseLink = bodyItem->currentBaseLink();
    if(sceneLink->link() != baseLink){
        bodyItem->setCurrentBaseLink(sceneLink->link());
    } else {
        bodyItem->setCurrentBaseLink(nullptr);
    }
    bodyItem->notifyUpdate();
}


void OperableSceneBody::Impl::togglePin(OperableSceneLink* sceneLink, bool toggleTranslation, bool toggleRotation)
{
    auto pin = bodyItem->getOrCreatePinDragIK();
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


void OperableSceneBody::Impl::makeLinkAttitudeLevel(OperableSceneLink* sceneLink)
{
    Link* link = sceneLink->link();
    auto ik = bodyItem->getCurrentIK(link);
    if(ik){
        const Isometry3& T = link->T();
        const double theta = acos(T(2, 2));
        const Vector3 z(T(0,2), T(1, 2), T(2, 2));
        const Vector3 axis = z.cross(Vector3::UnitZ()).normalized();
        const Matrix3 R2 = AngleAxisd(theta, axis) * T.linear();
        Isometry3 T2;
        T2.linear() = R2;
        T2.translation() = link->p();
        
        if(ik->calcInverseKinematics(T2)){
            if(!ik->calcRemainingPartForwardKinematicsForInverseKinematics()){
                bodyItem->body()->calcForwardKinematics();
            }
            bodyItem->notifyKinematicStateUpdate();
        }
    }
}


OperableSceneBody::Impl::PointedType OperableSceneBody::Impl::findPointedObject(const SgNodePath& path)
{
    PointedType pointedType = PT_NONE;
    pointedSceneLink = nullptr;
    for(size_t i = path.size() - 1; i >= 1; --i){
        pointedSceneLink = dynamic_cast<OperableSceneLink*>(path[i].get());
        if(pointedSceneLink){
            pointedType = PT_SCENE_LINK;
            break;
        }
        if(auto marker = dynamic_cast<SphereMarker*>(path[i].get())){
            if(marker == zmpMarker){
                pointedType = PT_ZMP;
                break;
            }
        }
    }
    return pointedType;
}


int OperableSceneBody::Impl::checkLinkOperationType(SceneLink* sceneLink, bool doUpdateIK)
{
    if(doUpdateIK){
        currentIK.reset();
    }
    
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
            if(checkLinkKinematicsType(link, doUpdateIK) != LinkOperationType::None){
                type = LinkOperationType::SimInterference;
            }
        }
    } else {
        type = checkLinkKinematicsType(link, doUpdateIK);
    }

    return type;
}


int OperableSceneBody::Impl::checkLinkKinematicsType(Link* link, bool doUpdateIK)
{
    // Check if the link is operable considering the ancestor bodies
    BodyItem* bodyItemChain = bodyItem;
    Link* linkChain = link;
    while(true){
        if(!bodyItemChain->isAttachedToParentBody()){
            if(bodyItemChain->isLocationLocked() && linkChain->isFixedToRoot()){
                return LinkOperationType::None;
            }
            break;
        }
        if(!linkChain->isFixedToRoot()){
            break;
        }
        bodyItemChain = bodyItemChain->parentBodyItem();
        linkChain = linkChain->body()->parentBodyLink();
    }

    int mode = kinematicsBar->mode();
    int type = LinkOperationType::None;

    if(mode == KinematicsBar::PresetKinematics){
        auto ik = bodyItem->findPresetIK(link);
        if(ik && kinematicsBar->isInverseKinematicsEnabled()){
            type = LinkOperationType::IK;
        } else if(link->isFixedToRoot()){
            type = LinkOperationType::IK;
        } else if(kinematicsBar->isForwardKinematicsEnabled()){
            type = LinkOperationType::FK;
        }
        if(doUpdateIK){
            currentIK = ik;
        }
    } else if(mode == KinematicsBar::ForwardKinematics){
        auto baseLink = bodyItem->currentBaseLink();
        if(link->isRoot()){
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
        type = LinkOperationType::IK;
    }
    
    return type;
}


void OperableSceneBody::Impl::updateMarkersAndManipulators(bool on)
{
    Link* baseLink = bodyItem->currentBaseLink();
    auto pin = bodyItem->checkPinDragIK();
    detachPositionDragger();

    const int n = self->numSceneLinks();
    for(int i=0; i < n; ++i){
        OperableSceneLink* sceneLink = operableSceneLink(i);
        sceneLink->hideMarker();

        if(on && isEditMode && !activeSimulatorItem){
            Link* link = sceneLink->link();
            if(link == baseLink){
                sceneLink->showMarker(Vector3f(1.0f, 0.1f, 0.1f), 0.4f);
            } else if(pin){
                int pinAxes = pin->pinAxes(link);
                if(pinAxes & (PinDragIK::TRANSFORM_6D)){
                    sceneLink->showMarker(Vector3f(1.0f, 1.0f, 0.1f), 0.4f);
                }
            }
        }
    }

    // The following connection is only necessary when the position dragger is shown
    kinematicsKitConnection.disconnect();
}


void OperableSceneBody::Impl::createPositionDragger()
{
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
    positionDragger->sigDragStarted().connect([this](){ onDraggerDragStarted(); });
    positionDragger->sigPositionDragged().connect([this](){ onDraggerDragged(); });
    positionDragger->sigDragFinished().connect([this](){ onDraggerDragFinished(); });
}

    
void OperableSceneBody::Impl::attachPositionDragger(Link* link)
{
    if(!positionDragger){
        createPositionDragger();
    }
    
    BodyItemKinematicsKit* kinematicsKit = nullptr;
    if(link->isRoot() && bodyItem->isAttachedToParentBody()){
        auto parentBodyLink = bodyItem->body()->parentBodyLink();
        if(!parentBodyLink->isRoot()){
            auto parentBodyItem = bodyItem->parentBodyItem();
            kinematicsKit = parentBodyItem->getCurrentKinematicsKit(parentBodyLink);
            if(kinematicsKit){
                positionDragger->setPosition(
                    link->Tb().inverse(Eigen::Isometry) * kinematicsKit->currentOffsetFrame()->T());
            }
        }
    }
    if(!kinematicsKit){
        kinematicsKit = bodyItem->getCurrentKinematicsKit(link);
        if(kinematicsKit){
            positionDragger->setPosition(kinematicsKit->currentOffsetFrame()->T());
        } else {
            positionDragger->setPosition(Isometry3::Identity());
        }
    }

    positionDragger->setOffset(positionDragger->T());

    // Even if the connection to sigFrameUpdate is remade, it must
    // first be disconnected to avoid the infinite loop to call the newly
    // connected slot when this function is called by the signal.
    kinematicsKitConnection.disconnect();
    if(kinematicsKit){
        kinematicsKitConnection =
            kinematicsKit->sigFrameSetChanged().connect(
                [this, link](){ attachPositionDragger(link); });
    }
    
    auto sceneLink = operableSceneLink(link->index());

    if(!positionDragger->isScreenFixedSizeMode()){
        double size = link->info("gui_handle_size", -1.0);
        if(size > 0.0){
            positionDragger->setHandleSize(size);
        } else {
            adjustPositionDraggerSize(link, sceneLink);
        }
    }
    
    positionDragger->notifyUpdate(update.withAction(SgUpdate::Modified));
    sceneLink->addChildOnce(positionDragger);
    sceneLinkForPositionDragger = sceneLink;
}


void OperableSceneBody::Impl::detachPositionDragger()
{
    if(sceneLinkForPositionDragger){
        sceneLinkForPositionDragger->removeChild(positionDragger, update);
        sceneLinkForPositionDragger = nullptr;
    }
}


/*
   To automatically adjust the size of the position dragger to a reasonable size,
   the sizes of the neighboring links of the target link are also taken into account
   to determine the size.
*/
void OperableSceneBody::Impl::adjustPositionDraggerSize(Link* link, OperableSceneLink* sceneLink)
{
    BoundingBox bb;
    
    if(auto shape = sceneLink->visualShape()){
        bb.expandBy(shape->untransformedBoundingBox());
    }
    constexpr double s = 0.5;
    if(auto parent = link->parent()){
        if(auto parentSceneLink = operableSceneLink(parent->index())){
            auto parentBBox = parentSceneLink->untransformedBoundingBox();
            parentBBox.scale(s);
            bb.expandBy(parentBBox);
        }
    }
    for(auto child = link->child(); child; child = child->sibling()){
        if(auto childSceneLink = operableSceneLink(child->index())){
            auto childBBox = childSceneLink->untransformedBoundingBox();
            childBBox.scale(s);
            bb.expandBy(childBBox);
        }
    }

    positionDragger->adjustSize(bb);
}


void OperableSceneBody::onSceneModeChanged(SceneWidgetEvent* event)
{
    impl->onSceneModeChanged(event);
}


void OperableSceneBody::Impl::onSceneModeChanged(SceneWidgetEvent* event)
{
    bool wasEditMode = isEditMode;
    isEditMode = event->sceneWidget()->isEditMode();

    if(isEditMode != wasEditMode){
        if(isEditMode){
            if(highlightedLink){
                highlightedLink->enableHighlight(true);
            }
        } else {
            finishEditing();
            if(highlightedLink){
                highlightedLink->enableHighlight(false);
                highlightedLink = nullptr;
            }
            updateMarkersAndManipulators(false);
        }
    }

    isHighlightingEnabled = event->sceneWidget()->isHighlightingEnabled();
    enableHighlight(isHighlightingEnabled && isEditMode && isSelected);
}


bool OperableSceneBody::Impl::finishEditing()
{
    bool finished = false;
    
    if(dragMode == LINK_VIRTUAL_ELASTIC_STRING){
        finishVirtualElasticString();
        finished = true;

    } else if(dragMode == LINK_FORCED_POSITION){
        finishForcedPosition();
        finished = true;
        
    } else if(dragMode != DRAG_NONE){
        if(dragged){
            bodyItem->notifyKinematicStateUpdate(false);
        }
        finished = true;
    }
    
    dragMode = DRAG_NONE;
    isDragging = false;
    dragged = false;

    return finished;
}


bool OperableSceneBody::onButtonPressEvent(SceneWidgetEvent* event)
{
    return impl->onButtonPressEvent(event);
}


bool OperableSceneBody::Impl::onButtonPressEvent(SceneWidgetEvent* event)
{
    if(highlightedLink){
        highlightedLink->enableHighlight(false);
        highlightedLink = nullptr;
    }
    
    PointedType pointedType = findPointedObject(event->nodePath());

    if(pointedType == PT_ZMP && event->button() == Qt::LeftButton){
        if(!bodyItem->isDoingContinuousKinematicUpdate()){
            startZmpTranslation(event);
            return true;
        }
        return false;
    }    

    if(!pointedSceneLink){
        return false;
    }
    targetLink = pointedSceneLink->link();

    int operationType = LinkOperationType::None;
    bool doEnableHighlight = false;
    auto bsm = BodySelectionManager::instance();

    if(event->button() == Qt::LeftButton || event->button() == Qt::RightButton){
        bsm->setCurrent(bodyItem, targetLink, true);
    }
        
    if(event->button() == Qt::RightButton){
        // The context menu is about to be shown
        doEnableHighlight = true;
    } else {
        operationType = checkLinkOperationType(pointedSceneLink, true);
        if(operationType != LinkOperationType::None){
            doEnableHighlight = true;
        }
    }
    
    if(doEnableHighlight){
        pointedSceneLink->enableHighlight(true);
        highlightedLink = pointedSceneLink;
    }
    if(operationType == LinkOperationType::None){
        return false;
    }

    bool handled = false;
    isDragging = false;
    dragged = false;

    if(operationType == LinkOperationType::SimInterference){
        startLinkOperationDuringSimulation(event);
        handled = true;
    } else {
        if(event->button() == Qt::LeftButton){
            updateMarkersAndManipulators(true);

            if(!bodyItem->isDoingContinuousKinematicUpdate()){
                if(operationType == LinkOperationType::FK){
                    startFK(event);
                } else if(operationType == LinkOperationType::IK){
                    startIK(event);
                }
            }
            handled = true;
        }
    }

    if((dragMode != DRAG_NONE) && highlightedLink){
        highlightedLink->enableHighlight(false);
        self->notifyUpdate(update.withAction(SgUpdate::Modified));
    }

    return handled;
}


bool OperableSceneBody::onButtonReleaseEvent(SceneWidgetEvent* event)
{
    return impl->onButtonReleaseEvent(event);
}


bool OperableSceneBody::Impl::onButtonReleaseEvent(SceneWidgetEvent* event)
{
    bool handled = finishEditing();

    if(highlightedLink){
        highlightedLink->enableHighlight(true);
        self->notifyUpdate(update.withAction(SgUpdate::Modified));
    }

    return handled;
}


bool OperableSceneBody::onDoubleClickEvent(SceneWidgetEvent* event)
{
    if(impl->targetLink){
        if(impl->checkLinkKinematicsType(impl->targetLink, false) != LinkOperationType::None){
            // prevent returning from the edit mode to the view mode
            return true;
        }
    }
    return false;
            
    // return impl->makePointedLinkCurrent();
}


/*
bool OperableSceneBody::Impl::makePointedLinkCurrent()
{
    if(event->button() == Qt::LeftButton){
        if(findPointedObject(event->nodePath()) == PT_SCENE_LINK){
            BodySelectionManager::instance()->setCurrent(bodyItem, targetLink, true);
            return true
        }
    }
    return false;
}
*/


bool OperableSceneBody::onPointerMoveEvent(SceneWidgetEvent* event)
{
    return impl->onPointerMoveEvent(event);
}


bool OperableSceneBody::Impl::onPointerMoveEvent(SceneWidgetEvent* event)
{
    if(dragMode == DRAG_NONE){
        findPointedObject(event->nodePath());
        if(!pointedSceneLink){
            event->updateIndicator("");
        } else {
            if(checkLinkOperationType(pointedSceneLink, false) != LinkOperationType::None){
                if(pointedSceneLink != highlightedLink){
                    if(highlightedLink){
                        highlightedLink->enableHighlight(false);
                    }
                    pointedSceneLink->enableHighlight(true);
                    highlightedLink = pointedSceneLink;
                }
            }
            const Vector3 p = pointedSceneLink->T().inverse() * event->point();
            event->updateIndicator(
                fmt::format("{0} / {1} : ({2:.3f}, {3:.3f}, {4:.3f})",
                            bodyItem->displayName(), pointedSceneLink->link()->name(),
                            p.x(), p.y(), p.z()));
        }
    } else {
        if(!isDragging){
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


void OperableSceneBody::onPointerLeaveEvent(SceneWidgetEvent* event)
{
    return impl->onPointerLeaveEvent(event);
}


void OperableSceneBody::Impl::onPointerLeaveEvent(SceneWidgetEvent* event)
{
    if(highlightedLink){
        highlightedLink->enableHighlight(false);
        highlightedLink = nullptr;
    }
}


bool OperableSceneBody::onScrollEvent(SceneWidgetEvent* event)
{
    return impl->onScrollEvent(event);
}


bool OperableSceneBody::Impl::onScrollEvent(SceneWidgetEvent* event)
{
    return false;
}


bool OperableSceneBody::onKeyPressEvent(SceneWidgetEvent* event)
{
    return impl->onKeyPressEvent(event);
}


bool OperableSceneBody::Impl::onKeyPressEvent(SceneWidgetEvent* event)
{
    if(!highlightedLink){
        return false;
    }

    bool handled = true;

    switch(event->key()){
    case Qt::Key_B:
        toggleBaseLink(highlightedLink);
        break;
        
    case Qt::Key_R:
        togglePin(highlightedLink, false, true);
        break;

    case Qt::Key_T:
        togglePin(highlightedLink, true, false);
        break;

    default:
        handled = false;
        break;
    }
        
    return handled;

}


bool OperableSceneBody::onKeyReleaseEvent(SceneWidgetEvent* event)
{
    return impl->onKeyReleaseEvent(event);
}


bool OperableSceneBody::Impl::onKeyReleaseEvent(SceneWidgetEvent* event)
{
    return false;
}


void OperableSceneBody::onFocusChanged(SceneWidgetEvent* event, bool on)
{
    impl->isFocused = on;
    if(!on){
        impl->updateMarkersAndManipulators(false);
    }
}


bool OperableSceneBody::onContextMenuRequest(SceneWidgetEvent* event)
{
    return impl->onContextMenuRequest(event);
}


bool OperableSceneBody::Impl::onContextMenuRequest(SceneWidgetEvent* event)
{
    PointedType pointedType = findPointedObject(event->nodePath());

    if(pointedType != PT_SCENE_LINK){
        return false;
    }

    auto menu = event->contextMenu();
    auto locationLockCheck = menu->addCheckItem(_("Lock location"));
    locationLockCheck->setChecked(bodyItem->isLocationLocked());
    locationLockCheck->sigToggled().connect(
        [this](bool on){ bodyItem->setLocationLocked(on); });
                    
    activeSimulatorItem = SimulatorItem::findActiveSimulatorItemFor(bodyItem);
    if(activeSimulatorItem){
        if(pointedSceneLink->link()->isRoot() && !bodyItem->isLocationLocked()){
            Action* item1 = menu->addCheckItem(_("Move Forcibly"));
            item1->setChecked(forcedPositionMode == MOVE_FORCED_POSITION);
            item1->sigToggled().connect(
                [this](bool on){ setForcedPositionMode(MOVE_FORCED_POSITION, on); });
                    
            Action* item2 = menu->addCheckItem(_("Hold Forcibly"));
            item2->setChecked(forcedPositionMode == KEEP_FORCED_POSITION);
            item2->sigToggled().connect(
                [this](bool on){ setForcedPositionMode(KEEP_FORCED_POSITION, on); });
                    
            menu->addSeparator();
        }
    } else {
        menu->addItem(_("Set Free"))->sigTriggered().connect(
            [this](){ makeLinkFree(pointedSceneLink); });
        menu->addItem(_("Set Base"))->sigTriggered().connect(
            [this](){ setBaseLink(pointedSceneLink); });
        menu->addItem(_("Set Translation Pin"))->sigTriggered().connect(
            [this](){ togglePin(pointedSceneLink, true, false); });
        menu->addItem(_("Set Rotation Pin"))->sigTriggered().connect(
            [this](){ togglePin(pointedSceneLink, false, true); });
        menu->addItem(_("Set Both Pins"))->sigTriggered().connect(
            [this](){ togglePin(pointedSceneLink, true, true); });
                
        menu->addSeparator();
            
        menu->addItem(_("Level Attitude"))->sigTriggered().connect(
            [this](){ makeLinkAttitudeLevel(pointedSceneLink); });
            
        menu->addSeparator();
    }

    menu->setPath(_("Markers"));

    int numLinks = self->numSceneLinks();
    
    auto linkOriginAction = new CheckBoxAction(_("Link Origins"));
    int numOriginsShown = 0;
    for(int i=0; i < numLinks; ++i){
        if(self->operableSceneLink(i)->isOriginShown()){
            ++numOriginsShown;
        }
    }
    auto originsCheck = linkOriginAction->checkBox();
    originsCheck->setTristate();
    if(numOriginsShown == 0){
        originsCheck->setCheckState(Qt::Unchecked);
    } else if(numOriginsShown == numLinks){
        originsCheck->setCheckState(Qt::Checked);
    } else {
        originsCheck->setCheckState(Qt::PartiallyChecked);
    }
    menu->addAction(linkOriginAction);
    originsCheck->sigToggled().connect([this](bool on){ onLinkOriginsCheckToggled(on); });

    auto linkCmAction = new CheckBoxAction(_("Link Center of Masses"));
    int numCmsShown = 0;
    for(int i=0; i < numLinks; ++i){
        if(self->operableSceneLink(i)->isCenterOfMassShown()){
            ++numCmsShown;
        }
    }
    auto cmsCheck = linkCmAction->checkBox();
    cmsCheck->setTristate();
    if(numCmsShown == 0){
        cmsCheck->setCheckState(Qt::Unchecked);
    } else if(numCmsShown == numLinks){
        cmsCheck->setCheckState(Qt::Checked);
    } else {
        cmsCheck->setCheckState(Qt::PartiallyChecked);
    }
    menu->addAction(linkCmAction);
    cmsCheck->sigToggled().connect([this](bool on){ onLinkCmsCheckToggled(on); });
    
    auto item = menu->addCheckItem(_("Center of Mass"));
    item->setChecked(isCmVisible);
    item->sigToggled().connect([this](bool on){ showCenterOfMass(on); });
            
    item = menu->addCheckItem(_("Center of Mass Projection"));
    item->setChecked(isCmProjectionVisible);
    item->sigToggled().connect([this](bool on){ showCmProjection(on); });


    if(checkLeggedBody()){
        item = menu->addCheckItem(_("ZMP"));
        item->setChecked(isZmpVisible);
        item->sigToggled().connect([this](bool on){ showZmp(on); });
    }

    menu->setPath("/");
    menu->addSeparator();

    return true;
}


void OperableSceneBody::Impl::onDraggerDragStarted()
{
    activeSimulatorItem = SimulatorItem::findActiveSimulatorItemFor(bodyItem);
    if(!activeSimulatorItem){
        initializeIK();
    }
    dragged = false;
}


void OperableSceneBody::Impl::onDraggerDragged()
{
    activeSimulatorItem = SimulatorItem::findActiveSimulatorItemFor(bodyItem);
    if(activeSimulatorItem){
        setForcedPosition(positionDragger->globalDraggingPosition());
    } else {
        doIK(positionDragger->globalDraggingPosition());
        dragged = true;
    }
}


void OperableSceneBody::Impl::onDraggerDragFinished()
{
    activeSimulatorItem = SimulatorItem::findActiveSimulatorItemFor(bodyItem);
    if(activeSimulatorItem){
        finishForcedPosition();
    } else {
        doIK(positionDragger->globalDraggingPosition());
        if(dragged){
            bodyItem->notifyKinematicStateUpdate(false);
        }
    }
    dragged = false;
}


bool OperableSceneBody::Impl::initializeIK()
{
    if(!currentIK){
        if(auto pin = bodyItem->checkPinDragIK()){
            if(pin->numPinnedLinks() > 0){
                pinDragIK = pin;
                pinDragIK->setBaseLink(bodyItem->currentBaseLink());
                pinDragIK->setTargetLink(targetLink, kinematicsBar->isPositionDraggerEnabled());
                if(pinDragIK->initialize()){
                    currentIK = pinDragIK;
                }
            }
        }
    }
    if(!currentIK){
        currentIK = bodyItem->getCurrentIK(targetLink);
    }
    if(auto jointPath = dynamic_pointer_cast<JointPath>(currentIK)){
        if(!jointPath->hasCustomIK()){
            jointPath->setBestEffortIkMode(true);
        }
    }

    return currentIK ? true: false;
}


void OperableSceneBody::Impl::startIK(SceneWidgetEvent* event)
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


void OperableSceneBody::Impl::dragIK(SceneWidgetEvent* event)
{
    if(dragProjector.dragTranslation(event)){
        //Position T = dragProjector.initialPosition();
        Isometry3 T;
        T.translation() = dragProjector.position().translation();
        T.linear() = targetLink->R();
        if(penetrationBlocker){
            penetrationBlocker->adjust(T, T.translation() - targetLink->p());
        }
        doIK(T);
        dragged = true;
    }
}


void OperableSceneBody::Impl::doIK(const Isometry3& position)
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


void OperableSceneBody::Impl::startFK(SceneWidgetEvent* event)
{
    dragProjector.setInitialPosition(targetLink->position());
    
    orgJointPosition = targetLink->q();

    if(!linkedJointHandler){
        linkedJointHandler = LinkedJointHandler::findOrCreateLinkedJointHandler(bodyItem->body());
    }
    
    if(targetLink->isRevoluteJoint()){
        dragProjector.setRotationAxis(targetLink->R() * targetLink->a());
        if(dragProjector.startRotation(event)){
            dragMode = LINK_FK_ROTATION;
        }
        
    } else if(targetLink->isPrismaticJoint()){
        dragProjector.setTranslationAxis(targetLink->R() * targetLink->d());
        if(dragProjector.startTranslation(event)){
            dragMode = LINK_FK_TRANSLATION;
        }
    }
}


void OperableSceneBody::Impl::dragFKRotation(SceneWidgetEvent* event)
{
    if(dragProjector.dragRotation(event)){
        double q = orgJointPosition + dragProjector.rotationAngle();
        double q_clamped = stdx::clamp(q, targetLink->q_lower(), targetLink->q_upper());
        if(linkedJointHandler->updateLinkedJointDisplacements(targetLink, q_clamped)){
            linkedJointHandler->limitLinkedJointDisplacementsWithinMovableRanges(targetLink);
        }
        bodyItem->notifyKinematicStateChange(true);
        dragged = true;
    }
}


void OperableSceneBody::Impl::dragFKTranslation(SceneWidgetEvent* event)
{
    if(dragProjector.dragTranslation(event)){
        double q = orgJointPosition + dragProjector.translationAxis().dot(dragProjector.translation());
        double q_clamped = stdx::clamp(q, targetLink->q_lower(), targetLink->q_upper());
        if(linkedJointHandler->updateLinkedJointDisplacements(targetLink, q_clamped)){
            linkedJointHandler->limitLinkedJointDisplacementsWithinMovableRanges(targetLink);
        }
        bodyItem->notifyKinematicStateChange(true);
        dragged = true;
    }
}


void OperableSceneBody::Impl::startLinkOperationDuringSimulation(SceneWidgetEvent* event)
{
    if(event->button() == Qt::LeftButton){
        if(targetLink->isRoot() && (forcedPositionMode != NO_FORCED_POSITION)){
            startForcedPosition(event);
        } else {
            startVirtualElasticString(event);
        }
    }
}


void OperableSceneBody::Impl::setForcedPositionMode(int mode, bool on)
{
    if(on){
        forcedPositionMode = mode;
    } else {
        forcedPositionMode = NO_FORCED_POSITION;
        updateMarkersAndManipulators(isFocused);
    }
    finishForcedPosition();
}


void OperableSceneBody::Impl::startVirtualElasticString(SceneWidgetEvent* event)
{
    const Vector3& point = event->point();
    dragProjector.setInitialTranslation(point);
    dragProjector.setTranslationAlongViewPlane();
    if(dragProjector.startTranslation(event)){
        pointedLinkLocalPoint = targetLink->T().inverse() * point;
        dragMode = LINK_VIRTUAL_ELASTIC_STRING;
        dragVirtualElasticString(event);
        markerGroup->addChildOnce(virtualElasticStringLine, update);
        dragged = true;
    }
}


void OperableSceneBody::Impl::dragVirtualElasticString(SceneWidgetEvent* event)
{
    if(dragMode == LINK_VIRTUAL_ELASTIC_STRING){
        SimulatorItem* simulatorItem = activeSimulatorItem.lock();
        if(simulatorItem && dragProjector.dragTranslation(event)){
            Vector3 p = targetLink->T() * pointedLinkLocalPoint;
            Vector3 d = dragProjector.position().translation() - p;
            double k = 2.0;
            if(event->modifiers() & Qt::ShiftModifier){
                k *= 10.0;
                if(event->modifiers() & Qt::ControlModifier){
                    k *= 10.0;
                }
            }
            Vector3 end = p + k * self->boundingBox().boundingSphereRadius() * d;
            SgVertexArray& points = *virtualElasticStringLine->vertices();
            points[0] = p.cast<Vector3f::Scalar>();
            points[1] = (p + d).cast<Vector3f::Scalar>();
            virtualElasticStringLine->notifyUpdate(update.withAction(SgUpdate::Modified));
            simulatorItem->setVirtualElasticString(bodyItem, targetLink, pointedLinkLocalPoint, end);
        }
    }
}


void OperableSceneBody::Impl::finishVirtualElasticString()
{
    if(SimulatorItem* simulatorItem = activeSimulatorItem.lock()){
        simulatorItem->clearVirtualElasticStrings();
    }
    markerGroup->removeChild(virtualElasticStringLine, update);
}


void OperableSceneBody::Impl::startForcedPosition(SceneWidgetEvent* event)
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


void OperableSceneBody::Impl::setForcedPosition(const Isometry3& position)
{
    if(SimulatorItem* simulatorItem = activeSimulatorItem.lock()){
        simulatorItem->setForcedPosition(bodyItem, position);
    }
}
    
    
void OperableSceneBody::Impl::dragForcedPosition(SceneWidgetEvent* event)
{
    if(dragProjector.dragTranslation(event)){
        Isometry3 T;
        T.translation() = dragProjector.position().translation();
        T.linear() = targetLink->R();
        setForcedPosition(T);
        dragged = true;
    }
}


void OperableSceneBody::Impl::finishForcedPosition()
{
    if(forcedPositionMode != KEEP_FORCED_POSITION){
        if(SimulatorItem* simulatorItem = activeSimulatorItem.lock()){
            simulatorItem->clearForcedPositions();
        }
    }
}


void OperableSceneBody::Impl::startZmpTranslation(SceneWidgetEvent* event)
{
    dragProjector.setInitialTranslation(bodyItem->zmp());
    dragProjector.setTranslationPlaneNormal(Vector3::UnitZ());
    if(dragProjector.startTranslation(event)){
        dragMode = ZMP_TRANSLATION;
    }
}


void OperableSceneBody::Impl::dragZmpTranslation(SceneWidgetEvent* event)
{
    if(dragProjector.dragTranslation(event)){
        Vector3 p = dragProjector.position().translation();
        p.z() = dragProjector.initialPosition().translation().z();
        bodyItem->setZmp(p);
        bodyItem->notifyKinematicStateChange(true);
        dragged = true;
    }
}


bool OperableSceneBody::Impl::storeProperties(Archive& archive)
{
    ListingPtr states = new Listing();

    for(auto& bodyItem : RootItem::instance()->descendantItems<BodyItem>()){
        OperableSceneBody* sceneBody = bodyItem->existingSceneBody();
        if(sceneBody){
            if(auto id = archive.getItemIdNode(bodyItem)){
                OperableSceneBody::Impl* impl = sceneBody->impl;
                MappingPtr state = new Mapping();
                state->insert("body_item", id);
                state->write("show_cm", impl->isCmVisible);
                state->write("show_cm_projection", impl->isCmProjectionVisible);
                state->write("show_zmp", impl->isZmpVisible);
                states->append(state);
            }
        }
    }
    if(!states->empty()){
        archive.insert("scene_bodies", states);
    }

    return true;
}
    
    
void OperableSceneBody::Impl::restoreProperties(const Archive& archive)
{
    archive.addPostProcess(
        [&archive](){ restoreSceneBodyProperties(archive); },
        1);
}


void OperableSceneBody::Impl::restoreSceneBodyProperties(const Archive& archive)
{
    Listing& states = *archive.findListing({ "scene_bodies", "editableSceneBodies" });
    if(states.isValid()){
        for(int i=0; i < states.size(); ++i){
            Mapping* state = states[i].toMapping();
            BodyItem* bodyItem = archive.findItem<BodyItem>(state->find({ "body_item", "bodyItem" }));
            if(bodyItem){
                OperableSceneBody::Impl* impl = bodyItem->sceneBody()->impl;
                impl->showCenterOfMass(state->get({ "show_cm", "showCenterOfMass" }, impl->isCmVisible));
                impl->showCmProjection(state->get({ "show_cm_projection", "showCmProjection" }, impl->isCmProjectionVisible));
                impl->showZmp(state->get({ "show_zmp", "showZmp" }, impl->isZmpVisible));
            }
        }
    }
}


void OperableSceneBody::initializeClass(ExtensionManager* ext)
{
    ext->setProjectArchiver(
        "OperableSceneBody",
        OperableSceneBody::Impl::storeProperties,
        OperableSceneBody::Impl::restoreProperties);

    ext->setProjectArchiver(
        "EditableSceneBody",
        nullptr,
        OperableSceneBody::Impl::restoreProperties);
}
