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
    SgUpdate& update;
    SgPolygonDrawStylePtr highlightStyle;
    BoundingBoxMarkerPtr bbMarker;
    bool isOriginShown;
    bool isPointed;
    bool isColliding;

    Impl(EditableSceneBody* sceneBody, EditableSceneLink* self);
    void showOrigin(bool on);
};

class EditableSceneBody::Impl
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EditableSceneBody* self;
    BodyItemPtr bodyItem;

    SgUpdate update;

    ScopedConnectionSet connections;
    ScopedConnection connectionToSigCollisionsUpdated;
    vector<bool> collisionLinkBitSet;
    ScopedConnection connectionToSigLinkSelectionChanged;

    enum PointedType { PT_NONE, PT_SCENE_LINK, PT_ZMP };
    EditableSceneLink* pointedSceneLink;
    EditableSceneLink* highlightedLink;

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
    bool isSelected;
    bool isHighlightingEnabled;

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

    Impl(EditableSceneBody* self, BodyItem* bodyItem);
    void initialize();
        
    EditableSceneLink* editableSceneLink(int index){
        return static_cast<EditableSceneLink*>(self->sceneLink(index));
    }

    double calcLinkMarkerRadius(SceneLink* sceneLink) const;
    void onSceneGraphConnection(bool on);
    void updateModel();
    void onSelectionChanged(bool on);
    void onKinematicStateChanged();
    void onCollisionsUpdated();
    void onCollisionLinkHighlightModeChanged();
    void changeCollisionLinkHighlightMode(bool on);
    void onLinkVisibilityCheckToggled();
    void onLinkSelectionChanged(const std::vector<bool>& selection);
    void onLinkOriginsCheckChanged(bool on);

    void enableHighlight(bool on);
    void calcBodyMarkerRadius();
    void ensureCmMarker();
    void ensureCmProjectionMarker();
    LeggedBodyHelper* checkLeggedBody();
    bool ensureZmpMarker();
    void showCenterOfMass(bool on);
    void showCmProjection(bool on);
    void showZmp(bool on);
    void makeLinkFree(EditableSceneLink* sceneLink);
    void setBaseLink(EditableSceneLink* sceneLink);
    void toggleBaseLink(EditableSceneLink* sceneLink);
    void togglePin(EditableSceneLink* sceneLink, bool toggleTranslation, bool toggleRotation);
    void makeLinkAttitudeLevel();
        
    PointedType findPointedObject(const SgNodePath& path);
    int checkLinkOperationType(SceneLink* sceneLink, bool doUpdateIK);
    int checkLinkKinematicsType(Link* link, bool doUpdateIK);
    void updateMarkersAndManipulators(bool on);
    void createPositionDragger();
    void attachPositionDragger(Link* link);
    void adjustPositionDraggerSize(Link* link, EditableSceneLink* sceneLink);

    bool onKeyPressEvent(SceneWidgetEvent* event);
    bool onKeyReleaseEvent(SceneWidgetEvent* event);
    bool onButtonPressEvent(SceneWidgetEvent* event);
    bool onButtonReleaseEvent(SceneWidgetEvent* event);
    bool onPointerMoveEvent(SceneWidgetEvent* event);
    void onPointerLeaveEvent(SceneWidgetEvent* event);
    bool onScrollEvent(SceneWidgetEvent* event);
    bool onContextMenuRequest(SceneWidgetEvent* event, MenuManager* menuManager);
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


EditableSceneLink::EditableSceneLink(EditableSceneBody* sceneBody, Link* link)
    : SceneLink(sceneBody, link)
{
    impl = new Impl(sceneBody, this);
}


EditableSceneLink::Impl::Impl(EditableSceneBody* sceneBody, EditableSceneLink* self)
    : self(self),
      update(sceneBody->impl->update)
{
    isOriginShown = false;
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
    if(on != isOriginShown){

        auto& originMarker = static_cast<EditableSceneBody*>(self->sceneBody())->impl->linkOriginMarker;

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
            self->addChildOnce(originMarker, update);
            
        } else {
            if(originMarker && originMarker->hasParents()){
                self->removeChild(originMarker, update);
            }
        }
    }
}


bool EditableSceneLink::isOriginShown() const
{
    return impl->isOriginShown;
}


void EditableSceneLink::enableHighlight(bool on)
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


void EditableSceneLink::showMarker(const Vector3f& color, float transparency)
{
    if(impl->bbMarker){
        removeChild(impl->bbMarker, impl->update);
    }
    if(visualShape()){
        impl->bbMarker = new BoundingBoxMarker(visualShape()->boundingBox(), color, transparency);
        addChildOnce(impl->bbMarker, impl->update);
    }
}


void EditableSceneLink::hideMarker()
{
    if(impl->bbMarker){
        removeChild(impl->bbMarker, impl->update);
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


EditableSceneBody::EditableSceneBody(BodyItem* bodyItem)
{
    setAttribute(Operable);
    impl = new Impl(this, bodyItem);
    impl->initialize();
}


EditableSceneBody::Impl::Impl(EditableSceneBody* self, BodyItem* bodyItem)
    : self(self),
      bodyItem(bodyItem),
      kinematicsBar(KinematicsBar::instance())
{

}


void EditableSceneBody::Impl::initialize()
{
    pointedSceneLink = nullptr;
    highlightedLink = nullptr;
    targetLink = nullptr;

    isEditMode = false;
    isFocused = false;
    isSelected = false;
    isHighlightingEnabled = false;

    self->setBody(bodyItem->body(), [this](Link* link){ return new EditableSceneLink(self, link); });

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
            bodyItem->sigSelectionChanged().connect(
                [&](bool on){ onSelectionChanged(on); }));

        onSelectionChanged(bodyItem->isSelected()); 

        connections.add(
            bodyItem->sigUpdated().connect(
                [&](){
                    if(isFocused){ updateMarkersAndManipulators(true); }
                }));

        connections.add(
            bodyItem->sigKinematicStateChanged().connect(
                [&](){ onKinematicStateChanged(); }));
            
        onKinematicStateChanged();

        connections.add(
            bodyItem->getLocationProxy()->sigAttributeChanged().connect(
                [&](){
                    bool on = bodyItem->isLocationEditable();
                    if(!on){
                        if(highlightedLink){
                            highlightedLink->enableHighlight(false);
                            highlightedLink = nullptr;
                        }
                        updateMarkersAndManipulators(false);
                    }
                }));

        connections.add(
            kinematicsBar->sigCollisionVisualizationChanged().connect(
                [&](){ onCollisionLinkHighlightModeChanged(); }));
        
        onCollisionLinkHighlightModeChanged();

        /*
        connections.add(
            bodyItem->sigModelUpdated().connect(
                [&](){ updateModel(); }));
        */

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
    if(highlightedLink){
        highlightedLink->enableHighlight(false);
        highlightedLink = nullptr;
    }
    dragMode = DRAG_NONE;
    isDragging = false;
    dragged = false;
    
    self->SceneBody::updateModel();
}


EditableSceneLink* EditableSceneBody::editableSceneLink(int index)
{
    return static_cast<EditableSceneLink*>(sceneLink(index));
}


void EditableSceneBody::Impl::onSelectionChanged(bool on)
{
    isSelected = on;
    enableHighlight(isHighlightingEnabled && isEditMode && isSelected);
}


void EditableSceneBody::Impl::onKinematicStateChanged()
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


void EditableSceneBody::Impl::onCollisionsUpdated()
{
    if(bodyItem->collisionLinkBitSet() != collisionLinkBitSet){
        collisionLinkBitSet = bodyItem->collisionLinkBitSet();
        const int n = self->numSceneLinks();
        for(int i=0; i < n; ++i){
            editableSceneLink(i)->setColliding(collisionLinkBitSet[i]);
        }
        self->notifyUpdate(update.withAction(SgUpdate::Modified));
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
        self->notifyUpdate(update.withAction(SgUpdate::Modified));
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
    notifyUpdate(impl->update.withAction(SgUpdate::Modified));
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


void EditableSceneBody::Impl::enableHighlight(bool on)
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
            if(self->numSceneLinks() == 1){
                self->sceneLink(0)->insertEffectGroup(highlight, update);
            } else{
                self->insertEffectGroup(highlight, update);
            }
        }
    } else {
        if(highlight && highlight->hasParents()){
            if(self->numSceneLinks() == 1){
                self->sceneLink(0)->removeEffectGroup(highlight, update);
            } else {
                self->removeEffectGroup(highlight, update);
            }
        }
    }
}


void EditableSceneBody::Impl::calcBodyMarkerRadius()
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


void EditableSceneBody::Impl::ensureCmMarker()
{
    if(!cmMarker){
        if(bodyMarkerRadius < 0.0){
            calcBodyMarkerRadius();
        }
        cmMarker = new CrossMarker(bodyMarkerRadius, Vector3f(0.0f, 1.0f, 0.0f), 2.0);
        cmMarker->setName("centerOfMass");
    }
}


void EditableSceneBody::Impl::ensureCmProjectionMarker()
{
    if(!cmProjectionMarker){
        if(bodyMarkerRadius < 0.0){
            calcBodyMarkerRadius();
        }
        cmProjectionMarker = new CrossMarker(bodyMarkerRadius, Vector3f(1.0f, 0.5f, 0.0f), 2.0);
        cmProjectionMarker->setName("CmProjection");
    }
}


LeggedBodyHelper* EditableSceneBody::Impl::checkLeggedBody()
{
    auto legged = getLeggedBodyHelper(self->body());
    if(!legged->isValid() || legged->numFeet() == 0){
        legged = nullptr;
    }
    return legged;
}


bool EditableSceneBody::Impl::ensureZmpMarker()
{
    if(!zmpMarker){
        if(auto legged = checkLeggedBody()){
            Link* footLink = legged->footLink(0);
            double radius = calcLinkMarkerRadius(self->sceneLink(footLink->index()));
            zmpMarker = new SphereMarker(radius, Vector3f(0.0f, 1.0f, 0.0f), 0.3);
            zmpMarker->addChild(new CrossMarker(radius * 2.5, Vector3f(0.0f, 1.0f, 0.0f), 2.0f));
            zmpMarker->setName("ZMP");
        }
    }
    return (zmpMarker != nullptr);
}


void EditableSceneBody::Impl::showCenterOfMass(bool on)
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


void EditableSceneBody::Impl::showCmProjection(bool on)
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


void EditableSceneBody::Impl::showZmp(bool on)
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


void EditableSceneBody::Impl::makeLinkFree(EditableSceneLink* sceneLink)
{
    if(bodyItem->currentBaseLink() == sceneLink->link()){
        bodyItem->setCurrentBaseLink(nullptr);
    }
    if(auto pin = bodyItem->checkPinDragIK()){
        pin->setPin(sceneLink->link(), PinDragIK::NO_AXES);
    }
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


void EditableSceneBody::Impl::makeLinkAttitudeLevel()
{
    if(pointedSceneLink){
        Link* link = highlightedLink->link();
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
}


EditableSceneBody::Impl::PointedType EditableSceneBody::Impl::findPointedObject(const SgNodePath& path)
{
    PointedType pointedType = PT_NONE;
    pointedSceneLink = nullptr;
    for(size_t i = path.size() - 1; i >= 1; --i){
        pointedSceneLink = dynamic_cast<EditableSceneLink*>(path[i]);
        if(pointedSceneLink){
            pointedType = PT_SCENE_LINK;
            break;
        }
        if(auto marker = dynamic_cast<SphereMarker*>(path[i])){
            if(marker == zmpMarker){
                pointedType = PT_ZMP;
                break;
            }
        }
    }
    return pointedType;
}


int EditableSceneBody::Impl::checkLinkOperationType(SceneLink* sceneLink, bool doUpdateIK)
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


int EditableSceneBody::Impl::checkLinkKinematicsType(Link* link, bool doUpdateIK)
{
    // Check if the link is editable considering the ancestor bodies
    BodyItem* bodyItemChain = bodyItem;
    Link* linkChain = link;
    while(true){
        if(!bodyItemChain->isAttachedToParentBody()){
            if(!bodyItemChain->isLocationEditable() && linkChain->isFixedToRoot()){
                return LinkOperationType::None;
            }
            break;
        }
        if(!linkChain->isRoot()){
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


void EditableSceneBody::Impl::updateMarkersAndManipulators(bool on)
{
    Link* baseLink = bodyItem->currentBaseLink();
    auto pin = bodyItem->checkPinDragIK();

    const int n = self->numSceneLinks();
    for(int i=0; i < n; ++i){
        EditableSceneLink* sceneLink = editableSceneLink(i);
        sceneLink->hideMarker();
        sceneLink->removeChild(positionDragger, update);

        if(on && isEditMode && !activeSimulatorItem){
            Link* link = sceneLink->link();
            if(link == baseLink){
                sceneLink->showMarker(Vector3f(1.0f, 0.1f, 0.1f), 0.4);
            } else if(pin){
                int pinAxes = pin->pinAxes(link);
                if(pinAxes & (PinDragIK::TRANSFORM_6D)){
                    sceneLink->showMarker(Vector3f(1.0f, 1.0f, 0.1f), 0.4);
                }
            }
        }
    }

    // The following connection is only necessary when the position dragger is shown
    kinematicsKitConnection.disconnect();
}


void EditableSceneBody::Impl::createPositionDragger()
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
    positionDragger->sigDragStarted().connect([&](){ onDraggerDragStarted(); });
    positionDragger->sigPositionDragged().connect([&](){ onDraggerDragged(); });
    positionDragger->sigDragFinished().connect([&](){ onDraggerDragFinished(); });
}

    
void EditableSceneBody::Impl::attachPositionDragger(Link* link)
{
    if(!positionDragger){
        createPositionDragger();
    }
    
    LinkKinematicsKit* kinematicsKit = nullptr;
    if(link->isRoot() && bodyItem->isAttachedToParentBody()){
        auto parentBodyLink = bodyItem->body()->parentBodyLink();
        if(!parentBodyLink->isRoot()){
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
            kinematicsKit->sigFrameUpdate().connect(
                [this, link](){ attachPositionDragger(link); });
    }
    
    auto sceneLink = editableSceneLink(link->index());

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
}


/*
   To automatically adjust the size of the position dragger to a reasonable size,
   the sizes of the neighboring links of the target link are also taken into account
   to determine the size.
*/
void EditableSceneBody::Impl::adjustPositionDraggerSize(Link* link, EditableSceneLink* sceneLink)
{
    BoundingBox bb;
    
    if(auto shape = sceneLink->visualShape()){
        bb.expandBy(shape->untransformedBoundingBox());
    }
    constexpr double s = 0.5;
    if(auto parent = link->parent()){
        if(auto parentSceneLink = editableSceneLink(parent->index())){
            auto parentBBox = parentSceneLink->untransformedBoundingBox();
            parentBBox.scale(s);
            bb.expandBy(parentBBox);
        }
    }
    for(auto child = link->child(); child; child = child->sibling()){
        if(auto childSceneLink = editableSceneLink(child->index())){
            auto childBBox = childSceneLink->untransformedBoundingBox();
            childBBox.scale(s);
            bb.expandBy(childBBox);
        }
    }

    positionDragger->adjustSize(bb);
}


void EditableSceneBody::onSceneModeChanged(SceneWidgetEvent* event)
{
    impl->onSceneModeChanged(event);
}


void EditableSceneBody::Impl::onSceneModeChanged(SceneWidgetEvent* event)
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


bool EditableSceneBody::Impl::finishEditing()
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


bool EditableSceneBody::onButtonPressEvent(SceneWidgetEvent* event)
{
    return impl->onButtonPressEvent(event);
}


bool EditableSceneBody::Impl::onButtonPressEvent(SceneWidgetEvent* event)
{
    if(highlightedLink){
        highlightedLink->enableHighlight(false);
        highlightedLink = nullptr;
    }
    
    PointedType pointedType = findPointedObject(event->nodePath());

    if(pointedType == PT_ZMP && event->button() == Qt::LeftButton){
        startZmpTranslation(event);
        return true;
    }    

    if(!pointedSceneLink){
        return false;
    }
    targetLink = pointedSceneLink->link();

    int operationType = LinkOperationType::None;
    bool doEnableHighlight = false;
    auto bsm = BodySelectionManager::instance();
        
    if(event->button() == Qt::RightButton){
        // The context menu is about to be shown
        bsm->setCurrent(bodyItem, targetLink, true);
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

    if(dragMode != DRAG_NONE && highlightedLink){
        highlightedLink->enableHighlight(false);
        self->notifyUpdate(update.withAction(SgUpdate::Modified));
    }

    return handled;
}


bool EditableSceneBody::onButtonReleaseEvent(SceneWidgetEvent* event)
{
    return impl->onButtonReleaseEvent(event);
}


bool EditableSceneBody::Impl::onButtonReleaseEvent(SceneWidgetEvent* event)
{
    bool handled = finishEditing();

    if(highlightedLink){
        highlightedLink->enableHighlight(true);
        self->notifyUpdate(update.withAction(SgUpdate::Modified));
    }

    return handled;
}


bool EditableSceneBody::onDoubleClickEvent(SceneWidgetEvent* event)
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
bool EditableSceneBody::Impl::makePointedLinkCurrent()
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


bool EditableSceneBody::onPointerMoveEvent(SceneWidgetEvent* event)
{
    return impl->onPointerMoveEvent(event);
}


bool EditableSceneBody::Impl::onPointerMoveEvent(SceneWidgetEvent* event)
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


void EditableSceneBody::onPointerLeaveEvent(SceneWidgetEvent* event)
{
    return impl->onPointerLeaveEvent(event);
}


void EditableSceneBody::Impl::onPointerLeaveEvent(SceneWidgetEvent* event)
{
    if(highlightedLink){
        highlightedLink->enableHighlight(false);
        highlightedLink = nullptr;
    }
}


bool EditableSceneBody::onScrollEvent(SceneWidgetEvent* event)
{
    return impl->onScrollEvent(event);
}


bool EditableSceneBody::Impl::onScrollEvent(SceneWidgetEvent* event)
{
    return false;
}


bool EditableSceneBody::onKeyPressEvent(SceneWidgetEvent* event)
{
    return impl->onKeyPressEvent(event);
}


bool EditableSceneBody::Impl::onKeyPressEvent(SceneWidgetEvent* event)
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


bool EditableSceneBody::onKeyReleaseEvent(SceneWidgetEvent* event)
{
    return impl->onKeyReleaseEvent(event);
}


bool EditableSceneBody::Impl::onKeyReleaseEvent(SceneWidgetEvent* event)
{
    return false;
}


void EditableSceneBody::onFocusChanged(SceneWidgetEvent* event, bool on)
{
    impl->isFocused = on;
    if(!on){
        impl->updateMarkersAndManipulators(false);
    }
}


bool EditableSceneBody::onContextMenuRequest(SceneWidgetEvent* event, MenuManager* menuManager)
{
    return impl->onContextMenuRequest(event, menuManager);
}


bool EditableSceneBody::Impl::onContextMenuRequest(SceneWidgetEvent* event, MenuManager* mm)
{
    PointedType pointedType = findPointedObject(event->nodePath());

    if(pointedType != PT_SCENE_LINK){
        return false;
    }

    auto locationLockCheck = mm->addCheckItem(_("Lock location"));
    locationLockCheck->setChecked(!bodyItem->isLocationEditable());
    locationLockCheck->sigToggled().connect(
        [&](bool on){ bodyItem->setLocationEditable(!on); });
                    
    activeSimulatorItem = SimulatorItem::findActiveSimulatorItemFor(bodyItem);
    if(activeSimulatorItem){
        if(pointedSceneLink->link()->isRoot() && bodyItem->isLocationEditable()){
            Action* item1 = mm->addCheckItem(_("Move Forcibly"));
            item1->setChecked(forcedPositionMode == MOVE_FORCED_POSITION);
            item1->sigToggled().connect(
                [&](bool on){ setForcedPositionMode(MOVE_FORCED_POSITION, on); });
                    
            Action* item2 = mm->addCheckItem(_("Hold Forcibly"));
            item2->setChecked(forcedPositionMode == KEEP_FORCED_POSITION);
            item2->sigToggled().connect(
                [&](bool on){ setForcedPositionMode(KEEP_FORCED_POSITION, on); });
                    
            mm->addSeparator();
        }
    } else {
        mm->addItem(_("Set Free"))->sigTriggered().connect(
            [&](){ makeLinkFree(pointedSceneLink); });
        mm->addItem(_("Set Base"))->sigTriggered().connect(
            [&](){ setBaseLink(pointedSceneLink); });
        mm->addItem(_("Set Translation Pin"))->sigTriggered().connect(
            [&](){ togglePin(pointedSceneLink, true, false); });
        mm->addItem(_("Set Rotation Pin"))->sigTriggered().connect(
            [&](){ togglePin(pointedSceneLink, false, true); });
        mm->addItem(_("Set Both Pins"))->sigTriggered().connect(
            [&](){ togglePin(pointedSceneLink, true, true); });
                
        mm->addSeparator();
            
        mm->addItem(_("Level Attitude"))->sigTriggered().connect(
            [&](){ makeLinkAttitudeLevel(); });
            
        mm->addSeparator();
    }

    mm->setPath(_("Markers"));

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
    mm->addAction(linkOriginAction);
    check->sigToggled().connect([&](bool on){ onLinkOriginsCheckChanged(on); });
        
    auto item = mm->addCheckItem(_("Center of Mass"));
    item->setChecked(isCmVisible);
    item->sigToggled().connect([&](bool on){ showCenterOfMass(on); });
            
    item = mm->addCheckItem(_("Center of Mass Projection"));
    item->setChecked(isCmProjectionVisible);
    item->sigToggled().connect([&](bool on){ showCmProjection(on); });


    if(checkLeggedBody()){
        item = mm->addCheckItem(_("ZMP"));
        item->setChecked(isZmpVisible);
        item->sigToggled().connect([&](bool on){ showZmp(on); });
    }

    mm->setPath("/");
    mm->addSeparator();

    return true;
}


void EditableSceneBody::Impl::onDraggerDragStarted()
{
    activeSimulatorItem = SimulatorItem::findActiveSimulatorItemFor(bodyItem);
    if(!activeSimulatorItem){
        initializeIK();
    }
    dragged = false;
}


void EditableSceneBody::Impl::onDraggerDragged()
{
    activeSimulatorItem = SimulatorItem::findActiveSimulatorItemFor(bodyItem);
    if(activeSimulatorItem){
        setForcedPosition(positionDragger->globalDraggingPosition());
    } else {
        doIK(positionDragger->globalDraggingPosition());
        dragged = true;
    }
}


void EditableSceneBody::Impl::onDraggerDragFinished()
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


bool EditableSceneBody::Impl::initializeIK()
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


void EditableSceneBody::Impl::startIK(SceneWidgetEvent* event)
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


void EditableSceneBody::Impl::dragIK(SceneWidgetEvent* event)
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


void EditableSceneBody::Impl::doIK(const Isometry3& position)
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


void EditableSceneBody::Impl::startFK(SceneWidgetEvent* event)
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


void EditableSceneBody::Impl::dragFKRotation(SceneWidgetEvent* event)
{
    if(dragProjector.dragRotation(event)){
        targetLink->q() = orgJointPosition + dragProjector.rotationAngle();
        bodyItem->notifyKinematicStateChange(true);
        dragged = true;
    }
}


void EditableSceneBody::Impl::dragFKTranslation(SceneWidgetEvent* event)
{
    if(dragProjector.dragTranslation(event)){
        targetLink->q() = orgJointPosition + dragProjector.translationAxis().dot(dragProjector.translation());
        bodyItem->notifyKinematicStateChange(true);
        dragged = true;
    }
}


void EditableSceneBody::Impl::startLinkOperationDuringSimulation(SceneWidgetEvent* event)
{
    if(event->button() == Qt::LeftButton){
        if(targetLink->isRoot() && (forcedPositionMode != NO_FORCED_POSITION)){
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


void EditableSceneBody::Impl::startVirtualElasticString(SceneWidgetEvent* event)
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


void EditableSceneBody::Impl::dragVirtualElasticString(SceneWidgetEvent* event)
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


void EditableSceneBody::Impl::finishVirtualElasticString()
{
    if(SimulatorItem* simulatorItem = activeSimulatorItem.lock()){
        simulatorItem->clearVirtualElasticStrings();
    }
    markerGroup->removeChild(virtualElasticStringLine, update);
}


void EditableSceneBody::Impl::startForcedPosition(SceneWidgetEvent* event)
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


void EditableSceneBody::Impl::setForcedPosition(const Isometry3& position)
{
    if(SimulatorItem* simulatorItem = activeSimulatorItem.lock()){
        simulatorItem->setForcedPosition(bodyItem, position);
    }
}
    
    
void EditableSceneBody::Impl::dragForcedPosition(SceneWidgetEvent* event)
{
    if(dragProjector.dragTranslation(event)){
        Isometry3 T;
        T.translation() = dragProjector.position().translation();
        T.linear() = targetLink->R();
        setForcedPosition(T);
        dragged = true;
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


void EditableSceneBody::Impl::startZmpTranslation(SceneWidgetEvent* event)
{
    dragProjector.setInitialTranslation(bodyItem->zmp());
    dragProjector.setTranslationPlaneNormal(Vector3::UnitZ());
    if(dragProjector.startTranslation(event)){
        dragMode = ZMP_TRANSLATION;
    }
}


void EditableSceneBody::Impl::dragZmpTranslation(SceneWidgetEvent* event)
{
    if(dragProjector.dragTranslation(event)){
        Vector3 p = dragProjector.position().translation();
        p.z() = dragProjector.initialPosition().translation().z();
        bodyItem->setZmp(p);
        bodyItem->notifyKinematicStateChange(true);
        dragged = true;
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
                state->write("show_cm", impl->isCmVisible);
                state->write("show_cm_projection", impl->isCmProjectionVisible);
                state->write("show_zmp", impl->isZmpVisible);
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
                impl->showCenterOfMass(state->get({ "show_cm", "showCenterOfMass" }, impl->isCmVisible));
                impl->showCmProjection(state->get({ "show_cm_projection", "showCmProjection" }, impl->isCmProjectionVisible));
                impl->showZmp(state->get({ "show_zmp", "showZmp" }, impl->isZmpVisible));
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
