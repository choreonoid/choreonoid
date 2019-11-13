/**
   @author Shin'ichiro Nakaoka
*/

#include "EditableSceneBody.h"
#include "BodyItem.h"
#include "SimulatorItem.h"
#include "KinematicsBar.h"
#include "BodyBar.h"
#include "LinkSelectionView.h"
#include <cnoid/JointPath>
#include <cnoid/PenetrationBlocker>
#include <cnoid/MenuManager>
#include <cnoid/SceneWidget>
#include <cnoid/SceneEffects>
#include <cnoid/SceneMarkers>
#include <cnoid/SceneDragProjector>
#include <cnoid/PositionDragger>
#include <cnoid/SceneDevice>
#include <cnoid/LeggedBodyHelper>
#include <cnoid/PinDragIK>
#include <cnoid/EigenUtil>
#include <cnoid/RootItem>
#include <cnoid/ExtensionManager>
#include <cnoid/Archive>
#include <cnoid/ConnectionSet>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace std::placeholders;
using namespace cnoid;

namespace {

Action* linkVisibilityCheck;
Action* enableStaticModelEditCheck;

}

namespace cnoid {

class EditableSceneLinkImpl
{
public:
    EditableSceneLink* self;
    SgOutlineGroupPtr outlineGroup;
    BoundingBoxMarkerPtr bbMarker;
    bool isPointed;
    bool isColliding;

    EditableSceneLinkImpl(EditableSceneLink* self);
};

}


EditableSceneLink::EditableSceneLink(Link* link)
    : SceneLink(link)
{
    impl = new EditableSceneLinkImpl(this);
}


EditableSceneLinkImpl::EditableSceneLinkImpl(EditableSceneLink* self)
    : self(self)
{
    isPointed = false;
    isColliding = false;
}


EditableSceneLink::~EditableSceneLink()
{
    delete impl;
}


void EditableSceneLink::showOutline(bool on)
{
    if(!visualShape()){
        return;
    }
    SgOutlineGroupPtr& outline = impl->outlineGroup;
    if(on){
        if(!outline){
            outline = new SgOutlineGroup;
            outline->setColor(Vector3f(1.0f, 1.0f, 0.0f));
        }
        if(!outline->hasParents()){
            insertEffectGroup(outline);
        }
    } else {
        if(outline && outline->hasParents()){
            removeEffectGroup(outline);
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
        impl->bbMarker = 0;
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


namespace  cnoid {

class EditableSceneBodyImpl
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EditableSceneBodyImpl(EditableSceneBody* self, BodyItemPtr& bodyItem);
    ~EditableSceneBodyImpl();
        
    EditableSceneBody* self;
    BodyItemPtr bodyItem;

    SgUpdate modified;

    ConnectionSet connections;
    Connection connectionToSigCollisionsUpdated;
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
    shared_ptr<InverseKinematics> defaultIK;
    shared_ptr<PinDragIK> pinDragIK;
    shared_ptr<PenetrationBlocker> penetrationBlocker;
    PositionDraggerPtr positionDragger;

    bool isEditMode;

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

    bool isEditable() {
        return bodyItem->isEditable() &&
            (!bodyItem->body()->isStaticModel() || enableStaticModelEditCheck->isChecked());
    }

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
    void onLinkSelectionChanged();

    void showCenterOfMass(bool on);
    void showPpcom(bool on);
    void showZmp(bool on);
    void makeLinkFree(EditableSceneLink* sceneLink);
    void setBaseLink(EditableSceneLink* sceneLink);
    void toggleBaseLink(EditableSceneLink* sceneLink);
    void togglePin(EditableSceneLink* sceneLink, bool toggleTranslation, bool toggleRotation);
    void makeLinkAttitudeLevel();
        
    EditableSceneBodyImpl::PointedType findPointedObject(const vector<SgNode*>& path);
    void updateMarkersAndManipulators();
    void attachPositionDragger(Link* link);

    bool onKeyPressEvent(const SceneWidgetEvent& event);
    bool onKeyReleaseEvent(const SceneWidgetEvent& event);
    bool onButtonPressEvent(const SceneWidgetEvent& event);
    bool onDoubleClickEvent(const SceneWidgetEvent& event);
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


namespace {
SceneLink* createEditableSceneLink(Link* link)
{
    return new EditableSceneLink(link);
}
}
    
    
EditableSceneBody::EditableSceneBody(BodyItemPtr bodyItem)
    : SceneBody(bodyItem->body(), createEditableSceneLink)
{
    setName(body()->name());
    impl = new EditableSceneBodyImpl(this, bodyItem);
}


EditableSceneBodyImpl::EditableSceneBodyImpl(EditableSceneBody* self, BodyItemPtr& bodyItem)
    : self(self),
      bodyItem(bodyItem),
      modified(SgUpdate::MODIFIED),
      kinematicsBar(KinematicsBar::instance())
{
    pointedSceneLink = 0;
    outlinedLink = 0;
    targetLink = 0;

    positionDragger = new PositionDragger;
    positionDragger->setDraggerAlwaysShown(true);
    positionDragger->sigDragStarted().connect(std::bind(&EditableSceneBodyImpl::onDraggerDragStarted, this));
    positionDragger->sigPositionDragged().connect(std::bind(&EditableSceneBodyImpl::onDraggerDragged, this));
    positionDragger->sigDragFinished().connect(std::bind(&EditableSceneBodyImpl::onDraggerDragFinished, this));
    
    dragMode = DRAG_NONE;
    isDragging = false;
    isEditMode = false;

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

    self->sigGraphConnection().connect(std::bind(&EditableSceneBodyImpl::onSceneGraphConnection, this, _1));
}


double EditableSceneBodyImpl::calcLinkMarkerRadius(SceneLink* sceneLink) const
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


void EditableSceneBodyImpl::onSceneGraphConnection(bool on)
{
    connections.disconnect();
    connectionToSigLinkSelectionChanged.disconnect();

    if(on){
        connections.add(bodyItem->sigUpdated().connect(
                            std::bind(&EditableSceneBodyImpl::onBodyItemUpdated, this)));
        onBodyItemUpdated();

        connections.add(bodyItem->sigKinematicStateChanged().connect(
                            std::bind(&EditableSceneBodyImpl::onKinematicStateChanged, this)));
        onKinematicStateChanged();

        connections.add(kinematicsBar->sigCollisionVisualizationChanged().connect(
                            std::bind(&EditableSceneBodyImpl::onCollisionLinkHighlightModeChanged, this)));
        onCollisionLinkHighlightModeChanged();

        connections.add(bodyItem->sigModelUpdated().connect(
                            std::bind(&EditableSceneBodyImpl::updateModel, this)));

        connections.add(
            linkVisibilityCheck->sigToggled().connect(
                std::bind(&EditableSceneBodyImpl::onLinkVisibilityCheckToggled, this)));
        onLinkVisibilityCheckToggled();
    }
}


void EditableSceneBody::updateModel()
{
    impl->updateModel();
}


void EditableSceneBodyImpl::updateModel()
{
    pointedSceneLink = 0;
    targetLink = 0;
    if(outlinedLink){
        outlinedLink->showOutline(false);
        outlinedLink = 0;
    }
    isDragging = false;
    dragMode = DRAG_NONE;
    
    self->SceneBody::updateModel();
}


void EditableSceneBodyImpl::onBodyItemUpdated()
{
    updateMarkersAndManipulators();
}


void EditableSceneBodyImpl::onKinematicStateChanged()
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


void EditableSceneBodyImpl::onCollisionsUpdated()
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


void EditableSceneBodyImpl::onCollisionLinkHighlightModeChanged()
{
    changeCollisionLinkHighlightMode(kinematicsBar->isCollisionLinkHighlihtMode());
}


void EditableSceneBodyImpl::changeCollisionLinkHighlightMode(bool on)
{
    if(!connectionToSigCollisionsUpdated.connected() && on){
        connectionToSigCollisionsUpdated =
            bodyItem->sigCollisionsUpdated().connect(
                std::bind(&EditableSceneBodyImpl::onCollisionsUpdated, this));
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


EditableSceneBodyImpl::~EditableSceneBodyImpl()
{
    connectionToSigCollisionsUpdated.disconnect();
    connections.disconnect();
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


void EditableSceneBodyImpl::onLinkVisibilityCheckToggled()
{
    LinkSelectionView* selectionView = LinkSelectionView::instance();

    if(linkVisibilityCheck->isChecked()){
        connectionToSigLinkSelectionChanged.reset(
            selectionView->sigSelectionChanged(bodyItem).connect(
                std::bind(&EditableSceneBodyImpl::onLinkSelectionChanged, this)));
        onLinkSelectionChanged();
    } else {
        connectionToSigLinkSelectionChanged.disconnect();
        vector<bool> visibilities(self->numSceneLinks(), true);
        self->setLinkVisibilities(visibilities);
    }
}


void EditableSceneBodyImpl::onLinkSelectionChanged()
{
    if(linkVisibilityCheck->isChecked()){
        self->setLinkVisibilities(LinkSelectionView::instance()->linkSelection(bodyItem));
    }
}


void EditableSceneBodyImpl::showCenterOfMass(bool on)
{
    isCmVisible = on;
    if(on){
        cmMarker->setTranslation(bodyItem->centerOfMass());
        markerGroup->addChildOnce(cmMarker, true);
    } else {
        markerGroup->removeChild(cmMarker, true);
    }
}


void EditableSceneBodyImpl::showPpcom(bool on)
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


void EditableSceneBodyImpl::showZmp(bool on)
{
    isZmpVisible = on;
    if(on){
        zmpMarker->setTranslation(bodyItem->zmp());
        markerGroup->addChildOnce(zmpMarker, true);
    } else {
        markerGroup->removeChild(zmpMarker, true);
    }
}


void EditableSceneBodyImpl::makeLinkFree(EditableSceneLink* sceneLink)
{
    if(bodyItem->currentBaseLink() == sceneLink->link()){
        bodyItem->setCurrentBaseLink(0);
    }
    bodyItem->pinDragIK()->setPin(sceneLink->link(), PinDragIK::NO_AXES);
    bodyItem->notifyUpdate();
}


void EditableSceneBodyImpl::setBaseLink(EditableSceneLink* sceneLink)
{
    bodyItem->setCurrentBaseLink(sceneLink->link());
    bodyItem->notifyUpdate();
}
    

void EditableSceneBodyImpl::toggleBaseLink(EditableSceneLink* sceneLink)
{
    Link* baseLink = bodyItem->currentBaseLink();
    if(sceneLink->link() != baseLink){
        bodyItem->setCurrentBaseLink(sceneLink->link());
    } else {
        bodyItem->setCurrentBaseLink(0);
    }
    bodyItem->notifyUpdate();
}


void EditableSceneBodyImpl::togglePin(EditableSceneLink* sceneLink, bool toggleTranslation, bool toggleRotation)
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


void EditableSceneBodyImpl::makeLinkAttitudeLevel()
{
    if(pointedSceneLink){
        Link* targetLink = outlinedLink->link();
        auto ik = bodyItem->getCurrentIK(targetLink);
        if(ik){
            const Position& T = targetLink->T();
            const double theta = acos(T(2, 2));
            const Vector3 z(T(0,2), T(1, 2), T(2, 2));
            const Vector3 axis = z.cross(Vector3::UnitZ()).normalized();
            const Matrix3 R2 = AngleAxisd(theta, axis) * T.linear();
            Position T2;
            T2.linear() = R2;
            T2.translation() = targetLink->p();

            bodyItem->beginKinematicStateEdit();
            if(ik->calcInverseKinematics(T2)){
                bool fkDone = ik->calcRemainingPartForwardKinematicsForInverseKinematics();
                bodyItem->notifyKinematicStateChange(!fkDone);
                bodyItem->acceptKinematicStateEdit();
            }
        }
    }
}


void EditableSceneBodyImpl::updateMarkersAndManipulators()
{
    Link* baseLink = bodyItem->currentBaseLink();
    auto pin = bodyItem->pinDragIK();

    const int n = self->numSceneLinks();
    for(int i=0; i < n; ++i){
        EditableSceneLink* sceneLink = editableSceneLink(i);
        sceneLink->hideMarker();
        sceneLink->removeChild(positionDragger);
        markerGroup->removeChild(positionDragger);

        if(isEditMode && !activeSimulatorItem){
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
    }

    bool showDragger = isEditMode && targetLink && kinematicsBar->isPositionDraggerEnabled();
    if(showDragger){
        if(activeSimulatorItem){
            showDragger = forcedPositionMode != NO_FORCED_POSITION;
                } else {
            showDragger = (kinematicsBar->mode() == KinematicsBar::IK_MODE);
        }
    }
        
    if(showDragger){
        attachPositionDragger(targetLink);
    }

    self->notifyUpdate(modified);
}


void EditableSceneBodyImpl::attachPositionDragger(Link* link)
{
    SceneLink* sceneLink = self->sceneLink(link->index());
    positionDragger->adjustSize(sceneLink->untransformedBoundingBox());
    sceneLink->addChild(positionDragger);
}


EditableSceneBodyImpl::PointedType EditableSceneBodyImpl::findPointedObject(const vector<SgNode*>& path)
{
    PointedType pointedType = PT_NONE;
    pointedSceneLink = 0;
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


bool EditableSceneBody::onKeyPressEvent(const SceneWidgetEvent& event)
{
    return impl->onKeyPressEvent(event);
}


bool EditableSceneBodyImpl::onKeyPressEvent(const SceneWidgetEvent& event)
{
    if(!isEditable()){
        return false;
    }
        
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


bool EditableSceneBodyImpl::onKeyReleaseEvent(const SceneWidgetEvent& event)
{
    return false;
}


bool EditableSceneBody::onButtonPressEvent(const SceneWidgetEvent& event)
{
    return impl->onButtonPressEvent(event);
}


bool EditableSceneBodyImpl::onButtonPressEvent(const SceneWidgetEvent& event)
{
    if(!isEditable()){
        return false;
    }
    
    if(outlinedLink){
        outlinedLink->showOutline(false);
        outlinedLink = 0;
    }

    PointedType pointedType = findPointedObject(event.nodePath());
    
    if(pointedSceneLink){
        pointedSceneLink->showOutline(true);
        outlinedLink = pointedSceneLink;
    }
    
    bool handled = false;
    isDragging = false;

    activeSimulatorItem = SimulatorItem::findActiveSimulatorItemFor(bodyItem);
    if(activeSimulatorItem){
        if(pointedType == PT_SCENE_LINK){
            targetLink = pointedSceneLink->link();
            if(event.button() == Qt::LeftButton){
                if(targetLink->isRoot() && (forcedPositionMode != NO_FORCED_POSITION)){
                    startForcedPosition(event);
                } else {
                    startVirtualElasticString(event);
                }
            }
            handled = true;
        }
    } else {
        if(pointedType == PT_SCENE_LINK){
            if(event.button() == Qt::LeftButton){
                targetLink = pointedSceneLink->link();
                updateMarkersAndManipulators();
                currentIK.reset();
                defaultIK.reset();
                
                switch(kinematicsBar->mode()){

                case KinematicsBar::AUTO_MODE:
                    defaultIK = bodyItem->getDefaultIK(targetLink);
                    if(defaultIK){
                        startIK(event);
                        break;
                    }
                    
                case KinematicsBar::FK_MODE:
                    if(targetLink == bodyItem->currentBaseLink()){
                        // Translation of the base link
                        startIK(event);
                    } else {
                        startFK(event);
                    }
                    break;

                case KinematicsBar::IK_MODE:
                    startIK(event);
                    break;
                }
            } else if(event.button() == Qt::MiddleButton){
                togglePin(pointedSceneLink, true, true);
                
            } else if(event.button() == Qt::RightButton){
                
            }
            
            handled = true;

        } else if(pointedType == PT_ZMP){
            startZmpTranslation(event);
            handled = true;
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


bool EditableSceneBodyImpl::onButtonReleaseEvent(const SceneWidgetEvent& event)
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
    return impl->onDoubleClickEvent(event);
}


bool EditableSceneBodyImpl::onDoubleClickEvent(const SceneWidgetEvent& event)
{
    if(findPointedObject(event.nodePath()) == PT_SCENE_LINK){
        if(event.button() == Qt::LeftButton){
            if(BodyBar::instance()->makeSingleSelection(bodyItem)){
                LinkSelectionView::instance()->makeSingleSelection(
                    bodyItem, pointedSceneLink->link()->index());
            }
            return true;
        }
    }
    return false;
}


bool EditableSceneBody::onPointerMoveEvent(const SceneWidgetEvent& event)
{
    return impl->onPointerMoveEvent(event);
}


bool EditableSceneBodyImpl::onPointerMoveEvent(const SceneWidgetEvent& event)
{
    if(!isEditable()){
        return false;
    }
    if(dragMode == DRAG_NONE){
        findPointedObject(event.nodePath());
        if(pointedSceneLink){
            if(pointedSceneLink != outlinedLink){
                if(outlinedLink){
                    outlinedLink->showOutline(false);
                }
                pointedSceneLink->showOutline(true);
                outlinedLink = pointedSceneLink;
            }
        }
        if(pointedSceneLink){
            const Vector3 p = pointedSceneLink->T().inverse() * event.point();
            event.updateIndicator(
                fmt::format("{0} / {1} : ({2:.3f}, {3:.3f}, {4:.3f})",
                            bodyItem->name(), pointedSceneLink->link()->name(),
                            p.x(), p.y(), p.z()));
        } else {
            event.updateIndicator("");
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


void EditableSceneBodyImpl::onPointerLeaveEvent(const SceneWidgetEvent& event)
{
    if(!isEditable()){
        return;
    }
    if(outlinedLink){
        outlinedLink->showOutline(false);
        outlinedLink = 0;
    }
}


bool EditableSceneBody::onScrollEvent(const SceneWidgetEvent& event)
{
    return impl->onScrollEvent(event);
}


bool EditableSceneBodyImpl::onScrollEvent(const SceneWidgetEvent& event)
{
    return false;
}


void EditableSceneBody::onContextMenuRequest(const SceneWidgetEvent& event, MenuManager& menuManager)
{
    impl->onContextMenuRequest(event, menuManager);
}


void EditableSceneBodyImpl::onContextMenuRequest(const SceneWidgetEvent& event, MenuManager& menuManager)
{
    if(!isEditable()){
        return;
    }
    
    PointedType pointedType = findPointedObject(event.nodePath());

    if(bodyItem && pointedType == PT_SCENE_LINK){

        activeSimulatorItem = SimulatorItem::findActiveSimulatorItemFor(bodyItem);
        if(activeSimulatorItem){
            if(pointedSceneLink->link()->isRoot()){
                Action* item1 = menuManager.addCheckItem(_("Move Forcibly"));
                item1->setChecked(forcedPositionMode == MOVE_FORCED_POSITION);
                item1->sigToggled().connect(
                    std::bind(&EditableSceneBodyImpl::setForcedPositionMode, this, MOVE_FORCED_POSITION, _1));

                Action* item2 = menuManager.addCheckItem(_("Hold Forcibly"));
                item2->setChecked(forcedPositionMode == KEEP_FORCED_POSITION);
                item2->sigToggled().connect(
                    std::bind(&EditableSceneBodyImpl::setForcedPositionMode, this, KEEP_FORCED_POSITION, _1));
                
                menuManager.addSeparator();
            }
        } else {
            menuManager.addItem(_("Set Free"))->sigTriggered().connect(
                std::bind(&EditableSceneBodyImpl::makeLinkFree, this, pointedSceneLink));
            menuManager.addItem(_("Set Base"))->sigTriggered().connect(
                std::bind(&EditableSceneBodyImpl::setBaseLink, this, pointedSceneLink));
            menuManager.addItem(_("Set Translation Pin"))->sigTriggered().connect(
                std::bind(&EditableSceneBodyImpl::togglePin, this, pointedSceneLink, true, false));
            menuManager.addItem(_("Set Rotation Pin"))->sigTriggered().connect(
                std::bind(&EditableSceneBodyImpl::togglePin, this, pointedSceneLink, false, true));
            menuManager.addItem(_("Set Both Pins"))->sigTriggered().connect(
                std::bind(&EditableSceneBodyImpl::togglePin, this, pointedSceneLink, true, true));

            menuManager.addSeparator();

            menuManager.addItem(_("Level Attitude"))->sigTriggered().connect(
                std::bind(&EditableSceneBodyImpl::makeLinkAttitudeLevel, this));

            menuManager.addSeparator();
        }
        
        menuManager.setPath(_("Markers"));
        
        Action* item = menuManager.addCheckItem(_("Center of Mass"));
        item->setChecked(isCmVisible);
        item->sigToggled().connect(std::bind(&EditableSceneBodyImpl::showCenterOfMass, this, _1));

        item = menuManager.addCheckItem(_("Projection Point of CoM"));
        item->setChecked(isPpcomVisible);
        item->sigToggled().connect(std::bind(&EditableSceneBodyImpl::showPpcom, this, _1));

        item = menuManager.addCheckItem(_("ZMP"));
        item->setChecked(isZmpVisible);
        item->sigToggled().connect(std::bind(&EditableSceneBodyImpl::showZmp, this, _1));

        menuManager.setPath("/");
        menuManager.addSeparator();
    }
}


void EditableSceneBody::onSceneModeChanged(const SceneWidgetEvent& event)
{
    impl->onSceneModeChanged(event);
}


void EditableSceneBodyImpl::onSceneModeChanged(const SceneWidgetEvent& event)
{
    if(!isEditable()){
        isEditMode = false;
        return;
    }
    isEditMode = event.sceneWidget()->isEditMode();

    if(isEditMode){
        if(outlinedLink){
            outlinedLink->showOutline(true);
        }
    } else {
        finishEditing();
        if(outlinedLink){
            outlinedLink->showOutline(false);
            outlinedLink = 0;
        }
        updateMarkersAndManipulators();
    }
}


bool EditableSceneBodyImpl::finishEditing()
{
    bool finished = false;
    
    if(isEditable()){

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
    }

    return finished;
}


bool EditableSceneBody::onUndoRequest()
{
    return impl->onUndoRequest();
}


bool EditableSceneBodyImpl::onUndoRequest()
{
    if(!isEditable()){
        return false;
    }
    return bodyItem->undoKinematicState();
}


bool EditableSceneBody::onRedoRequest()
{
    return impl->onRedoRequest();
}


bool EditableSceneBodyImpl::onRedoRequest()
{
    if(!isEditable()){
        return false;
    }
    return bodyItem->redoKinematicState();
}


void EditableSceneBodyImpl::onDraggerDragStarted()
{
    activeSimulatorItem = SimulatorItem::findActiveSimulatorItemFor(bodyItem);
    if(!activeSimulatorItem){
        initializeIK();
    }
}


void EditableSceneBodyImpl::onDraggerDragged()
{
    activeSimulatorItem = SimulatorItem::findActiveSimulatorItemFor(bodyItem);
    if(activeSimulatorItem){
        setForcedPosition(positionDragger->draggedPosition());
    } else {
        Affine3 T = positionDragger->draggedPosition();
        T.linear() = targetLink->calcRfromAttitude(T.linear());
        doIK(T);
    }
}


void EditableSceneBodyImpl::onDraggerDragFinished()
{
    activeSimulatorItem = SimulatorItem::findActiveSimulatorItemFor(bodyItem);
    if(activeSimulatorItem){
        finishForcedPosition();
    } else {
        Affine3 T = positionDragger->draggedPosition();
        T.linear() = targetLink->calcRfromAttitude(T.linear());
        doIK(T);
    }
}


bool EditableSceneBodyImpl::initializeIK()
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
        if(defaultIK){
            currentIK = defaultIK;
        } else {
            currentIK = bodyItem->getCurrentIK(targetLink);
        }
    }
    if(auto jointPath = dynamic_pointer_cast<JointPath>(currentIK)){
        if(!jointPath->hasAnalyticalIK()){
            jointPath->setBestEffortIKmode(true);
        }
    }

    return currentIK? true: false;
}


void EditableSceneBodyImpl::startIK(const SceneWidgetEvent& event)
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


void EditableSceneBodyImpl::dragIK(const SceneWidgetEvent& event)
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


void EditableSceneBodyImpl::doIK(const Position& position)
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


void EditableSceneBodyImpl::startFK(const SceneWidgetEvent& event)
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


void EditableSceneBodyImpl::dragFKRotation(const SceneWidgetEvent& event)
{
    if(dragProjector.dragRotation(event)){
        targetLink->q() = orgJointPosition + dragProjector.rotationAngle();
        bodyItem->notifyKinematicStateChange(true);
    }
}


void EditableSceneBodyImpl::dragFKTranslation(const SceneWidgetEvent& event)
{
    if(dragProjector.dragTranslation(event)){
        targetLink->q() = orgJointPosition + dragProjector.translationAxis().dot(dragProjector.translation());
        bodyItem->notifyKinematicStateChange(true);
    }
}


void EditableSceneBodyImpl::setForcedPositionMode(int mode, bool on)
{
    if(on){
        forcedPositionMode = mode;
    } else {
        forcedPositionMode = NO_FORCED_POSITION;
        updateMarkersAndManipulators();
    }
    finishForcedPosition();
}


void EditableSceneBodyImpl::startVirtualElasticString(const SceneWidgetEvent& event)
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


void EditableSceneBodyImpl::dragVirtualElasticString(const SceneWidgetEvent& event)
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


void EditableSceneBodyImpl::finishVirtualElasticString()
{
    if(SimulatorItem* simulatorItem = activeSimulatorItem.lock()){
        simulatorItem->clearVirtualElasticStrings();
    }
    markerGroup->removeChild(virtualElasticStringLine, true);
}


void EditableSceneBodyImpl::startForcedPosition(const SceneWidgetEvent& event)
{
    finishForcedPosition();
    updateMarkersAndManipulators();

    dragProjector.setInitialPosition(targetLink->position());
    dragProjector.setTranslationAlongViewPlane();
    if(dragProjector.startTranslation(event)){
        dragMode = LINK_FORCED_POSITION;
    }
}


void EditableSceneBodyImpl::setForcedPosition(const Position& position)
{
    if(SimulatorItem* simulatorItem = activeSimulatorItem.lock()){
        simulatorItem->setForcedPosition(bodyItem, position);
    }
}
    
    
void EditableSceneBodyImpl::dragForcedPosition(const SceneWidgetEvent& event)
{
    if(dragProjector.dragTranslation(event)){
        Position T;
        T.translation() = dragProjector.position().translation();
        T.linear() = targetLink->R();
        setForcedPosition(T);
    }
}


void EditableSceneBodyImpl::finishForcedPosition()
{
    if(forcedPositionMode != KEEP_FORCED_POSITION){
        if(SimulatorItem* simulatorItem = activeSimulatorItem.lock()){
            simulatorItem->clearForcedPositions();
        }
    }
}


void EditableSceneBodyImpl::startZmpTranslation(const SceneWidgetEvent& event)
{
    dragProjector.setInitialTranslation(bodyItem->zmp());
    dragProjector.setTranslationPlaneNormal(Vector3::UnitZ());
    if(dragProjector.startTranslation(event)){
        dragMode = ZMP_TRANSLATION;
    }
}


void EditableSceneBodyImpl::dragZmpTranslation(const SceneWidgetEvent& event)
{
    if(dragProjector.dragTranslation(event)){
        Vector3 p = dragProjector.position().translation();
        p.z() = dragProjector.initialPosition().translation().z();
        bodyItem->setZmp(p);
        bodyItem->notifyKinematicStateChange(true);
    }
}


bool EditableSceneBodyImpl::storeProperties(Archive& archive)
{
    ListingPtr states = new Listing();

    ItemList<BodyItem> bodyItems;
    bodyItems.extractChildItems(RootItem::instance());
    
    for(size_t i=0; i < bodyItems.size(); ++i){
        BodyItem* bodyItem = bodyItems[i];
        EditableSceneBody* sceneBody = bodyItem->existingSceneBody();
        if(sceneBody){
            ValueNodePtr id = archive.getItemId(bodyItem);
            if(id){
                EditableSceneBodyImpl* impl = sceneBody->impl;
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

    archive.write("staticModelEditing", enableStaticModelEditCheck->isChecked());
    
    return true;
}
    
    
void EditableSceneBodyImpl::restoreProperties(const Archive& archive)
{
    enableStaticModelEditCheck->setChecked(archive.get("staticModelEditing", false));
    archive.addPostProcess(std::bind(&EditableSceneBodyImpl::restoreSceneBodyProperties, std::ref(archive)), 1);
}


void EditableSceneBodyImpl::restoreSceneBodyProperties(const Archive& archive)
{
    Listing& states = *archive.findListing("editableSceneBodies");
    if(states.isValid()){
        for(int i=0; i < states.size(); ++i){
            Mapping* state = states[i].toMapping();
            BodyItem* bodyItem = archive.findItem<BodyItem>(state->find("bodyItem"));
            if(bodyItem){
                EditableSceneBodyImpl* impl = bodyItem->sceneBody()->impl;
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
    enableStaticModelEditCheck = mm.addCheckItem(_("Enable editing static models"));
    enableStaticModelEditCheck->setChecked(true);

    ext->setProjectArchiver(
        "EditableSceneBody",
        EditableSceneBodyImpl::storeProperties,
        EditableSceneBodyImpl::restoreProperties);
}
