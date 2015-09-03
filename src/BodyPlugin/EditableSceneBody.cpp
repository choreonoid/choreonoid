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
#include <cnoid/SceneDragger>
#include <cnoid/SceneWidget>
#include <cnoid/SceneDevice>
#include <cnoid/LeggedBodyHelper>
#include <cnoid/PinDragIK>
#include <cnoid/EigenUtil>
#include <cnoid/RootItem>
#include <cnoid/ExtensionManager>
#include <cnoid/Archive>
#include <cnoid/ConnectionSet>
#include <boost/bind.hpp>
#include "gettext.h"

using namespace std;
using namespace cnoid;


EditableSceneLink::EditableSceneLink(Link* link)
    : SceneLink(link)
{
    isPointed_ = false;
    isColliding_ = false;
}


void EditableSceneLink::showBoundingBox(bool on)
{
    if(!visualShape()){
        return;
    }
#if 0
    if(on){
        if(!bbLineSet){
            createBoundingBoxLineSet();
        }
        if(!contains(bbLineSet)){
            addChild(bbLineSet, true);
        }
    } else if(bbLineSet){
        removeChild(bbLineSet, true);
    }
#else
    if(on){
        if(!outlineGroup){
            outlineGroup = new SgOutlineGroup();
        }
        setShapeGroup(outlineGroup);
    } else if(outlineGroup){
        resetShapeGroup();
    }
#endif
}


void EditableSceneLink::createBoundingBoxLineSet()
{
    bbLineSet = new SgLineSet;
    bbLineSet->setName("BoundingBox");

    SgVertexArray& vertices = *bbLineSet->setVertices(new SgVertexArray);
    vertices.resize(8);
    const BoundingBoxf bb(visualShape()->boundingBox());
    const Vector3f& min = bb.min();
    const Vector3f& max = bb.max();
    vertices[0] << min.x(), min.y(), min.z();
    vertices[1] << max.x(), min.y(), min.z();
    vertices[2] << max.x(), max.y(), min.z();
    vertices[3] << min.x(), max.y(), min.z();
    vertices[4] << min.x(), min.y(), max.z();
    vertices[5] << max.x(), min.y(), max.z();
    vertices[6] << max.x(), max.y(), max.z();
    vertices[7] << min.x(), max.y(), max.z();

    bbLineSet->reserveNumLines(12);
    bbLineSet->addLine(0, 1);
    bbLineSet->addLine(1, 2);
    bbLineSet->addLine(2, 3);
    bbLineSet->addLine(3, 0);
    bbLineSet->addLine(4, 5);
    bbLineSet->addLine(5, 6);
    bbLineSet->addLine(6, 7);
    bbLineSet->addLine(7, 4);
    bbLineSet->addLine(0, 4);
    bbLineSet->addLine(1, 5);
    bbLineSet->addLine(2, 6);
    bbLineSet->addLine(3, 7);

    bbLineSet->setColors(new SgColorArray)->push_back(Vector3f(1.0f, 0.0f, 0.0f));
    SgIndexArray& iColors = bbLineSet->colorIndices();
    iColors.resize(24, 0);
}


void EditableSceneLink::showMarker(const Vector3f& color, float transparency)
{
    if(bbMarker){
        removeChild(bbMarker);
    }
    bbMarker = new BoundingBoxMarker(visualShape()->boundingBox(), color, transparency);
    addChild(bbMarker, true);
}


void EditableSceneLink::hideMarker()
{
    if(bbMarker){
        removeChild(bbMarker, true);
        bbMarker = 0;
    }
}


void EditableSceneLink::setColliding(bool on)
{
    if(!isColliding_ && on){
        if(!isPointed_){
            
        }
        isColliding_ = true;
    } else if(isColliding_ && !on){
        if(!isPointed_){
            
        }
        isColliding_ = false;
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
    boost::dynamic_bitset<> collisionLinkBitSet;

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
        
    JointPathPtr ikPath;
    LinkTraverse fkTraverse;
    PinDragIKptr pinDragIK;
    InverseKinematicsPtr ik;
    PenetrationBlockerPtr penetrationBlocker;
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
    static void restorePropertiesLater(const Archive& archive);
    static void restoreProperties(const Archive& archive);
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
      kinematicsBar(KinematicsBar::instance()),
      modified(SgUpdate::MODIFIED)
{
    pointedSceneLink = 0;
    outlinedLink = 0;
    targetLink = 0;

    positionDragger = new PositionDragger;
    positionDragger->setDraggerAlwaysShown(true);
    positionDragger->sigDragStarted().connect(boost::bind(&EditableSceneBodyImpl::onDraggerDragStarted, this));
    positionDragger->sigPositionDragged().connect(boost::bind(&EditableSceneBodyImpl::onDraggerDragged, this));
    positionDragger->sigDragFinished().connect(boost::bind(&EditableSceneBodyImpl::onDraggerDragFinished, this));
    
    dragMode = DRAG_NONE;
    isDragging = false;
    isEditMode = false;

    markerGroup = new SgGroup;
    markerGroup->setName("Marker");
    self->addChild(markerGroup);

    double radius = 0;
    const int n = self->numSceneLinks();
    for(int i=0; i < n; ++i){
        SceneLink* sLink = self->sceneLink(i);
        BoundingBox bb = sLink->boundingBox();
        double radius0 = (bb.max() - bb.center()).norm();
        if(radius0 > radius){
            radius = radius0;
        }
    }
    cmMarker = new CrossMarker(0.25, Vector3f(0.0f, 1.0f, 0.0f), 2.0);
    cmMarker->setName("centerOfMass");
    cmMarker->setSize(radius);
    isCmVisible = false;
    ppcomMarker = new CrossMarker(0.25, Vector3f(1.0f, 0.5f, 0.0f), 2.0);
    ppcomMarker->setName("ProjectionPointCoM");
    ppcomMarker->setSize(radius);
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

    self->sigGraphConnection().connect(boost::bind(&EditableSceneBodyImpl::onSceneGraphConnection, this, _1));
}


double EditableSceneBodyImpl::calcLinkMarkerRadius(SceneLink* sceneLink) const
{
    const BoundingBox& bb = sceneLink->visualShape()->boundingBox();
    double V = ((bb.max().x() - bb.min().x()) * (bb.max().y() - bb.min().y()) * (bb.max().z() - bb.min().z()));
    return pow(V, 1.0 / 3.0) * 0.6;
}


void EditableSceneBodyImpl::onSceneGraphConnection(bool on)
{
    connections.disconnect();

    if(on){
        connections.add(bodyItem->sigUpdated().connect(
                            boost::bind(&EditableSceneBodyImpl::onBodyItemUpdated, this)));
        onBodyItemUpdated();

        connections.add(bodyItem->sigKinematicStateChanged().connect(
                            boost::bind(&EditableSceneBodyImpl::onKinematicStateChanged, this)));
        onKinematicStateChanged();

        connections.add(kinematicsBar->sigCollisionVisualizationChanged().connect(
                            boost::bind(&EditableSceneBodyImpl::onCollisionLinkHighlightModeChanged, this)));
        onCollisionLinkHighlightModeChanged();

        connections.add(bodyItem->sigModelUpdated().connect(
                            boost::bind(&EditableSceneBodyImpl::updateModel, this)));
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
        outlinedLink->showBoundingBox(false);
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
                boost::bind(&EditableSceneBodyImpl::onCollisionsUpdated, this));
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


void EditableSceneBody::setLinkVisibilities(const boost::dynamic_bitset<>& visibilities)
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



void EditableSceneBodyImpl::showCenterOfMass(bool on)
{
    isCmVisible = on;
    if(on){
        markerGroup->addChild(cmMarker, true);
        cmMarker->setTranslation(bodyItem->centerOfMass());
    } else {
        markerGroup->removeChild(cmMarker, true);
    }
}


void EditableSceneBodyImpl::showPpcom(bool on)
{
    isPpcomVisible = on;
    if(on){
        markerGroup->addChild(ppcomMarker, true);
        Vector3 com = bodyItem->centerOfMass();
        com(2) = 0.0;
        ppcomMarker->setTranslation(com);
    } else {
        markerGroup->removeChild(ppcomMarker, true);
    }
}


void EditableSceneBodyImpl::showZmp(bool on)
{
    isZmpVisible = on;
    if(on){
        markerGroup->addChild(zmpMarker, true);
        zmpMarker->setTranslation(bodyItem->zmp());
    } else {
        markerGroup->removeChild(zmpMarker, true);
    }
}


void EditableSceneBodyImpl::makeLinkFree(EditableSceneLink* sceneLink)
{
    if(bodyItem->currentBaseLink() == sceneLink->link()){
        bodyItem->setCurrentBaseLink(0);
    }
    bodyItem->pinDragIK()->setPin(sceneLink->link(), InverseKinematics::NO_AXES);
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
    PinDragIKptr pin = bodyItem->pinDragIK();

    InverseKinematics::AxisSet axes = pin->pinAxes(sceneLink->link());

    if(toggleTranslation && toggleRotation){
        if(axes == InverseKinematics::NO_AXES){
            axes = InverseKinematics::TRANSFORM_6D;
        } else {
            axes = InverseKinematics::NO_AXES;
        }
    } else {
        if(toggleTranslation){
            axes = (InverseKinematics::AxisSet)(axes ^ InverseKinematics::TRANSLATION_3D);
        }
        if(toggleRotation){
            axes = (InverseKinematics::AxisSet)(axes ^ InverseKinematics::ROTATION_3D);
        }
    }
        
    pin->setPin(sceneLink->link(), axes);
    bodyItem->notifyUpdate();
}


void EditableSceneBodyImpl::makeLinkAttitudeLevel()
{
    if(pointedSceneLink){
        Link* targetLink = outlinedLink->link();
        InverseKinematicsPtr ik = bodyItem->getCurrentIK(targetLink);
        if(ik){
            const Position& T = targetLink->T();
            const double theta = acos(T(2, 2));
            const Vector3 z(T(0,2), T(1, 2), T(2, 2));
            const Vector3 axis = z.cross(Vector3::UnitZ()).normalized();
            const Matrix3 R2 = AngleAxisd(theta, axis) * T.linear();

            bodyItem->beginKinematicStateEdit();
            if(ik->calcInverseKinematics(targetLink->p(), R2)){
                bodyItem->notifyKinematicStateChange(true);
                bodyItem->acceptKinematicStateEdit();
            }
        }
    }
}


void EditableSceneBodyImpl::updateMarkersAndManipulators()
{
    bool show = (isEditMode && !self->body()->isStaticModel());
    
    Link* baseLink = bodyItem->currentBaseLink();
    PinDragIKptr pin = bodyItem->pinDragIK();

    const int n = self->numSceneLinks();
    for(int i=0; i < n; ++i){
        EditableSceneLink* sceneLink = editableSceneLink(i);
        sceneLink->hideMarker();
        sceneLink->removeChild(positionDragger);
        markerGroup->removeChild(positionDragger);

        if(show && !activeSimulatorItem){
            Link* link = sceneLink->link();
            if(link == baseLink){
                sceneLink->showMarker(Vector3f(1.0f, 0.1f, 0.1f), 0.4);
            } else {
                int pinAxes = pin->pinAxes(link);
                if(pinAxes & (InverseKinematics::TRANSFORM_6D)){
                    sceneLink->showMarker(Vector3f(1.0f, 1.0f, 0.1f), 0.4);
                }
            }
        }
    }

    bool showDragger = show && targetLink && kinematicsBar->isPositionDraggerEnabled();
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
    if(!bodyItem->isEditable()){
        return false;
    }
        
    if(!outlinedLink || self->body()->isStaticModel()){
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
    if(!bodyItem->isEditable()){
        return false;
    }
    if(self->body()->isStaticModel()){
        return false;
    }
    
    if(outlinedLink){
        outlinedLink->showBoundingBox(false);
        outlinedLink = 0;
    }

    PointedType pointedType = findPointedObject(event.nodePath());
    
    if(pointedSceneLink){
        pointedSceneLink->showBoundingBox(true);
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
                ik.reset();
                
                switch(kinematicsBar->mode()){
                case KinematicsBar::AUTO_MODE:
                    ik = bodyItem->getDefaultIK(targetLink);
                    if(ik){
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
        outlinedLink->showBoundingBox(false);
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
        outlinedLink->showBoundingBox(true);
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
                LinkSelectionView::mainInstance()->makeSingleSelection(
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
    if(!bodyItem->isEditable()){
        return false;
    }
    if(dragMode == DRAG_NONE){
        PointedType pointedType = findPointedObject(event.nodePath());
        if(pointedSceneLink){
            if(pointedSceneLink != outlinedLink){
                if(outlinedLink){
                    outlinedLink->showBoundingBox(false);
                }
                if(!self->body()->isStaticModel()){
                    pointedSceneLink->showBoundingBox(true);
                    outlinedLink = pointedSceneLink;
                }
            }
        }
        if(pointedSceneLink){
            const Vector3 p = pointedSceneLink->T().inverse() * event.point();
            event.updateIndicator(
                (str(boost::format("%1% / %2% : (%3$ .3f, %4$ .3f, %5$ .3f)")
                     % bodyItem->name() % pointedSceneLink->link()->name()
                     % p.x() % p.y() % p.z())));
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
    if(!bodyItem->isEditable()){
        return;
    }
    if(outlinedLink){
        outlinedLink->showBoundingBox(false);
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
    if(!bodyItem->isEditable()){
        return;
    }
    if(self->body()->isStaticModel()){
        return;
    }
    
    PointedType pointedType = findPointedObject(event.nodePath());

    if(bodyItem && pointedType == PT_SCENE_LINK){

        activeSimulatorItem = SimulatorItem::findActiveSimulatorItemFor(bodyItem);
        if(activeSimulatorItem){
            if(pointedSceneLink->link()->isRoot()){
                Action* item1 = menuManager.addCheckItem(_("Move Position"));
                item1->setChecked(forcedPositionMode == MOVE_FORCED_POSITION);
                item1->sigToggled().connect(
                    boost::bind(&EditableSceneBodyImpl::setForcedPositionMode, this, MOVE_FORCED_POSITION, _1));

                Action* item2 = menuManager.addCheckItem(_("Keep Position"));
                item2->setChecked(forcedPositionMode == KEEP_FORCED_POSITION);
                item2->sigToggled().connect(
                    boost::bind(&EditableSceneBodyImpl::setForcedPositionMode, this, KEEP_FORCED_POSITION, _1));
                
                menuManager.addSeparator();
            }
        } else {
            menuManager.addItem(_("Set Free"))->sigTriggered().connect(
                boost::bind(&EditableSceneBodyImpl::makeLinkFree, this, pointedSceneLink));
            menuManager.addItem(_("Set Base"))->sigTriggered().connect(
                boost::bind(&EditableSceneBodyImpl::setBaseLink, this, pointedSceneLink));
            menuManager.addItem(_("Set Translation Pin"))->sigTriggered().connect(
                boost::bind(&EditableSceneBodyImpl::togglePin, this, pointedSceneLink, true, false));
            menuManager.addItem(_("Set Rotation Pin"))->sigTriggered().connect(
                boost::bind(&EditableSceneBodyImpl::togglePin, this, pointedSceneLink, false, true));
            menuManager.addItem(_("Set Both Pins"))->sigTriggered().connect(
                boost::bind(&EditableSceneBodyImpl::togglePin, this, pointedSceneLink, true, true));

            menuManager.addSeparator();

            menuManager.addItem(_("Level Attitude"))->sigTriggered().connect(
                boost::bind(&EditableSceneBodyImpl::makeLinkAttitudeLevel, this));

            menuManager.addSeparator();
        }
        
        menuManager.setPath(_("Markers"));
        
        Action* item = menuManager.addCheckItem(_("Center of Mass"));
        item->setChecked(isCmVisible);
        item->sigToggled().connect(boost::bind(&EditableSceneBodyImpl::showCenterOfMass, this, _1));

        item = menuManager.addCheckItem(_("Projection Point of CoM"));
        item->setChecked(isPpcomVisible);
        item->sigToggled().connect(boost::bind(&EditableSceneBodyImpl::showPpcom, this, _1));

        item = menuManager.addCheckItem(_("ZMP"));
        item->setChecked(isZmpVisible);
        item->sigToggled().connect(boost::bind(&EditableSceneBodyImpl::showZmp, this, _1));

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
    if(!bodyItem->isEditable()){
        return;
    }
    isEditMode = event.sceneWidget()->isEditMode();

    if(isEditMode){
        if(outlinedLink){
            outlinedLink->showBoundingBox(true);
        }
    } else {
        finishEditing();
        if(outlinedLink){
            outlinedLink->showBoundingBox(false);
            outlinedLink = 0;
        }
        updateMarkersAndManipulators();
    }
}


bool EditableSceneBodyImpl::finishEditing()
{
    bool finished = false;
    
    if(bodyItem->isEditable()){

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
    if(!bodyItem->isEditable()){
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
    if(!bodyItem->isEditable()){
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
        doIK(positionDragger->draggedPosition());
    }
}


void EditableSceneBodyImpl::onDraggerDragFinished()
{
    activeSimulatorItem = SimulatorItem::findActiveSimulatorItemFor(bodyItem);
    if(activeSimulatorItem){
        finishForcedPosition();
    } else {
        doIK(positionDragger->draggedPosition());
    }
}


bool EditableSceneBodyImpl::initializeIK()
{
    Link* baseLink = bodyItem->currentBaseLink();

    if(!ik){
        if(bodyItem->pinDragIK()->numPinnedLinks() == 0 && baseLink){
            ikPath = getCustomJointPath(bodyItem->body(), baseLink, targetLink);
            if(ikPath){
                if(!ikPath->hasAnalyticalIK()){
                    ikPath->setBestEffortIKmode(true);
                }
                ik = ikPath;
            }
        }
    }
    if(!ik){
        pinDragIK = bodyItem->pinDragIK();
        pinDragIK->setBaseLink(baseLink);
        pinDragIK->setTargetLink(targetLink, kinematicsBar->isPositionDraggerEnabled());
        if(pinDragIK->initialize()){
            ik = pinDragIK;
        }
    }

    return ik;
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
    if(ik){
        if(ik->calcInverseKinematics(position.translation(), position.linear()) || true /* Best effort */){
            fkTraverse.calcForwardKinematics();
            bodyItem->notifyKinematicStateChange(true);
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
        simulatorItem->setForcedBodyPosition(bodyItem, position);
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
            simulatorItem->clearForcedBodyPositions();
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
        return true;
    }
    return false;
}
    
    
void EditableSceneBodyImpl::restorePropertiesLater(const Archive& archive)
{
    archive.addPostProcess(boost::bind(&EditableSceneBodyImpl::restoreProperties, boost::ref(archive)), 1);
}


void EditableSceneBodyImpl::restoreProperties(const Archive& archive)
{
    Listing& states = *archive["editableSceneBodies"].toListing();
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


void EditableSceneBody::initializeClass(ExtensionManager* ext)
{
    ext->setProjectArchiver(
        "EditableSceneBody",
        EditableSceneBodyImpl::storeProperties,
        EditableSceneBodyImpl::restorePropertiesLater);
}
