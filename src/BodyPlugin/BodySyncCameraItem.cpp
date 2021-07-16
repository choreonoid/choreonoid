/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "BodySyncCameraItem.h"
#include "BodySyncCameraConfigDialog.h"
#include <cnoid/ItemManager>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Archive>
#include <cnoid/InteractiveCameraTransform>
#include <cnoid/SceneCameras>
#include <cnoid/SceneView>
#include <cnoid/SceneWidget>
#include <cnoid/BodyItem>
#include <cnoid/SceneRenderer>
#include <cnoid/SceneBar>
#include <cnoid/EigenUtil>
#include <cnoid/EigenArchive>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

class BodySyncCameraTransform : public InteractiveCameraTransform
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    BodyItem* bodyItem;
    string targetLinkName;
    LinkPtr targetLink;
    ScopedConnection bodyItemConnection;
    Vector3 relativeTranslationFromBody;
    Isometry3 relativePositionFromBody;
    bool isSigUpdatedEmittedBySelf;
    bool isParallelTrackingMode_;

    BodySyncCameraTransform() {
        bodyItem = nullptr;
        relativeTranslationFromBody.setZero();
        relativePositionFromBody.setIdentity();
        isSigUpdatedEmittedBySelf = false;
        isParallelTrackingMode_ = false;

        initialize();
    }
    
    BodySyncCameraTransform(const BodySyncCameraTransform& org, CloneMap* cloneMap)
        : InteractiveCameraTransform(org, cloneMap) {
        bodyItem = nullptr;
        relativeTranslationFromBody.setZero();
        relativePositionFromBody.setIdentity();
        isSigUpdatedEmittedBySelf = false;
        isParallelTrackingMode_ = org.isParallelTrackingMode_;

        initialize();
    }

    void initialize()
    {
        sigUpdated().connect(
            [this](const SgUpdate& update){
                if(isSigUpdatedEmittedBySelf){
                    isSigUpdatedEmittedBySelf = false;
                } else {
                    updateRelativePosition();
                }
            });
    }

    virtual Referenced* doClone(CloneMap* cloneMap) const override {
        return new BodySyncCameraTransform(*this, cloneMap);
    }

    bool isParallelTrackingMode() const {
        return isParallelTrackingMode_;
    }

    void setParallelTrackingMode(bool on){
        isParallelTrackingMode_ = on;
    }
            
    void setBodyItem(BodyItem* bodyItem) {
        if(bodyItem != this->bodyItem){
            this->bodyItem = bodyItem;
            updateTargetLink();
            bodyItemConnection.disconnect();
            if(bodyItem){
                bodyItemConnection.reset(
                    bodyItem->sigKinematicStateChanged().connect( [&](){ onBodyMoved(); } ));
                updateRelativePosition();
            }
        }
    }

    void setTargetLink(const string& name){
        targetLinkName = name;
        updateTargetLink();
    }

    void updateTargetLink(){
        targetLink = nullptr;
        if(bodyItem){
            if(!targetLinkName.empty()){
                targetLink = bodyItem->body()->link(targetLinkName);
            }
            if(!targetLink){
                targetLink = bodyItem->body()->rootLink();
            }
        }
    }

    void updateRelativePosition(){
        if(bodyItem){
            relativeTranslationFromBody = translation() - targetLink->translation();
            relativePositionFromBody = targetLink->position().inverse() * position();
        }
    }

    void onBodyMoved(){
        if(bodyItem){
            if(isParallelTrackingMode_){
                setTranslation(targetLink->translation() + relativeTranslationFromBody);
            } else {
                setPosition(targetLink->position() * relativePositionFromBody);
            }
            isSigUpdatedEmittedBySelf = true;

            // TODO: Use SgUpdate of BodySyncCameraItem::Impl
            notifyUpdate();
        }
    }
};
  
typedef ref_ptr<BodySyncCameraTransform> BodySyncCameraTransformPtr;

}


namespace cnoid {

class BodySyncCameraItem::Impl
{
public:
    BodySyncCameraItem* self;
    BodySyncCameraTransformPtr cameraTransform;
    SgPerspectiveCameraPtr persCamera;
    SgOrthographicCameraPtr orthoCamera;
    SgCameraPtr currentCamera;
    SgUpdate update;
    Selection cameraType;
    
    Impl(BodySyncCameraItem* self, bool initCameraPosition);
    Impl(BodySyncCameraItem* self, const Impl& org);
    void doPutProperties(PutPropertyFunction& putProperty);
    bool setClipDistances(double nearDistance, double farDistance);
    bool setFieldOfView(double fov);
    bool setCameraType(int index);
};

}


void BodySyncCameraItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager()
        .registerClass<BodySyncCameraItem>(N_("BodySyncCameraItem"))
        .addAlias<BodySyncCameraItem>("BodyTrackingCameraItem", "Body")
        .addCreationPanel<BodySyncCameraItem>();
}


BodySyncCameraItem* BodySyncCameraItem::createBodySyncCameraItemWithDialog
(BodyItem* bodyItem, Link* link)
{
    return BodySyncCameraConfigDialog::instance()->createCameraItem(bodyItem, link);
}


void BodySyncCameraItem::configureCameraItemWithDialog(BodySyncCameraItem* cameraItem)
{
    BodySyncCameraConfigDialog::instance()->configureCameraItem(cameraItem);
}


BodySyncCameraItem::BodySyncCameraItem()
    : Item("BodySyncCamera")
{
    impl = new Impl(this, true);
}


BodySyncCameraItem::BodySyncCameraItem(const BodySyncCameraItem& org)
    : Item(org)
{
    impl = new Impl(this, *org.impl);
}


BodySyncCameraItem::Impl::Impl(BodySyncCameraItem* self, bool initCameraPosition)
    : self(self),
      cameraType(NumCameraTypes, CNOID_GETTEXT_DOMAIN_NAME)
{
    cameraType.setSymbol(Perspective,  N_("Perspective"));
    cameraType.setSymbol(Orthographic, N_("Orthographic"));
    cameraType.select(Perspective);

    cameraTransform = new BodySyncCameraTransform;
    if(initCameraPosition){
        cameraTransform->setPosition(
            SceneView::instance()->sceneWidget()->builtinCameraTransform()->position());
    } else {
        cameraTransform->setInteractiveViewpointChangeLocked(true);
    }

    persCamera = new SgPerspectiveCamera;
    persCamera->setName(self->name());
    cameraTransform->addChild(persCamera);
    currentCamera = persCamera;

    orthoCamera = new SgOrthographicCamera;
    orthoCamera->setName(self->name());
}


BodySyncCameraItem::Impl::Impl(BodySyncCameraItem* self, const Impl& org)
    : Impl(self, false)
{
    cameraType = org.cameraType;
    cameraTransform->setPosition(org.cameraTransform->position());
    cameraTransform->setInteractiveViewpointChangeLocked(
        org.cameraTransform->isInteractiveViewpointChangeLocked());
    cameraTransform->targetLinkName = org.cameraTransform->targetLinkName;
}


BodySyncCameraItem::~BodySyncCameraItem()
{
    delete impl;
}


bool BodySyncCameraItem::setName(const std::string& name_)
{
    if(name_ != name()){
        impl->persCamera->setName(name_);
        impl->orthoCamera->setName(name_);
        Item::setName(name_);
        impl->persCamera->notifyUpdate(impl->update);
        impl->orthoCamera->notifyUpdate(impl->update);
    }
    return true;
}


Item* BodySyncCameraItem::doDuplicate() const
{
    return new BodySyncCameraItem(*this);
}


void BodySyncCameraItem::setTargetLink(const std::string& name)
{
    impl->cameraTransform->setTargetLink(name);
}


const std::string& BodySyncCameraItem::targetLinkName() const
{
    return impl->cameraTransform->targetLinkName;
}


Isometry3 BodySyncCameraItem::targetLinkPosition() const
{
    if(auto link = impl->cameraTransform->targetLink){
        return link->position();
    }
    return Isometry3::Identity();
}


void BodySyncCameraItem::setParallelTrackingMode(bool on)
{
    impl->cameraTransform->setParallelTrackingMode(on);
}


bool BodySyncCameraItem::isParallelTrackingMode() const
{
    return impl->cameraTransform->isParallelTrackingMode();
}


void BodySyncCameraItem::setInteractiveViewpointChangeEnabled(bool on)
{
    impl->cameraTransform->setInteractiveViewpointChangeLocked(!on);
}


bool BodySyncCameraItem::isInteractiveViewpointChangeEnabled() const
{
    return !impl->cameraTransform->isInteractiveViewpointChangeLocked();
}


void BodySyncCameraItem::setCameraType(CameraType type)
{
    impl->setCameraType(type);
}


/**
   \todo Improve the scene widget so that the current camera path described in a string list
   can be kept even if the actual camera node is changed, and simplify the following implementation.
*/
bool BodySyncCameraItem::Impl::setCameraType(int index)
{
    if(cameraType.selectedIndex() == index){
        return true;
    }

    cameraType.select(index);

    SgCamera* cameraToRemove;
    if(cameraType.is(BodySyncCameraItem::Perspective)){
        currentCamera = persCamera;
        cameraToRemove = orthoCamera;
    }else if(cameraType.is(BodySyncCameraItem::Orthographic)){
        currentCamera = orthoCamera;
        cameraToRemove = persCamera;
    }

    vector<SceneRenderer*> renderers;
    for(auto sceneView : SceneView::instances()){
        renderers.push_back(sceneView->sceneWidget()->renderer());
    }

    cameraTransform->addChild(currentCamera, update);
    for(auto renderer : renderers){
        if(renderer->currentCamera() == cameraToRemove){
            renderer->extractPreprocessedNodes();
            renderer->setCurrentCamera(currentCamera);
        }
    }
    cameraTransform->removeChild(cameraToRemove, update);
    for(auto renderer : renderers){
        renderer->extractPreprocessedNodes();
    }

    return true;
}


BodySyncCameraItem::CameraType BodySyncCameraItem::cameraType() const
{
    return static_cast<CameraType>(impl->cameraType.which());
}


SgPerspectiveCamera* BodySyncCameraItem::perspectiveCamera()
{
    return impl->persCamera;
}


SgOrthographicCamera* BodySyncCameraItem::orthographicCamera()
{
    return impl->orthoCamera;
}


SgCamera* BodySyncCameraItem::currentCamera()
{
    return impl->currentCamera;
}


SgPosTransform* BodySyncCameraItem::cameraTransform()
{
    return impl->cameraTransform;
}


Isometry3 BodySyncCameraItem::relativeCameraPosition() const
{
    return impl->cameraTransform->relativePositionFromBody;
}


SgNode* BodySyncCameraItem::getScene()
{
    return impl->cameraTransform;
}


void BodySyncCameraItem::onTreePathChanged()
{
    impl->cameraTransform->setBodyItem(findOwnerItem<BodyItem>());
}


double BodySyncCameraItem::fieldOfView() const
{
    return impl->persCamera->fieldOfView();
}


bool BodySyncCameraItem::setFieldOfView(double fov)
{
    return impl->setFieldOfView(fov);
}


bool BodySyncCameraItem::Impl::setFieldOfView(double fov)
{
    if(fov > 0.0 && fov < PI){
        persCamera->setFieldOfView(fov);
        persCamera->notifyUpdate(update);
        return true;
    }
    return false;
}


double BodySyncCameraItem::nearClipDistance() const
{
    return impl->persCamera->nearClipDistance();
}


bool BodySyncCameraItem::setNearClipDistance(double nearDistance)
{
    if(nearDistance > 0.0){
        impl->setClipDistances(nearDistance, farClipDistance());
        return true;
    }
    return false;
}


double BodySyncCameraItem::farClipDistance() const
{
    return impl->persCamera->farClipDistance();
}


bool BodySyncCameraItem::setFarClipDistance(double farDistance)
{
    if(farDistance > 0.0){
        impl->setClipDistances(nearClipDistance(), farDistance);
        return true;
    }
    return false;
}


bool BodySyncCameraItem::setClipDistances(double nearDistance, double farDistance)
{
    if(nearDistance > 0.0 && farDistance > 0.0){
        impl->setClipDistances(nearDistance, farDistance);
        return true;
    }
    return false;
}


bool BodySyncCameraItem::Impl::setClipDistances(double nearDistance, double farDistance)
{
    if(persCamera->nearClipDistance() != nearDistance || persCamera->farClipDistance() != farDistance){
        persCamera->setNearClipDistance(nearDistance);
        persCamera->setFarClipDistance(farDistance);
        orthoCamera->setNearClipDistance(nearDistance);
        orthoCamera->setFarClipDistance(farDistance);
        persCamera->notifyUpdate(update);
        orthoCamera->notifyUpdate(update);
    }
    return true;
}


void BodySyncCameraItem::doPutProperties(PutPropertyFunction& putProperty)
{
    impl->doPutProperties(putProperty);
}


void BodySyncCameraItem::Impl::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Target link"), cameraTransform->targetLinkName,
                [&](const string& name){ cameraTransform->setTargetLink(name); return true; });
    putProperty(_("Camera type"), cameraType,
                [&](int index){ return setCameraType(index); });
    putProperty(_("Field Of View"), degree(self->fieldOfView()),
                [&](double fov){ return setFieldOfView(radian(fov)); } );
    putProperty(_("Near clip distance"), self->nearClipDistance(),
                [&](double distance){ return self->setNearClipDistance(distance); });
    putProperty(_("Far clip distance"), self->farClipDistance(),
                [&](double distance){ return self->setFarClipDistance(distance); } );
    putProperty(_("Parallel tracking"), cameraTransform->isParallelTrackingMode(),
                [&](bool on){ self->setParallelTrackingMode(on); return true; });
    putProperty(_("Interactive viewpoint change"), self->isInteractiveViewpointChangeEnabled(),
                [&](bool on){ self->setInteractiveViewpointChangeEnabled(on); return true; });
}


bool BodySyncCameraItem::store(Archive& archive)
{
    archive.write("target_link", impl->cameraTransform->targetLinkName, DOUBLE_QUOTED);
    archive.write("parallel_tracking", isParallelTrackingMode());
    archive.write("interactive_viewpoint_change", isInteractiveViewpointChangeEnabled());
    archive.write("camera_type", impl->cameraType.selectedSymbol());
    archive.write("near_clip_distance", impl->persCamera->nearClipDistance());    
    archive.write("far_clip_distance", impl->persCamera->farClipDistance());
    archive.write("field_of_view", impl->persCamera->fieldOfView());

    auto transform = impl->cameraTransform;
    Isometry3 T = transform->T();
    write(archive, "translation", T.translation());
    writeDegreeAngleAxis(archive, "rotation", AngleAxis(T.linear()));

    if(auto link = transform->targetLink){
        T = link->T().inverse() * T;
        write(archive, "local_translation", T.translation());
        writeDegreeAngleAxis(archive, "local_rotation", AngleAxis(T.linear()));
    }
    
    return true;
}


bool BodySyncCameraItem::restore(const Archive& archive)
{
    auto transform = impl->cameraTransform;

    string symbol;
    if(archive.read({ "target_link", "targetLink" }, symbol)){
        transform->setTargetLink(symbol);
    }
    if(auto bodyItem = dynamic_cast<BodyItem*>(archive.currentParentItem())){
        // need to update the target link object
        transform->setBodyItem(bodyItem);
    }

    bool doUpdate = false;
    bool on;
    if(archive.read("parallel_tracking", on)){
        doUpdate = true;
    } else if(archive.read("keepRelativeAttitude", on)){
        on = !on;
        doUpdate = true;
    }
    if(doUpdate){
        setParallelTrackingMode(on);
    }
    if(archive.read("interactive_viewpoint_change", on)){
        setInteractiveViewpointChangeEnabled(on);
    }
    if(archive.read({ "camera_type", "cameraType" }, symbol)){
        int index = impl->cameraType.index(symbol);
        impl->setCameraType(index);
    }
    double nearDistance =
        archive.get({ "near_clip_distance", "nearClipDistance" }, impl->persCamera->nearClipDistance());
    double farDistance =
        archive.get({ "far_clip_distance", "farClipDistance" }, impl->persCamera->farClipDistance());
    impl->setClipDistances(nearDistance, farDistance);

    impl->setFieldOfView(archive.get({ "field_of_view", "fieldOfView" }, impl->persCamera->fieldOfView()));

    bool isPositionReady = false;
    Vector3 p;
    AngleAxis aa;
    Isometry3 T = Isometry3::Identity();
    if(auto link = transform->targetLink){
        if(read(archive, "local_translation", p)){
            T.translation() = p;
            isPositionReady = true;
        }
        if(readDegreeAngleAxis(archive, "local_rotation", aa)){
            T.linear() = aa.toRotationMatrix();
            isPositionReady = true;
        }
        if(isPositionReady){
            T = link->T() * T;
        }
    }
    if(!isPositionReady){
        if(read(archive, "translation", p)){
            T.translation() = p;
        }
        if(readDegreeAngleAxis(archive, "rotation", aa)){
            T.linear() = aa.toRotationMatrix();
        }
    }
    transform->setPosition(T);
    transform->notifyUpdate();

    return true;
}
