/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "BodyTrackingCameraItem.h"
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
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

class BodyTrackingCameraTransform : public InteractiveCameraTransform
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    BodyItem* bodyItem;
    string targetLinkName;
    LinkPtr targetLink;
    ScopedConnection connection;
    Vector3 relativeTranslationFromBody;
    Isometry3 relativePositionFromBody;
    bool isSigUpdatedEmittedBySelf;
    bool isConstantRelativeAttitudeMode_;

    BodyTrackingCameraTransform() {
        bodyItem = 0;
        relativeTranslationFromBody.setZero();
        relativePositionFromBody.setIdentity();
        isSigUpdatedEmittedBySelf = false;
        isConstantRelativeAttitudeMode_ = false;

        initialize();
    }
    
    BodyTrackingCameraTransform(const BodyTrackingCameraTransform& org, CloneMap* cloneMap)
        : InteractiveCameraTransform(org, cloneMap) {
        bodyItem = nullptr;
        relativeTranslationFromBody.setZero();
        relativePositionFromBody.setIdentity();
        isSigUpdatedEmittedBySelf = false;
        isConstantRelativeAttitudeMode_ = org.isConstantRelativeAttitudeMode_;

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
        return new BodyTrackingCameraTransform(*this, cloneMap);
    }

    bool isConstantRelativeAttitudeMode() const {
        return isConstantRelativeAttitudeMode_;
    }

    void setConstantRelativeAttitudeMode(bool on){
        isConstantRelativeAttitudeMode_ = on;
    }
            
    void setBodyItem(BodyItem* bodyItem) {
        if(bodyItem != this->bodyItem){
            this->bodyItem = bodyItem;
            updateTargetLink();
            connection.disconnect();
            if(bodyItem){
                connection.reset(
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
            if(isConstantRelativeAttitudeMode_){
                setPosition(targetLink->position() * relativePositionFromBody);
            } else {
                setTranslation(targetLink->translation() + relativeTranslationFromBody);
            }
            isSigUpdatedEmittedBySelf = true;

            // TODO: Use SgUpdate of BodyTrackingCameraItemImpl
            notifyUpdate();
        }
    }
};
  
typedef ref_ptr<BodyTrackingCameraTransform> BodyTrackingCameraTransformPtr;

}


namespace cnoid {

class BodyTrackingCameraItem::Impl
{
public:
    BodyTrackingCameraTransformPtr cameraTransform;
    SgPerspectiveCameraPtr persCamera;
    SgOrthographicCameraPtr orthoCamera;
    SgCameraPtr currentCamera;
    SgUpdate update;
    Selection cameraType;
    
    Impl(BodyTrackingCameraItem* self, bool initCameraPosition);
    Impl(BodyTrackingCameraItem* self, const Impl& org);
    void doPutProperties(PutPropertyFunction& putProperty);
    bool onKeepRelativeAttitudeChanged(bool on);
    bool setClipDistances(double nearDistance, double farDistance);
    bool setFieldOfView(double fov);
    bool setCameraType(int index);
};

}


void BodyTrackingCameraItem::initializeClass(ExtensionManager* ext)
{
    ItemManager& im = ext->itemManager();
    im.registerClass<BodyTrackingCameraItem>(N_("BodyTrackingCameraItem"));
    im.addCreationPanel<BodyTrackingCameraItem>();
}


BodyTrackingCameraItem::BodyTrackingCameraItem()
    : Item("BodyTrackingCamera")
{
    impl = new Impl(this, true);
}


BodyTrackingCameraItem::BodyTrackingCameraItem(const BodyTrackingCameraItem& org)
    : Item(org)
{
    impl = new Impl(this, *org.impl);
}


BodyTrackingCameraItem::Impl::Impl(BodyTrackingCameraItem* self, bool initCameraPosition)
    : cameraType(NumCameraTypes, CNOID_GETTEXT_DOMAIN_NAME)
{
    cameraType.setSymbol(Perspective,  N_("Perspective"));
    cameraType.setSymbol(Orthographic, N_("Orthographic"));
    cameraType.select(Perspective);

    cameraTransform = new BodyTrackingCameraTransform();
    if(initCameraPosition){
        cameraTransform->setPosition(
            SceneView::instance()->sceneWidget()->builtinCameraTransform()->position());
    }

    persCamera = new SgPerspectiveCamera();
    persCamera->setName(self->name());
    cameraTransform->addChild(persCamera);
    currentCamera = persCamera;

    orthoCamera = new SgOrthographicCamera();
    orthoCamera->setName(self->name());
    //cameraTransform->addChild(orthoCamera);
}


BodyTrackingCameraItem::Impl::Impl(BodyTrackingCameraItem* self, const Impl& org)
    : Impl(self, false)
{
    cameraType = org.cameraType;
    cameraTransform->setPosition(org.cameraTransform->position());
    cameraTransform->targetLinkName = org.cameraTransform->targetLinkName;
}


BodyTrackingCameraItem::~BodyTrackingCameraItem()
{
    delete impl;
}


bool BodyTrackingCameraItem::setName(const std::string& name)
{
    impl->persCamera->setName(name);
    impl->orthoCamera->setName(name);
    Item::setName(name);
    impl->persCamera->notifyUpdate(impl->update);
    impl->orthoCamera->notifyUpdate(impl->update);
    return true;
}


Item* BodyTrackingCameraItem::doDuplicate() const
{
    return new BodyTrackingCameraItem(*this);
}


void BodyTrackingCameraItem::setTargetLink(const std::string& name)
{
    impl->cameraTransform->setTargetLink(name);
}


const std::string& BodyTrackingCameraItem::targetLinkName() const
{
    return impl->cameraTransform->targetLinkName;
}


void BodyTrackingCameraItem::setRotationSyncEnabled(bool on)
{
    impl->cameraTransform->setConstantRelativeAttitudeMode(on);
}


bool BodyTrackingCameraItem::isRotationSyncEnabled() const
{
    return impl->cameraTransform->isConstantRelativeAttitudeMode();
}


void BodyTrackingCameraItem::setCameraType(CameraType type)
{
    impl->setCameraType(type);
}


/**
   \todo Improve the scene widget so that the current camera path described in a string list
   can be kept even if the actual camera node is changed, and simplify the following implementation.
*/
bool BodyTrackingCameraItem::Impl::setCameraType(int index)
{
    if(cameraType.selectedIndex() == index){
        return true;
    }

    cameraType.select(index);

    SgCamera* cameraToRemove;
    if(cameraType.is(BodyTrackingCameraItem::Perspective)){
        currentCamera = persCamera;
        cameraToRemove = orthoCamera;
    }else if(cameraType.is(BodyTrackingCameraItem::Orthographic)){
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


BodyTrackingCameraItem::CameraType BodyTrackingCameraItem::cameraType() const
{
    return static_cast<CameraType>(impl->cameraType.which());
}


SgCamera* BodyTrackingCameraItem::currentCamera()
{
    return impl->currentCamera;
}


SgPosTransform* BodyTrackingCameraItem::cameraTransform()
{
    return impl->cameraTransform;
}


SgNode* BodyTrackingCameraItem::getScene()
{
    return impl->cameraTransform;
}


void BodyTrackingCameraItem::onTreePathChanged()
{
    impl->cameraTransform->setBodyItem(findOwnerItem<BodyItem>());
}


void BodyTrackingCameraItem::doPutProperties(PutPropertyFunction& putProperty)
{
    impl->doPutProperties(putProperty);
}


void BodyTrackingCameraItem::Impl::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Target link"), cameraTransform->targetLinkName,
                [&](const string& name){ cameraTransform->setTargetLink(name); return true; });
    putProperty(_("Keep relative attitude"), cameraTransform->isConstantRelativeAttitudeMode(),
                [&](bool on){ return onKeepRelativeAttitudeChanged(on); } );
    putProperty(_("Camera type"), cameraType,
                [&](int index){ return setCameraType(index); });
    putProperty(_("Near clip distance"), persCamera->nearClipDistance(),
                [&](double nearDistance){ return setClipDistances( nearDistance, persCamera->farClipDistance()); } );
    putProperty(_("Far clip distance"), persCamera->farClipDistance(),
                [&](double farDistance){ return setClipDistances(persCamera->nearClipDistance(), farDistance); } );
    putProperty(_("field Of View"), degree(persCamera->fieldOfView()),
                [&](double fov){ return setFieldOfView(radian(fov)); } );
}


bool BodyTrackingCameraItem::Impl::onKeepRelativeAttitudeChanged(bool on)
{
    cameraTransform->setConstantRelativeAttitudeMode(on);
    return true;
}


bool BodyTrackingCameraItem::Impl::setClipDistances(double nearDistance, double farDistance)
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


bool BodyTrackingCameraItem::Impl::setFieldOfView(double fov)
{
    persCamera->setFieldOfView(fov);
    persCamera->notifyUpdate(update);
    return true;
}


bool BodyTrackingCameraItem::store(Archive& archive)
{
    archive.write("targetLink", impl->cameraTransform->targetLinkName, DOUBLE_QUOTED);
    archive.write("keepRelativeAttitude", impl->cameraTransform->isConstantRelativeAttitudeMode());
    archive.write("cameraType", impl->cameraType.selectedSymbol(), DOUBLE_QUOTED);
    archive.write("nearClipDistance", impl->persCamera->nearClipDistance());    
    archive.write("farClipDistance", impl->persCamera->farClipDistance());
    archive.write("fieldOfView", impl->persCamera->fieldOfView());
    return true;
}


bool BodyTrackingCameraItem::restore(const Archive& archive)
{
    string symbol;
    if(archive.read("targetLink", symbol)){
        impl->cameraTransform->setTargetLink(symbol);
    }
    impl->cameraTransform->setConstantRelativeAttitudeMode(
        archive.get("keepRelativeAttitude", false));

    if(archive.read("cameraType", symbol)){
        int index = impl->cameraType.index(symbol);
        impl->setCameraType(index);
    }
    double nearDistance = archive.get("nearClipDistance", impl->persCamera->nearClipDistance());
    double farDistance = archive.get("farClipDistance", impl->persCamera->farClipDistance());
    impl->setClipDistances(nearDistance, farDistance);
    impl->setFieldOfView( archive.get("fieldOfView", impl->persCamera->fieldOfView()) );

    return true;
}
