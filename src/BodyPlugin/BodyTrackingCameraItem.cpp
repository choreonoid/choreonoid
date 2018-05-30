/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "BodyTrackingCameraItem.h"
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/InteractiveCameraTransform>
#include <cnoid/SceneCameras>
#include <cnoid/SceneView>
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
    ScopedConnection connection;
    Vector3 relativeTranslationFromBody;
    Affine3 relativePositionFromBody;
    bool isSigUpdatedEmittedBySelf;
    bool isConstantRelativeAttitudeMode_;

    BodyTrackingCameraTransform() {
        bodyItem = 0;
        relativeTranslationFromBody.setZero();
        relativePositionFromBody.setIdentity();
        isSigUpdatedEmittedBySelf = false;
        isConstantRelativeAttitudeMode_ = false;
    }
    
    BodyTrackingCameraTransform(const BodyTrackingCameraTransform& org) {
        bodyItem = 0;
        relativeTranslationFromBody.setZero();
        relativePositionFromBody.setIdentity();
        isSigUpdatedEmittedBySelf = false;
        isConstantRelativeAttitudeMode_ = org.isConstantRelativeAttitudeMode_;
    }
    
    BodyTrackingCameraTransform(const BodyTrackingCameraTransform& org, SgCloneMap& cloneMap)
        : InteractiveCameraTransform(org, cloneMap) {
        bodyItem = 0;
        relativeTranslationFromBody.setZero();
        relativePositionFromBody.setIdentity();
        isSigUpdatedEmittedBySelf = false;
        isConstantRelativeAttitudeMode_ = org.isConstantRelativeAttitudeMode_;
    }

    virtual SgObject* clone(SgCloneMap& cloneMap) const {
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
            connection.disconnect();
            if(bodyItem){
                connection.reset(
                    bodyItem->sigKinematicStateChanged().connect( [&](){ onBodyMoved(); } ));
                updateRelativePosition();
            }
        }
    }

    virtual void onUpdated(SgUpdate& update) {
        if(isSigUpdatedEmittedBySelf){
            isSigUpdatedEmittedBySelf = false;
        } else {
            updateRelativePosition();
        }
        InteractiveCameraTransform::onUpdated(update);
    }
    
    void updateRelativePosition(){
        if(bodyItem){
            Link* rootLink = bodyItem->body()->rootLink();
            relativeTranslationFromBody = translation() - rootLink->translation();
            relativePositionFromBody = rootLink->position().inverse() * position();
        }
    }

    void onBodyMoved(){
        if(bodyItem){
            Link* rootLink = bodyItem->body()->rootLink();
            if(isConstantRelativeAttitudeMode_){
                setPosition(rootLink->position() * relativePositionFromBody);
            } else {
                setTranslation(rootLink->translation() + relativeTranslationFromBody);
            }
            isSigUpdatedEmittedBySelf = true;
            notifyUpdate();
        }
    }
};
  
typedef ref_ptr<BodyTrackingCameraTransform> BodyTrackingCameraTransformPtr;

}


namespace cnoid {

class BodyTrackingCameraItemImpl
{
public:
    BodyTrackingCameraTransformPtr cameraTransform;
    SgPerspectiveCameraPtr persCamera;
    SgOrthographicCameraPtr orthoCamera;
    SgUpdate update;
    Selection cameraType;
    BodyTrackingCameraItemImpl();
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
{
    impl = new BodyTrackingCameraItemImpl();
}


BodyTrackingCameraItem::BodyTrackingCameraItem(const BodyTrackingCameraItem& org)
    : Item(org)
{
    impl = new BodyTrackingCameraItemImpl();
    setName(org.name());
}


BodyTrackingCameraItemImpl::BodyTrackingCameraItemImpl()
    : cameraType(BodyTrackingCameraItem::N_CAMERA_TYPE, CNOID_GETTEXT_DOMAIN_NAME)
{
    cameraType.setSymbol(BodyTrackingCameraItem::PERSPECTIVE,  N_("Perspective"));
    cameraType.setSymbol(BodyTrackingCameraItem::ORTHOGRAPHIC, N_("Orthographic"));
    cameraType.select(BodyTrackingCameraItem::PERSPECTIVE);

    cameraTransform = new BodyTrackingCameraTransform();
    cameraTransform->setPosition(
        SceneView::instance()->sceneWidget()->builtinCameraTransform()->position());

    persCamera = new SgPerspectiveCamera();
    cameraTransform->addChild(persCamera);

    orthoCamera = new SgOrthographicCamera();
    //cameraTransform->addChild(orthoCamera);
}


void BodyTrackingCameraItem::setName(const std::string& name)
{
    Item::setName(name);
    
    impl->persCamera->setName(name);
    impl->persCamera->notifyUpdate(impl->update);
    impl->orthoCamera->setName(name);
    impl->orthoCamera->notifyUpdate(impl->update);
}


Item* BodyTrackingCameraItem::doDuplicate() const
{
    return new BodyTrackingCameraItem(*this);
}


SgNode* BodyTrackingCameraItem::getScene()
{
    return impl->cameraTransform;
}


void BodyTrackingCameraItem::onPositionChanged()
{
    impl->cameraTransform->setBodyItem(findOwnerItem<BodyItem>());
}


void BodyTrackingCameraItem::doPutProperties(PutPropertyFunction& putProperty)
{
    impl->doPutProperties(putProperty);
}


void BodyTrackingCameraItemImpl::doPutProperties(PutPropertyFunction& putProperty)
{
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


bool BodyTrackingCameraItemImpl::onKeepRelativeAttitudeChanged(bool on)
{
    cameraTransform->setConstantRelativeAttitudeMode(on);
    return true;
}


bool BodyTrackingCameraItemImpl::setClipDistances(double nearDistance, double farDistance)
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


bool BodyTrackingCameraItemImpl::setFieldOfView(double fov)
{
    persCamera->setFieldOfView(fov);
    persCamera->notifyUpdate(update);
    return true;
}


bool BodyTrackingCameraItemImpl::setCameraType(int index)
{
    if(cameraType.selectedIndex() == index){
        return true;
    }

    cameraType.select(index);

    SgCamera* newCamera;
    SgCamera* removeCamera;
    if(cameraType.is(BodyTrackingCameraItem::PERSPECTIVE)){
        newCamera = persCamera;
        removeCamera = orthoCamera;
    }else if(cameraType.is(BodyTrackingCameraItem::ORTHOGRAPHIC)){
        newCamera = orthoCamera;
        removeCamera = persCamera;
    }

    vector<SceneWidget*> sceneWidgets;
    SceneBar::instance()->sceneWidget(sceneWidgets);
    vector<SceneRenderer*> renderers;
    for(auto sceneWidget : sceneWidgets){
        renderers.push_back(sceneWidget->renderer());
    }

    cameraTransform->addChild(newCamera, true);
    for(auto renderer : renderers){
        if(renderer->currentCamera() == removeCamera){
            renderer->extractPreprocessedNodes();
            renderer->setCurrentCamera(newCamera);
        }
    }
    cameraTransform->removeChild(removeCamera, true);
    for(auto renderer : renderers){
        renderer->extractPreprocessedNodes();
    }

    return true;
}


bool BodyTrackingCameraItem::store(Archive& archive)
{
    archive.write("keepRelativeAttitude", impl->cameraTransform->isConstantRelativeAttitudeMode());
    archive.write("cameraType", impl->cameraType.selectedSymbol(), DOUBLE_QUOTED);
    archive.write("nearClipDistance", impl->persCamera->nearClipDistance());    
    archive.write("farClipDistance", impl->persCamera->farClipDistance());
    archive.write("fieldOfView", impl->persCamera->fieldOfView());
    return true;
}


bool BodyTrackingCameraItem::restore(const Archive& archive)
{
    impl->cameraTransform->setConstantRelativeAttitudeMode(
        archive.get("keepRelativeAttitude", false));
    string symbol;
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
