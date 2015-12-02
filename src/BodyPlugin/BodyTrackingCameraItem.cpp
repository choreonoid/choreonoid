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
#include <boost/bind.hpp>
#include "gettext.h"

using namespace std;
using namespace boost;
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
                    bodyItem->sigKinematicStateChanged().connect(
                        boost::bind(&BodyTrackingCameraTransform::onBodyMoved, this)));
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
    BodyTrackingCameraItemImpl();
    bool onKeepRelativeAttitudeChanged(bool on);    
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
{
    cameraTransform = new BodyTrackingCameraTransform();
    cameraTransform->setPosition(
        SceneView::instance()->sceneWidget()->builtinCameraTransform()->position());

    persCamera = new SgPerspectiveCamera();
    cameraTransform->addChild(persCamera);

    orthoCamera = new SgOrthographicCamera();
    cameraTransform->addChild(orthoCamera);
}


void BodyTrackingCameraItem::setName(const std::string& name)
{
    Item::setName(name);
    
    SgUpdate update;
    impl->persCamera->setName(name + " (Perspective)");
    impl->persCamera->notifyUpdate(update);
    impl->orthoCamera->setName(name + " (Orthographic)");
    impl->orthoCamera->notifyUpdate(update);
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
    putProperty("Keep relative attitude", impl->cameraTransform->isConstantRelativeAttitudeMode(),
                boost::bind(&BodyTrackingCameraItemImpl::onKeepRelativeAttitudeChanged, impl, _1));
}


bool BodyTrackingCameraItemImpl::onKeepRelativeAttitudeChanged(bool on)
{
    cameraTransform->setConstantRelativeAttitudeMode(on);
    return true;
}


bool BodyTrackingCameraItem::store(Archive& archive)
{
    archive.write("keepRelativeAttitude", impl->cameraTransform->isConstantRelativeAttitudeMode());    
    return true;
}


bool BodyTrackingCameraItem::restore(const Archive& archive)
{
    impl->cameraTransform->setConstantRelativeAttitudeMode(
        archive.get("keepRelativeAttitude", false));
    return true;
}
