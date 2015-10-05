/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "BodyTrackingCameraItem.h"
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/InteractiveCameraTransform>
#include <cnoid/SceneCamera>
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
    Vector3 relativePositionFromBody;
    bool isSigUpdatedEmittedBySelf;

    BodyTrackingCameraTransform() {
        bodyItem = 0;
        relativePositionFromBody.setIdentity();
        isSigUpdatedEmittedBySelf = false;
    }
    
    BodyTrackingCameraTransform(const BodyTrackingCameraTransform& org) {
        bodyItem = 0;
        relativePositionFromBody.setIdentity();
        isSigUpdatedEmittedBySelf = false;
    }
    
    BodyTrackingCameraTransform(const BodyTrackingCameraTransform& org, SgCloneMap& cloneMap)
        : InteractiveCameraTransform(org, cloneMap) {
        bodyItem = 0;
        relativePositionFromBody.setIdentity();
        isSigUpdatedEmittedBySelf = false;
    }

    virtual SgObject* clone(SgCloneMap& cloneMap) const {
        return new BodyTrackingCameraTransform(*this, cloneMap);
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
            relativePositionFromBody = translation() - rootLink->translation();
        }
    }

    void onBodyMoved(){
        if(bodyItem){
            Link* rootLink = bodyItem->body()->rootLink();
            setTranslation(rootLink->translation() + relativePositionFromBody);
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


ItemPtr BodyTrackingCameraItem::doDuplicate() const
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

}


bool BodyTrackingCameraItem::store(Archive& archive)
{
    return true;
}


bool BodyTrackingCameraItem::restore(const Archive& archive)
{
    return true;
}
