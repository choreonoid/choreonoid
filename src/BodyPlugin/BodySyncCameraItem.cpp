#include "BodySyncCameraItem.h"
#include "BodySyncCameraConfigDialog.h"
#include <cnoid/ItemManager>
#include <cnoid/MenuManager>
#include <cnoid/InteractiveCameraTransform>
#include <cnoid/ItemTreeView>
#include <cnoid/BodyItem>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Archive>
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
        if(on != isParallelTrackingMode_){
            isParallelTrackingMode_ = on;
            updateRelativePosition();
        }
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
        auto prevTargetLink = targetLink;
        targetLink = nullptr;
        if(bodyItem){
            if(!targetLinkName.empty()){
                targetLink = bodyItem->body()->link(targetLinkName);
            }
            if(!targetLink){
                targetLink = bodyItem->body()->rootLink();
            }
            if(targetLink != prevTargetLink){
                updateRelativePosition();
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
    
    Impl(BodySyncCameraItem* self);
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
        .registerClass<BodySyncCameraItem, CameraItem>(N_("BodySyncCameraItem"))
        .addAlias<BodySyncCameraItem>("BodyTrackingCameraItem", "Body")
        .addCreationPanel<BodySyncCameraItem>();
}


BodySyncCameraItem* BodySyncCameraItem::showDialogToCreateBodySyncCameraItem(BodyItem* bodyItem, Link* link)
{
    return BodySyncCameraConfigDialog::instance()->showToCreateCameraItem(bodyItem, link);
}


void BodySyncCameraItem::showDialogToConfigureCamera()
{
    BodySyncCameraConfigDialog::instance()->showToConfigureCameraItem(this);
}


BodySyncCameraItem::BodySyncCameraItem()
    : CameraItem("BodySyncCamera", new BodySyncCameraTransform)
{
    impl = new Impl(this);
}


BodySyncCameraItem::BodySyncCameraItem(const BodySyncCameraItem& org)
    : CameraItem(org, new BodySyncCameraTransform)
{
    impl = new Impl(this, *org.impl);
}


BodySyncCameraItem::Impl::Impl(BodySyncCameraItem* self_)
    : self(self_)
{
    cameraTransform = static_cast<BodySyncCameraTransform*>(self->cameraTransform());
}


BodySyncCameraItem::Impl::Impl(BodySyncCameraItem* self, const Impl& org)
    : Impl(self)
{
    cameraTransform->setParallelTrackingMode(org.cameraTransform->isParallelTrackingMode());
    cameraTransform->targetLinkName = org.cameraTransform->targetLinkName;
}


BodySyncCameraItem::~BodySyncCameraItem()
{
    delete impl;
}


Item* BodySyncCameraItem::doCloneItem(CloneMap* /* cloneMap */) const
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


void BodySyncCameraItem::updateRelativeCameraPosition()
{
    return impl->cameraTransform->updateRelativePosition();
}


Isometry3 BodySyncCameraItem::relativeCameraPosition() const
{
    return impl->cameraTransform->relativePositionFromBody;
}


void BodySyncCameraItem::onTreePathChanged()
{
    impl->cameraTransform->setBodyItem(findOwnerItem<BodyItem>());
}


void BodySyncCameraItem::doPutProperties(PutPropertyFunction& putProperty)
{
    CameraItem::doPutProperties(putProperty);
    impl->doPutProperties(putProperty);
}


void BodySyncCameraItem::Impl::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Target link"), cameraTransform->targetLinkName,
                [&](const string& name){ cameraTransform->setTargetLink(name); return true; });
    putProperty(_("Parallel tracking"), cameraTransform->isParallelTrackingMode(),
                [&](bool on){ self->setParallelTrackingMode(on); return true; });
}


bool BodySyncCameraItem::store(Archive& archive)
{
    if(!CameraItem::store(archive)){
        return false;
    }

    archive.write("target_link", impl->cameraTransform->targetLinkName, DOUBLE_QUOTED);
    archive.write("parallel_tracking", isParallelTrackingMode());

    auto transform = impl->cameraTransform;
    if(auto link = transform->targetLink){
        Isometry3 T = link->T().inverse() * transform->T();
        write(archive, "local_translation", T.translation());
        writeDegreeAngleAxis(archive, "local_rotation", AngleAxis(T.linear()));
    }
    
    return true;
}


bool BodySyncCameraItem::restore(const Archive& archive)
{
    if(!CameraItem::restore(archive)){
        return false;
    }

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
        
    bool hasLocalPosition = false;
    Vector3 p;
    AngleAxis aa;
    Isometry3 T = Isometry3::Identity();
    if(auto link = transform->targetLink){
        if(read(archive, "local_translation", p)){
            T.translation() = p;
            hasLocalPosition = true;
        }
        if(readDegreeAngleAxis(archive, "local_rotation", aa)){
            T.linear() = aa.toRotationMatrix();
            hasLocalPosition = true;
        }
        if(hasLocalPosition){
            T = link->T() * T;
            transform->setPosition(T);
            transform->notifyUpdate();
        }
    }
    
    return true;
}
