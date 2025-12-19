#include "BodyItem.h"
#include "WorldItem.h"
#include "OperableSceneBody.h"
#include "BodyItemKinematicsKitManager.h"
#include "BodyItemKinematicsKit.h"
#include "KinematicsBar.h"
#include <cnoid/Archive>
#include <cnoid/RootItem>
#include <cnoid/ConnectionSet>
#include <cnoid/LazySignal>
#include <cnoid/MessageView>
#include <cnoid/TimeBar>
#include <cnoid/ItemManager>
#include <cnoid/OptionManager>
#include <cnoid/MenuManager>
#include <cnoid/ProjectManager>
#include <cnoid/UnifiedEditHistory>
#include <cnoid/EditRecord>
#include <cnoid/PutPropertyFunction>
#include <cnoid/JointPath>
#include <cnoid/BodyState>
#include <cnoid/InverseKinematics>
#include <cnoid/CompositeBodyIK>
#include <cnoid/PenetrationBlocker>
#include <cnoid/PinDragIK>
#include <cnoid/LeggedBodyHelper>
#include <cnoid/AttachmentDevice>
#include <cnoid/HolderDevice>
#include <cnoid/LinkedJointHandler>
#include <cnoid/RenderableItemUtil>
#include <cnoid/EigenArchive>
#include <cnoid/CloneMap>
#include <cnoid/Format>
#include <bitset>
#include <algorithm>
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

const bool TRACE_FUNCTIONS = false;

vector<string> bodyFilesToLoad;

class BodyLocation : public LocationProxy
{
public:
    BodyItem::Impl* impl;

    BodyLocation(BodyItem::Impl* impl);
    void updateLocationType();
    virtual Isometry3 getLocation() const override;
    virtual bool isLocked() const override;
    virtual void setLocked(bool on) override;
    virtual bool setLocation(const Isometry3& T) override;
    virtual void finishLocationEditing() override;
    virtual LocationProxyPtr getParentLocationProxy() override;
    virtual SignalProxy<void()> sigLocationChanged() override;
};
    
class LinkLocation : public LocationProxy
{
public:
    weak_ref_ptr<Link> refLink;

    LinkLocation(BodyItem* bodyItem, Link* link);
    virtual std::string getName() const override;
    virtual Isometry3 getLocation() const override;
    virtual bool isLocked() const override;
    virtual LocationProxyPtr getParentLocationProxy() override;
    virtual SignalProxy<void()> sigLocationChanged() override;
};

class MyCompositeBodyIK : public CompositeBodyIK
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    MyCompositeBodyIK(BodyItem::Impl* bodyItemImpl);
    bool isValid() const { return parentLinkIK != nullptr; }
    virtual std::shared_ptr<InverseKinematics> getParentBodyIK() override;
    virtual bool isBestEffortIkAvailable() const override;
    virtual bool isBestEffortIkEnabled() const override;
    virtual void setBestEffortIkEnabled(bool on) override;
    virtual int getDOF() const override;
    virtual bool calcInverseKinematics(const Isometry3& T) override;
    virtual bool calcRemainingPartForwardKinematicsForInverseKinematics() override;

    BodyItem::Impl* bodyItemImpl;
    Isometry3 T_local_inv;
    Link* parentLink;
    shared_ptr<InverseKinematics> parentLinkIK;
};

class KinematicStateRecord : public EditRecord
{
public:
    BodyItemPtr bodyItem;
    BodyItem::Impl* bodyItemImpl;
    BodyState newState;
    BodyState oldState;
    
    KinematicStateRecord(BodyItem::Impl* bodyItemImpl);
    KinematicStateRecord(BodyItem::Impl* bodyItemImpl, const BodyState& oldState);
    KinematicStateRecord(const KinematicStateRecord& org);

    virtual EditRecord* clone() const override;
    virtual std::string label() const override;
    virtual bool undo() override;
    virtual bool redo() override;
};

}

namespace cnoid {

class BodyItem::Impl
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    BodyItem* self;
    BodyPtr body;

    bool isBeingRestored; // flag to check if this item and its sub tree is being restored
    bool isLocalPositionRestored;
    bool isUpdateNotificationOnSubTreeRestoredRequested;
    bool isNonRootLinkStateRestorationOnSubTreeRestoredRequested;
    bool isSharingShapes;
    bool isLocationLocked;
    bool isKinematicStateChangeNotifiedByParentBodyItem;
    bool isProcessingInverseKinematicsIncludingParentBody;
    bool isFkRequested;
    bool isVelFkRequested;
    bool isAccFkRequested;
    bool isCollisionDetectionEnabled;
    bool isSelfCollisionDetectionEnabled;

    ScopedConnection bodyExistenceConnection;
    
    BodyItem* linkedParentBodyItem;
    Link* parentLink;
    string parentLinkName;
    ref_ptr<BodyLocation> bodyLocation;
    ref_ptr<LinkLocation> parentLinkLocation;
    AttachmentDevicePtr attachmentToParent;
    ScopedConnection linkedParentBodyItemConnection;

    enum { UF_POSITIONS, UF_VELOCITIES, UF_ACCELERATIONS, UF_CM, UF_ZMP, NUM_UPUDATE_ELEMENTS };
    std::bitset<NUM_UPUDATE_ELEMENTS> updateElements;

    LazySignal<Signal<void()>> sigKinematicStateChanged;
    Signal<void()> sigKinematicStateUpdated;

    LinkPtr currentBaseLink;
    LinkTraverse fkTraverse;
    unique_ptr<BodyItemKinematicsKitManager> kinematicsKitManager;
    shared_ptr<PinDragIK> pinDragIK;

    bool isJointRangeLimitEnabled;

    BodyState initialState;
    BodyState lastEditState;

    BodyPtr exchangedMultiplexBody;
            
    OperableSceneBodyPtr sceneBody;
    float transparency;
    Signal<void(int flags)> sigModelUpdated;

    LeggedBodyHelperPtr legged;

    static unique_ptr<RenderableItemUtil> renderableItemUtil;

    Impl(BodyItem* self);
    Impl(BodyItem* self, const Impl& org, CloneMap* cloneMap);
    Impl(BodyItem* self, Body* body, bool isSharingShapes);
    ~Impl();
    void init(bool calledFromCopyConstructor);
    void initBody(bool calledFromCopyConstructor);
    void resetLinkCollisions();
    bool doAssign(const Item* srcItem);
    void setBody(Body* body);
    void notifyModelUpdate(int flags);
    void setCurrentBaseLink(Link* link, bool forceUpdate, bool doNotify);
    bool makeRootFixed();
    bool makeRootFree();
    BodyItemKinematicsKitManager* getOrCreateKinematicsKitManager();
    void createPenetrationBlocker(Link* link, bool excludeSelfCollisions, shared_ptr<PenetrationBlocker>& blocker);
    void setPresetPose(BodyItem::PresetPoseID id);
    bool doLegIkToMoveCm(const Vector3& c, bool onlyProjectionToFloor);
    bool setStance(double width);
    void getParticularPosition(BodyItem::PositionType position, stdx::optional<Vector3>& pos);
    void notifyKinematicStateChange(bool requestFK, bool requestVelFK, bool requestAccFK, bool isDirect);
    void emitSigKinematicStateChanged();
    bool setCollisionDetectionEnabled(bool on);
    bool setSelfCollisionDetectionEnabled(bool on);
    void updateCollisionDetectorLater();
    void setLocationLocked(bool on, bool updateInitialPositionWhenLocked, bool doNotiyUpdate);
    void createSceneBody();
    void setTransparency(float t);
    bool isLinkableToParentBody() const;
    void setParentLink(const std::string& name);
    bool updateParentBodyLinkage(bool doNotifyUpdate);
    void clearParentBodyLinkage(bool doNotifyUpdate);
    bool updateParentBodyLinkageTo(BodyItem* parentBodyItem);
    bool findMatchedAttachmentAndHolder(
        BodyItem* parentBodyItem, AttachmentDevice*& out_attachment, HolderDevice*& out_holder);
    void setLinkageToParentBody(BodyItem* parentBodyItem, Link* newParentLink);
    void setLinkageToParentBody(
        BodyItem* parentBodyItem, AttachmentDevice* attachment, HolderDevice* holder);
    void setLinkageToParentBody(BodyItem* parentBodyItem, int linkageType);
    void setLocalPositionOnParentBodyLink();
    void onParentBodyKinematicStateChanged();
    void doPutProperties(PutPropertyFunction& putProperty);
    bool store(Archive& archive);
    bool restore(const Archive& archive);
    void restoreNonRootLinkStates(const Archive& archive);
    RenderableItemUtil* getOrCreateRenderableItemUtil();
};

unique_ptr<RenderableItemUtil> BodyItem::Impl::renderableItemUtil;

}


static void onSigOptionsParsed(OptionManager*)
{
    for(auto& file : bodyFilesToLoad){
        BodyItemPtr item = new BodyItem;
        auto rootItem = RootItem::instance();
        if(item->load(file, rootItem, "CHOREONOID-BODY")){
            item->setChecked(true);
            rootItem->addChildItem(item);
    	}
    }
}


void BodyItem::initializeClass(ExtensionManager* ext)
{
    ItemManager* im = &ext->itemManager();
    im->registerClass<BodyItem>(N_("BodyItem"));

    // Implemented in BodyItemFileIO.cpp
    registerBodyItemFileIoSet(im);

    auto om = OptionManager::instance();
    om->add_option("--body", bodyFilesToLoad, "load a body file");
    om->sigOptionsParsed(1).connect(onSigOptionsParsed);
}


BodyItem::BodyItem()
{
    setAttributes(FileImmutable | Reloadable);

    impl = new Impl(this);
    impl->init(false);

    preferredParentBodyLinkage_ = Coordinated;
    currentParentBodyLinkage_ = Unlinked;
    isVisibleLinkSelectionMode_ = false;
}
    

BodyItem::BodyItem(const std::string& name)
    : BodyItem()
{
    BodyItem::setName(name);
}


BodyItem::Impl::Impl(BodyItem* self)
    : Impl(self, new Body, false)
{
    body->rootLink()->setName("Root");
    isCollisionDetectionEnabled = true;
    isSelfCollisionDetectionEnabled = false;
    isJointRangeLimitEnabled = true;
}


BodyItem::Impl::Impl(BodyItem* self, Body* body, bool isSharingShapes)
    : self(self),
      body(body),
      isBeingRestored(false),
      isLocalPositionRestored(false),
      isSharingShapes(isSharingShapes),
      isLocationLocked(false),
      sigKinematicStateChanged([this](){ emitSigKinematicStateChanged(); })
{

}


BodyItem::BodyItem(const BodyItem& org, CloneMap* cloneMap)
    : Item(org)
{
    impl = new Impl(this, *org.impl, cloneMap);
    impl->init(true);

    preferredParentBodyLinkage_ = org.preferredParentBodyLinkage_;
    currentParentBodyLinkage_ = Unlinked;
    isVisibleLinkSelectionMode_ = org.isVisibleLinkSelectionMode_;
}


BodyItem::Impl::Impl(BodyItem* self, const Impl& org, CloneMap* cloneMap)
    : Impl(self, CloneMap::getClone(org.body, cloneMap), true)
{
    isCollisionDetectionEnabled = org.isCollisionDetectionEnabled;
    isSelfCollisionDetectionEnabled = org.isSelfCollisionDetectionEnabled;
    parentLinkName = org.parentLinkName;

    if(org.currentBaseLink){
        setCurrentBaseLink(body->link(org.currentBaseLink->index()), true, false);
    } else {
        setCurrentBaseLink(nullptr, true, false);
    }

    isJointRangeLimitEnabled = org.isJointRangeLimitEnabled;

    initialState = org.initialState;
    transparency = org.transparency;
}


BodyItem::~BodyItem()
{
    delete impl;
}


BodyItem::Impl::~Impl()
{

}


void BodyItem::Impl::init(bool calledFromCopyConstructor)
{
    transparency = 0.0f;
    isFkRequested = isVelFkRequested = isAccFkRequested = false;

    initBody(calledFromCopyConstructor);
}


void BodyItem::Impl::initBody(bool calledFromCopyConstructor)
{
    linkedParentBodyItem = nullptr;
    parentLink = nullptr;
    isKinematicStateChangeNotifiedByParentBodyItem = false;
    isProcessingInverseKinematicsIncludingParentBody = false;
    
    if(pinDragIK){
        pinDragIK.reset();
    }

    if(!calledFromCopyConstructor){
        setCurrentBaseLink(nullptr, true, false);
        self->storeInitialState();
    }

    resetLinkCollisions();
}


void BodyItem::Impl::resetLinkCollisions()
{
    int n = body->numLinks();
    self->collisionsOfLink_.clear();
    self->collisionsOfLink_.resize(n);
    self->collisionLinkBitSet_.clear();
    self->collisionLinkBitSet_.resize(n);
}


Item* BodyItem::doCloneItem(CloneMap* cloneMap) const
{
    return new BodyItem(*this, cloneMap);
}


bool BodyItem::doAssign(const Item* srcItem)
{
    return impl->doAssign(srcItem);
}


bool BodyItem::Impl::doAssign(const Item* srcItem)
{
    const BodyItem* srcBodyItem = dynamic_cast<const BodyItem*>(srcItem);

    if(!srcBodyItem){
        return false;
    }
    
    auto srcImpl = srcBodyItem->impl;
    self->preferredParentBodyLinkage_ = srcBodyItem->preferredParentBodyLinkage_;
    self->isVisibleLinkSelectionMode_ = srcBodyItem->isVisibleLinkSelectionMode_;
    isLocationLocked = srcImpl->isLocationLocked;
    isCollisionDetectionEnabled = srcImpl->isCollisionDetectionEnabled;
    isSelfCollisionDetectionEnabled = srcImpl->isSelfCollisionDetectionEnabled;
    parentLinkName = srcImpl->parentLinkName;
    transparency = srcImpl->transparency;
    
    // copy the base link property
    Link* baseLink = nullptr;
    Link* srcBaseLink = srcBodyItem->currentBaseLink();
    if(srcBaseLink){
        baseLink = body->link(srcBaseLink->name());
        if(baseLink){
            setCurrentBaseLink(baseLink, false, false);
        }
    }
    Body* srcBody = srcBodyItem->body();
    if(baseLink){
        baseLink->p() = srcBaseLink->p();
        baseLink->R() = srcBaseLink->R();
    } else {
        body->rootLink()->p() = srcBody->rootLink()->p();
        body->rootLink()->R() = srcBody->rootLink()->R();
    }
    
    // copy the current kinematic state
    int numSrcLinks = srcBody->numLinks();
    for(int i=0; i < numSrcLinks; ++i){
        Link* srcLink = srcBody->link(i);
        Link* link = body->link(srcLink->name());
        if(link){
            link->q() = srcLink->q();
        }
    }

    self->calcForwardKinematics();
    self->storeKinematicState(initialState);
    
    self->notifyKinematicStateChange();

    return true;
}


void BodyItem::onTreePathChanged()
{
    auto worldItem = findOwnerItem<WorldItem>();
    if(!worldItem){
        clearCollisions(true);
    }

    /*
      If the item is being restored in loading a project, the linkage to parent
      body item is updated when all the child items are restored so that an attachment
      device dynamically added by a DeviceOverwriteItem can work in the linkage update.
      In that case, the linkage update is processed by the callback function
      given to the Archive::addProcessOnSubTreeRestored in the restore function.
    */
    if(!impl->isBeingRestored){
        impl->updateParentBodyLinkage(true);
    }
}


void BodyItem::onConnectedToRoot()
{
    storeKinematicState(impl->lastEditState);
}    


bool BodyItem::setName(const std::string& name)
{
    auto body = impl->body;
    if(body){
        body->setName(name);
        if(body->modelName().empty()){
            body->setModelName(name);
        }
    }
    return Item::setName(name);
}


Body* BodyItem::body() const
{
    return impl->body;
}


void BodyItem::setBody(Body* body)
{
    impl->setBody(body);
}


void BodyItem::Impl::setBody(Body* body_)
{
    body = body_;

    bodyExistenceConnection = body->sigExistenceChanged().connect([this](bool){ self->notifyUpdate(); });

    auto rootLink = body->rootLink();
    if(rootLink->name().empty()){
        rootLink->setName("Root");
    }
        
    body->initializePosition();
    body->setCurrentTimeFunction([](){ return TimeBar::instance()->time(); });

    initBody(false);

    auto& itemName = self->name();
    if(itemName.empty()){
        if(!body_->name().empty()){
            self->setName(body_->name());
        } else if(!body_->modelName().empty()){
            self->setName(body_->modelName());
        }
    } else {
        if(body_->name().empty()){
            body->setName(itemName);
        }
        if(body_->modelName().empty()){
            body->setModelName(itemName);
        }
    }

    // Is this necessary?
    //notifyModelUpdate();
    //self->notifyUpdate();
}


bool BodyItem::isSharingShapes() const
{
    return impl->isSharingShapes;
}


void BodyItem::cloneShapes(CloneMap& cloneMap)
{
    impl->body->cloneShapes(cloneMap);
}


SignalProxy<void()> BodyItem::sigKinematicStateChanged()
{
    return impl->sigKinematicStateChanged.signal();
}


SignalProxy<void()> BodyItem::sigKinematicStateUpdated()
{
    return impl->sigKinematicStateUpdated;
}


SignalProxy<void(int flags)> BodyItem::sigModelUpdated()
{
    return impl->sigModelUpdated;
}


void BodyItem::notifyModelUpdate(int flags)
{
    impl->notifyModelUpdate(flags);
}


void BodyItem::Impl::notifyModelUpdate(int flags)
{
    bool hasMultiplexBody = body->nextMultiplexBody();
    if(hasMultiplexBody){
        if(flags & LinkSetUpdate){
            body->clearMultiplexBodies(true);
        } else {
            BodyState state;
            state.storeMultiplexStateOfBody(body);
            body->clearMultiplexBodies(true);
            state.restoreMultiplexStateToBody(body);
        }
    }
    
    if(flags & LinkSetUpdate){
        resetLinkCollisions();
        setCurrentBaseLink(currentBaseLink, true, false);
        if(kinematicsKitManager){
            kinematicsKitManager->clearKinematicsKits();
        }
    }

    if(sceneBody){
        if(hasMultiplexBody ||
           (flags & (LinkSetUpdate | LinkSpecUpdate | ShapeUpdate))){
            sceneBody->updateSceneModel();
            if(transparency > 0.0f){
                sceneBody->setTransparency(transparency);
            }
        } else if(flags & (DeviceSetUpdate | DeviceSpecUpdate)){
            //! This is a temporary code to support DeviceOverwriteItem.
            sceneBody->updateSceneDeviceModels(true);
        }
    }

    sigModelUpdated(flags);

    self->notifyUpdate();
}


Link* BodyItem::currentBaseLink() const
{
    return impl->currentBaseLink;
}


void BodyItem::setCurrentBaseLink(Link* link)
{
    impl->setCurrentBaseLink(link, false, true);
}


void BodyItem::Impl::setCurrentBaseLink(Link* link, bool forceUpdate, bool doNotify)
{
    if(link != currentBaseLink || forceUpdate){
        if(link){
            fkTraverse.find(link, true, true);
        } else {
            fkTraverse.find(body->rootLink());
        }
        currentBaseLink = link;
        if(doNotify){
            self->notifyUpdate();
        }
    }
}


/**
   Forward kinematics from the current base link is done.
*/
void BodyItem::calcForwardKinematics(bool calcVelocity, bool calcAcceleration)
{
    impl->fkTraverse.calcForwardKinematics(calcVelocity, calcAcceleration);
}


void BodyItem::storeKinematicState(BodyState& state)
{
    state.storeMultiplexStateOfBody(impl->body);
}


void BodyItem::restoreKinematicState(const BodyState& state)
{
    state.restoreMultiplexStateToBody(impl->body);
}


void BodyItem::storeInitialState()
{
    Item::setConsistentWithProjectArchive(false);
    storeKinematicState(impl->initialState);
}


void BodyItem::restoreInitialState(bool doNotify)
{
    restoreKinematicState(impl->initialState);
    if(doNotify){
        notifyKinematicStateUpdate();
    }
}


void BodyItem::getInitialState(BodyState& out_state)
{
    out_state = impl->initialState;
}


void BodyItem::beginKinematicStateEdit()
{
    if(TRACE_FUNCTIONS){
        cout << "BodyItem::beginKinematicStateEdit()" << endl;
    }
}


void BodyItem::cancelKinematicStateEdit()
{
    if(TRACE_FUNCTIONS){
        cout << "BodyItem::cancelKinematicStateEdit()" << endl;
    }
}
        

void BodyItem::acceptKinematicStateEdit()
{
    if(TRACE_FUNCTIONS){
        cout << "BodyItem::acceptKinematicStateEdit()" << endl;
    }

    notifyKinematicStateUpdate(false);
}


BodyItemKinematicsKitManager* BodyItem::Impl::getOrCreateKinematicsKitManager()
{
    if(!kinematicsKitManager){
        kinematicsKitManager.reset(new BodyItemKinematicsKitManager(self));
        kinematicsKitManager->setIkJointLimitEnabled(isJointRangeLimitEnabled);
    }
    return kinematicsKitManager.get();
}


BodyItemKinematicsKit* BodyItem::findPresetKinematicsKit(Link* targetLink)
{
    return impl->getOrCreateKinematicsKitManager()->findPresetKinematicsKit(targetLink);
}
    

std::shared_ptr<InverseKinematics> BodyItem::findPresetIK(Link* targetLink)
{
    if(isAttachedToParentBody() && targetLink->isFixedToRoot()){
        auto ik = make_shared<MyCompositeBodyIK>(impl);
        if(ik->isValid()){
            return ik;
        }
    } else if(auto kinematicsKit = findPresetKinematicsKit(targetLink)){
        return kinematicsKit->inverseKinematics();
    }
    return nullptr;
}


BodyItemKinematicsKit* BodyItem::getCurrentKinematicsKit(Link* targetLink)
{
    return impl->getOrCreateKinematicsKitManager()->getCurrentKinematicsKit(targetLink);
}


std::shared_ptr<InverseKinematics> BodyItem::getCurrentIK(Link* targetLink)
{
    std::shared_ptr<InverseKinematics> ik;
    if(isAttachedToParentBody() && targetLink->isFixedToRoot()){
        auto compositeBodyIK = make_shared<MyCompositeBodyIK>(impl);
        if(compositeBodyIK->isValid()){
            ik = compositeBodyIK;
        }
    } else if(auto kinematicsKit = getCurrentKinematicsKit(targetLink)){
        ik = kinematicsKit->inverseKinematics();
    }
    return ik;
}


std::shared_ptr<PinDragIK> BodyItem::getOrCreatePinDragIK()
{
    if(!impl->pinDragIK){
        impl->pinDragIK = std::make_shared<PinDragIK>(impl->body);
    }
    return impl->pinDragIK;
}


std::shared_ptr<PinDragIK> BodyItem::checkPinDragIK()
{
    return impl->pinDragIK;
}


std::shared_ptr<PenetrationBlocker> BodyItem::createPenetrationBlocker(Link* link, bool excludeSelfCollisions)
{
    shared_ptr<PenetrationBlocker> blocker;
    impl->createPenetrationBlocker(link, excludeSelfCollisions, blocker);
    return blocker;
}


void BodyItem::Impl::createPenetrationBlocker(Link* link, bool excludeSelfCollisions, shared_ptr<PenetrationBlocker>& blocker)
{
    WorldItem* worldItem = self->findOwnerItem<WorldItem>();
    if(worldItem){
        blocker = std::make_shared<PenetrationBlocker>(worldItem->collisionDetector()->clone(), link);
        const ItemList<BodyItem>& bodyItems = worldItem->coldetBodyItems();
        for(size_t i=0; i < bodyItems.size(); ++i){
            BodyItem* bodyItem = bodyItems.get(i);
            if(bodyItem != self && bodyItem->body()->isStaticModel()){
                blocker->addOpponentLink(bodyItem->body()->rootLink());
            }
        }
        blocker->setDepth(KinematicsBar::instance()->penetrationBlockDepth());
        blocker->start();
    }
}


bool BodyItem::isJointRangeLimitEnabled() const
{
    return impl->isJointRangeLimitEnabled;
}


void BodyItem::setJointRangeLimitEnabled(bool on)
{
    if(on != impl->isJointRangeLimitEnabled){
        impl->isJointRangeLimitEnabled = on;
        if(impl->kinematicsKitManager){
            impl->kinematicsKitManager->setIkJointLimitEnabled(on);
        }
    }
}


void BodyItem::exchangeWithMultiplexBody(Body* multiplexBody)
{
    impl->body->exchangePositionWithMultiplexBody(multiplexBody);
    impl->exchangedMultiplexBody = multiplexBody;
}


void BodyItem::moveToOrigin()
{
    impl->body->rootLink()->T() = impl->body->defaultPosition();
    impl->body->calcForwardKinematics();
    notifyKinematicStateUpdate();
}


void BodyItem::setPresetPose(PresetPoseID id)
{
    impl->setPresetPose(id);
}


void BodyItem::Impl::setPresetPose(BodyItem::PresetPoseID id)
{
    int jointIndex = 0;

    if(id == BodyItem::STANDARD_POSE){
        auto info = body->info();
        const Listing& pose = *info->findListing({ "standard_pose", "standardPose" });
        if(pose.isValid()){
            const int n = std::min(pose.size(), body->numJoints());
            while(jointIndex < n){
                double q = pose[jointIndex].toDouble();
                if(!info->isForcedRadianMode()){
                    q = radian(q);
                }
                body->joint(jointIndex)->q() = q;
                jointIndex++;
            }
        }
    }

    const int n = body->numAllJoints();
    while(jointIndex < n){
        Link* joint = body->joint(jointIndex++);
        joint->q() = joint->q_initial();
    }

    if(auto linkedJointHandler = body->findHandler<LinkedJointHandler>()){
        linkedJointHandler->updateLinkedJointDisplacements();
    }

    fkTraverse.calcForwardKinematics();
    self->notifyKinematicStateUpdate();
}


const Vector3& BodyItem::centerOfMass()
{
    if(!impl->updateElements.test(BodyItem::Impl::UF_CM)){
        impl->body->calcCenterOfMass();
        impl->updateElements.set(BodyItem::Impl::UF_CM);
    }

    return impl->body->centerOfMass();
}


bool BodyItem::isLeggedBody() const
{
    if(!impl->legged){
        impl->legged = getLeggedBodyHelper(impl->body);
    }
    return impl->legged->isValid();
}
        
/**
   \todo use getDefaultIK() if the kinematics bar is in the AUTO mode.
*/
bool BodyItem::doLegIkToMoveCm(const Vector3& c, bool onlyProjectionToFloor)
{
    return impl->doLegIkToMoveCm(c, onlyProjectionToFloor);
}


bool BodyItem::Impl::doLegIkToMoveCm(const Vector3& c, bool onlyProjectionToFloor)
{
    bool result = false;

    if(self->isLeggedBody()){
        
        BodyState orgKinematicState;
        self->storeKinematicState(orgKinematicState);
        
        result = legged->doLegIkToMoveCm(c, onlyProjectionToFloor);

        if(result){
            self->notifyKinematicStateUpdate();
            updateElements.set(UF_CM);
        } else {
            self->restoreKinematicState(orgKinematicState);
        }
    }

    return result;
}


bool BodyItem::setStance(double width)
{
    return impl->setStance(width);
}


bool BodyItem::Impl::setStance(double width)
{
    bool result = false;
    
    if(self->isLeggedBody()){
        
        BodyState orgKinematicState;
        self->storeKinematicState(orgKinematicState);
        
        result = legged->setStance(width, currentBaseLink);

        if(result){
            self->notifyKinematicStateUpdate();
        } else {
            self->restoreKinematicState(orgKinematicState);
        }
    }

    return result;
}
                

stdx::optional<Vector3> BodyItem::getParticularPosition(PositionType position)
{
    stdx::optional<Vector3> pos;
    impl->getParticularPosition(position, pos);
    return pos;
}


void BodyItem::Impl::getParticularPosition(BodyItem::PositionType position, stdx::optional<Vector3>& pos)
{
    if(position == BodyItem::ZERO_MOMENT_POINT){
        if(self->isLeggedBody()){
            pos = legged->zmp();
        }
    } else {
        if(position == BodyItem::CM_PROJECTION){
            pos = self->centerOfMass();

        } else if(self->isLeggedBody()){
            if(position == BodyItem::HOME_COP){
                pos = legged->homeCopOfSoles();
            } else if(position == BodyItem::RIGHT_HOME_COP || position == BodyItem::LEFT_HOME_COP) {
                if(legged->numFeet() == 2){
                    int which;
                    if(position == BodyItem::LEFT_HOME_COP){
                        which = LeggedBodyHelper::Left;
                    }  else {
                        which = LeggedBodyHelper::Right;
                    }
                    pos = legged->homeCopOfSole(which);
                }
            }
        }
        if(pos){
            (*pos).z() = 0.0;
        }
    }
}


void BodyItem::Impl::notifyKinematicStateChange(bool requestFK, bool requestVelFK, bool requestAccFK, bool isDirect)
{
    updateElements.reset();

    if(isProcessingInverseKinematicsIncludingParentBody){
        isProcessingInverseKinematicsIncludingParentBody = false;
        if(linkedParentBodyItem){
            linkedParentBodyItem->impl->notifyKinematicStateChange(
                requestFK, requestVelFK, requestAccFK, isDirect);
        }
    } else {
        if(requestFK){
            isFkRequested |= requestFK;
            isVelFkRequested |= requestVelFK;
            isAccFkRequested |= requestAccFK;
        }
        if(isDirect){
            sigKinematicStateChanged.emit();
        } else {
            sigKinematicStateChanged.request();
        }
    }
}


void BodyItem::Impl::emitSigKinematicStateChanged()
{
    if(isFkRequested){
        fkTraverse.calcForwardKinematics(isVelFkRequested, isAccFkRequested);
        isFkRequested = isVelFkRequested = isAccFkRequested = false;
    }
    if(isKinematicStateChangeNotifiedByParentBodyItem){
        isKinematicStateChangeNotifiedByParentBodyItem = false;
    } else {
        if(self->currentParentBodyLinkage_ == Coordinated){
            setLocalPositionOnParentBodyLink();
        }
    }

    sigKinematicStateChanged.signal()();
}


void BodyItem::notifyKinematicStateChange(bool requestFK, bool requestVelFK, bool requestAccFK)
{
    impl->notifyKinematicStateChange(requestFK, requestVelFK, requestAccFK, true);
}


void BodyItem::notifyKinematicStateChange
(Connection& connectionToBlock, bool requestFK, bool requestVelFK, bool requestAccFK)
{
    impl->sigKinematicStateChanged.requestBlocking(connectionToBlock);
    impl->notifyKinematicStateChange(requestFK, requestVelFK, requestAccFK, true);
}


void BodyItem::notifyKinematicStateChangeLater(bool requestFK, bool requestVelFK, bool requestAccFK)
{
    impl->notifyKinematicStateChange(requestFK, requestVelFK,requestAccFK, false);
}


void BodyItem::notifyKinematicStateChangeLater
(Connection& connectionToBlock, bool requestFK, bool requestVelFK, bool requestAccFK)
{
    impl->sigKinematicStateChanged.requestBlocking(connectionToBlock);
    impl->notifyKinematicStateChange(requestFK, requestVelFK, requestAccFK, false);
}


void BodyItem::notifyKinematicStateUpdate(bool doNotifyStateChange)
{
    if(doNotifyStateChange){
        impl->notifyKinematicStateChange(false, false, false, true);
    }
    
    impl->sigKinematicStateUpdated();

    if(isAttachedToParentBody()){
        impl->linkedParentBodyItem->notifyKinematicStateUpdate(false);
    }

    auto record = new KinematicStateRecord(impl, impl->lastEditState);
    UnifiedEditHistory::instance()->addRecord(record);
    storeKinematicState(impl->lastEditState);
}


void BodyItem::setCollisionDetectionEnabled(bool on)
{
    impl->setCollisionDetectionEnabled(on);
}


bool BodyItem::Impl::setCollisionDetectionEnabled(bool on)
{
    if(on != isCollisionDetectionEnabled){
        isCollisionDetectionEnabled = on;
        if(!on){
            self->clearCollisions(true);
        }
        updateCollisionDetectorLater();
        return true;
    }
    return false;
}


void BodyItem::setSelfCollisionDetectionEnabled(bool on)
{
    impl->setSelfCollisionDetectionEnabled(on);
}


bool BodyItem::Impl::setSelfCollisionDetectionEnabled(bool on)
{
    if(on != isSelfCollisionDetectionEnabled){
        isSelfCollisionDetectionEnabled = on;
        if(!on){
            self->clearCollisions(true);
        }
        updateCollisionDetectorLater();
        return true;
    }
    return false;
}


void BodyItem::Impl::updateCollisionDetectorLater()
{
    if(TRACE_FUNCTIONS){
        cout << "BodyItem::Impl::updateCollisionDetectorLater(): " << self->name() << endl;
    }
    
    WorldItem* worldItem = self->findOwnerItem<WorldItem>();
    if(worldItem){
        worldItem->updateCollisionDetectionBodiesLater();
    }
}

        
bool BodyItem::isCollisionDetectionEnabled() const
{
    return impl->isCollisionDetectionEnabled;
}


bool BodyItem::isSelfCollisionDetectionEnabled() const
{
    return impl->isSelfCollisionDetectionEnabled;
}


void BodyItem::clearCollisions(bool doNotifyCollisionUpdate)
{
    collisions_.clear();

    bool changed = false;
    for(size_t i=0; i < collisionLinkBitSet_.size(); ++i){
        if(collisionLinkBitSet_[i]){
            collisionsOfLink_[i].clear();
            collisionLinkBitSet_[i] = false;
            changed = true;
        }
    }
    if(doNotifyCollisionUpdate && changed){
        notifyCollisionUpdate();
    }
}


LocationProxyPtr BodyItem::getLocationProxy()
{
    if(!impl->bodyLocation){
        impl->bodyLocation = new BodyLocation(impl);
    }
    return impl->bodyLocation;
}


bool BodyItem::isLocationLocked() const
{
    return impl->isLocationLocked;
}


void BodyItem::setLocationLocked(bool on)
{
    impl->setLocationLocked(on, true, true);
}


void BodyItem::Impl::setLocationLocked(bool on, bool updateInitialPositionWhenLocked, bool doNotiyUpdate)
{
    if(on != isLocationLocked){
        isLocationLocked = on;

        if(on && updateInitialPositionWhenLocked){
            initialState.rootLinkPosition().set(body->rootLink()->T());
        }
        if(sceneBody){
            sceneBody->notifyUpdate();
        }
        if(bodyLocation){
            bodyLocation->notifyAttributeChange();
        }
        if(doNotiyUpdate){
            self->notifyUpdate();
        }
    }
}


LocationProxyPtr BodyItem::createLinkLocationProxy(Link* link)
{
    return new LinkLocation(this, link);
}


BodyLocation::BodyLocation(BodyItem::Impl* impl)
    : LocationProxy(impl->self, impl->self->isAttachedToParentBody()? OffsetLocation : GlobalLocation),
      impl(impl)
{

}


void BodyLocation::updateLocationType()
{
    if(impl->self->isAttachedToParentBody()){
        setLocationType(OffsetLocation);
    } else {
        setLocationType(GlobalLocation);
    }
}


Isometry3 BodyLocation::getLocation() const
{
    auto rootLink = impl->body->rootLink();
    switch(locationType()){
    case OffsetLocation:
        // relative position from the parent link
        return rootLink->offsetPosition();
    case GlobalLocation:
        return rootLink->T();
    case InvalidLocation:
    case ParentRelativeLocation:
        break;
    }
    return Isometry3::Identity();
}


bool BodyLocation::isLocked() const
{
    return impl->self->isLocationLocked();
}


void BodyLocation::setLocked(bool on)
{
    impl->setLocationLocked(on, true, true);
}


bool BodyLocation::setLocation(const Isometry3& T)
{
    auto rootLink = impl->body->rootLink();
    switch(locationType()){
    case OffsetLocation:
        rootLink->setOffsetPosition(T);
        impl->linkedParentBodyItem->notifyKinematicStateChange(true);
        return true;
    case GlobalLocation:
        rootLink->setPosition(T);
        impl->body->calcForwardKinematics();
        impl->self->notifyKinematicStateChange();
        return true;
    default:
        break;
    }
    return false;
}


void BodyLocation::finishLocationEditing()
{
    impl->self->notifyKinematicStateUpdate(false);
}


LocationProxyPtr BodyLocation::getParentLocationProxy()
{
    if(impl->linkedParentBodyItem){
        if(!impl->self->isAttachedToParentBody()){
            return impl->linkedParentBodyItem->getLocationProxy();
        } else {
            auto parentLink = impl->body->parentBodyLink();
            if(impl->parentLinkLocation){
                if(impl->parentLinkLocation->refLink.lock() != parentLink){
                    impl->parentLinkLocation.reset();
                }
            }
            if(!impl->parentLinkLocation){
                impl->parentLinkLocation = new LinkLocation(impl->linkedParentBodyItem, parentLink);
            }
            return impl->parentLinkLocation;
        }
    }
    return nullptr;
}


SignalProxy<void()> BodyLocation::sigLocationChanged()
{
    return impl->sigKinematicStateChanged.signal();
}


LinkLocation::LinkLocation(BodyItem* bodyItem, Link* link)
    : LocationProxy(bodyItem, GlobalLocation),
      refLink(link)
{
    setNameDependencyOnItemName();
}


std::string LinkLocation::getName() const
{
    if(auto link = refLink.lock()){
        return link->body()->name() + " - " + link->name();
    }
    return string();
}


Isometry3 LinkLocation::getLocation() const
{
    if(auto link = refLink.lock()){
        return link->T();
    }
    return Isometry3::Identity();
}


bool LinkLocation::isLocked() const
{
    return true;
}


LocationProxyPtr LinkLocation::getParentLocationProxy()
{
    if(auto bodyItem = static_cast<BodyItem*>(locatableItem())){
        bodyItem->getLocationProxy();
    }
    return nullptr;
}


SignalProxy<void()> LinkLocation::sigLocationChanged()
{
    if(auto bodyItem = static_cast<BodyItem*>(locatableItem())){
        return bodyItem->sigKinematicStateChanged();
    } else {
        static Signal<void()> dummySignal;
        return dummySignal;
    }
}


OperableSceneBody* BodyItem::sceneBody()
{
    if(!impl->sceneBody){
        impl->createSceneBody();
    }
    return impl->sceneBody;
}


void BodyItem::Impl::createSceneBody()
{
    sceneBody = new OperableSceneBody(self);
    sceneBody->setSceneDeviceUpdateConnection(true);
    if(transparency > 0.0f){
        sceneBody->setTransparency(transparency);
    }
}


SgNode* BodyItem::getScene()
{
    return sceneBody();
}


OperableSceneBody* BodyItem::existingSceneBody()
{
    return impl->sceneBody;
}


float BodyItem::transparency() const
{
    return impl->transparency;
}



void BodyItem::setTransparency(float t)
{
    impl->setTransparency(t);
}


void BodyItem::Impl::setTransparency(float t)
{
    if(t != transparency){
        if(sceneBody){
            sceneBody->setTransparency(t);
        }
        transparency = t;
    }
}


bool BodyItem::Impl::isLinkableToParentBody() const
{
    return self->findOwnerItem<BodyItem>() != nullptr;
}


BodyItem* BodyItem::linkedParentBodyItem() const
{
    return impl->linkedParentBodyItem;
}


void BodyItem::Impl::setParentLink(const std::string& name)
{
    if(name != parentLinkName){
        parentLinkName = name;
        updateParentBodyLinkage(true);
    }
}


void BodyItem::setParentLink(const std::string& name)
{
    impl->setParentLink(name);
}


const std::string& BodyItem::parentLinkName() const
{
    return impl->parentLinkName;
}


bool BodyItem::setPreferredParentBodyLinkage(int linkageType, bool doNotifyUpdate)
{
    preferredParentBodyLinkage_ = linkageType;
    if(preferredParentBodyLinkage_ != currentParentBodyLinkage_){
        impl->updateParentBodyLinkage(doNotifyUpdate);
    }
    return currentParentBodyLinkage_ == preferredParentBodyLinkage_;
}


bool BodyItem::Impl::updateParentBodyLinkage(bool doNotifyUpdate)
{
    if(self->preferredParentBodyLinkage_ == Unlinked && self->currentParentBodyLinkage_ == Unlinked){
        return false;
    }
    
    bool updated = false;
    BodyItem* parentBodyItem = nullptr;
    bool doClear = false;
    if(linkedParentBodyItem){
        if(self->preferredParentBodyLinkage_ == Unlinked){
            doClear = true;
        } else {
            parentBodyItem = self->findOwnerItem<BodyItem>();
            if(!parentBodyItem){
                doClear = true;
            }
        }
    }
    if(doClear){
        clearParentBodyLinkage(doNotifyUpdate);
        updated = true;
    } else {
        if(!parentBodyItem){
            parentBodyItem = self->findOwnerItem<BodyItem>();
        }
        if(parentBodyItem && updateParentBodyLinkageTo(parentBodyItem)){
            if(doNotifyUpdate){
                self->notifyUpdate();
            }
            updated = true;
        }
    }
    return updated;
}


void BodyItem::Impl::clearParentBodyLinkage(bool doNotifyUpdate)
{
    if(linkedParentBodyItem){
        if(attachmentToParent){
            if(attachmentToParent->isAttaching()){
                if(auto holder = attachmentToParent->holder()){
                    auto holderLink = holder->link();
                    mvout(false) <<
                        formatR(_("{0} has been detached from {1} of {2}."),
                                self->displayName(), holderLink->name(), holderLink->body()->name())
                                 << endl;
                }
                attachmentToParent->detach();
                attachmentToParent->on(false);
                if(doNotifyUpdate){
                    notifyKinematicStateChange(false, false, false, true);
                }
            }
            attachmentToParent = nullptr;
        }
        body->resetParent();
        linkedParentBodyItemConnection.disconnect();
        linkedParentBodyItem = nullptr;
        parentLink = nullptr;
        self->currentParentBodyLinkage_ = Unlinked;

        if(bodyLocation){
            bodyLocation->updateLocationType();
            // Notify the change of the parent location proxy
            bodyLocation->notifyAttributeChange();
        }
    }
}


bool BodyItem::Impl::updateParentBodyLinkageTo(BodyItem* parentBodyItem)
{
    Link* newParentLink = nullptr;
    AttachmentDevice* newAttachment = nullptr;
    HolderDevice* newHolder = nullptr;
    bool updated = false;

    if(self->preferredParentBodyLinkage_ != Unlinked){
        auto parentBody = parentBodyItem->body();
        bool isInvalidParentLink = false;
        if(!parentLinkName.empty()){
            newParentLink = parentBody->link(parentLinkName);
            if(!newParentLink){
                parentBodyItem = nullptr;
                isInvalidParentLink = true;
            }
        }
        if(!isInvalidParentLink){
            if(!newParentLink && self->preferredParentBodyLinkage_ == Attached){
                findMatchedAttachmentAndHolder(parentBodyItem, newAttachment, newHolder);
            }
            if(!newAttachment && parentLinkName.empty()){
                newParentLink = parentBody->rootLink();
            }
        }
    }
    if(!newAttachment){
        if(newParentLink != parentLink ||
           self->preferredParentBodyLinkage_ != self->currentParentBodyLinkage_){
            setLinkageToParentBody(parentBodyItem, newParentLink);
            updated = true;
        } else if(attachmentToParent){
            clearParentBodyLinkage(true);
            updated = true;
        }
    } else {
        bool doUpdateAttachmentDevice = false;
        if (self->preferredParentBodyLinkage_ != self->currentParentBodyLinkage_) {
            doUpdateAttachmentDevice = true;
        } else if (newAttachment != attachmentToParent) {
            doUpdateAttachmentDevice = true;
        } else {
            HolderDevice* holder = nullptr;
            if (attachmentToParent) {
                holder = attachmentToParent->holder();
            }
            if (newHolder != holder) {
                doUpdateAttachmentDevice = true;
            }
        }
        if (doUpdateAttachmentDevice) {
            setLinkageToParentBody(parentBodyItem, newAttachment, newHolder);
            updated = true;
        }
    }

    return updated;
}


bool BodyItem::Impl::findMatchedAttachmentAndHolder
(BodyItem* parentBodyItem, AttachmentDevice*& out_attachment, HolderDevice*& out_holder)
{
    for(auto& attachment : body->devices<AttachmentDevice>()){
        if(attachment->link()->isRoot()){
            for(auto& holder : parentBodyItem->body()->devices<HolderDevice>()){
                if(attachment->isAttachableTo(holder)){
                    out_attachment = attachment;
                    out_holder = holder;
                    return true;
                }
            }
        }
    }
    return false;
}


void BodyItem::Impl::setLinkageToParentBody(BodyItem* parentBodyItem, Link* newParentLink)
{
    clearParentBodyLinkage(true);
    parentLink = newParentLink;
    if(parentLink){
        setLocalPositionOnParentBodyLink();
    }
    setLinkageToParentBody(parentBodyItem, self->preferredParentBodyLinkage_);
}


void BodyItem::Impl::setLinkageToParentBody
(BodyItem* parentBodyItem, AttachmentDevice* attachment, HolderDevice* holder)
{
    clearParentBodyLinkage(true);

    if(attachment && holder){
        holder->addAttachment(attachment);
        holder->on(true);
        attachment->on(true);
        attachmentToParent = attachment;
        parentLink = holder->link();
        Isometry3 T_offset = holder->T_local() * attachment->T_local().inverse(Eigen::Isometry);
        body->rootLink()->setOffsetPosition(T_offset);
        mvout(false) <<
            formatR(_("{0} has been attached to {1} of {2}."),
                    self->displayName(), parentLink->name(), parentBodyItem->displayName()) << endl;
    }

    setLinkageToParentBody(parentBodyItem, Attached);
}


void BodyItem::Impl::setLinkageToParentBody(BodyItem* parentBodyItem, int linkageType)
{
    linkedParentBodyItem = parentBodyItem;

    if(linkedParentBodyItem && parentLink){
        self->currentParentBodyLinkage_ = linkageType;
        if(linkageType == Attached){
            body->setParent(parentLink);
        }
        linkedParentBodyItemConnection =
            linkedParentBodyItem->sigKinematicStateChanged().connect(
                [&](){ onParentBodyKinematicStateChanged(); });
        onParentBodyKinematicStateChanged();
    } 
    if(bodyLocation){
        bodyLocation->updateLocationType();
        // Notify the change of the parent location proxy
        bodyLocation->notifyAttributeChange();
    }
}    


void BodyItem::Impl::setLocalPositionOnParentBodyLink()
{
    if(isLocalPositionRestored){
        isLocalPositionRestored = false;
    } else if(parentLink) {
        auto T_inv = parentLink->T().inverse(Eigen::Isometry);
        auto rootLink = body->rootLink();
        rootLink->setOffsetPosition(T_inv * rootLink->T());
    }
}


void BodyItem::Impl::onParentBodyKinematicStateChanged()
{
    if(parentLink){
        auto rootLink = body->rootLink();
        rootLink->setPosition(parentLink->T() * rootLink->Tb());
        isKinematicStateChangeNotifiedByParentBodyItem = true;
        isProcessingInverseKinematicsIncludingParentBody = false;
        //! \todo requestVelFK and requestAccFK should be set appropriately
        notifyKinematicStateChange(true, false, false, true);
    }
}


MyCompositeBodyIK::MyCompositeBodyIK(BodyItem::Impl* bodyItemImpl)
    : bodyItemImpl(bodyItemImpl),
      parentLink(bodyItemImpl->parentLink)
{
    parentLinkIK = bodyItemImpl->linkedParentBodyItem->getCurrentIK(parentLink);
    T_local_inv = bodyItemImpl->body->rootLink()->offsetPosition().inverse(Eigen::Isometry);
}


std::shared_ptr<InverseKinematics> MyCompositeBodyIK::getParentBodyIK()
{
    return parentLinkIK;
}


bool MyCompositeBodyIK::isBestEffortIkAvailable() const
{
    return parentLinkIK ? parentLinkIK->isBestEffortIkAvailable() : false;
}


bool MyCompositeBodyIK::isBestEffortIkEnabled() const
{
    return parentLinkIK ? parentLinkIK->isBestEffortIkEnabled() : false;
}


void MyCompositeBodyIK::setBestEffortIkEnabled(bool on)
{
    if(parentLinkIK){
        parentLinkIK->setBestEffortIkEnabled(on);
    }
}


int MyCompositeBodyIK::getDOF() const
{
    return parentLinkIK ? parentLinkIK->getDOF() : 0;
}


bool MyCompositeBodyIK::calcInverseKinematics(const Isometry3& T)
{
    bool result = false;
    if(parentLinkIK){
        Isometry3 Tp = T * T_local_inv;
        result = parentLinkIK->calcInverseKinematics(Tp);
        if(result || parentLinkIK->isBestEffortIkEnabled()){
            bodyItemImpl->isProcessingInverseKinematicsIncludingParentBody = true;
        }
    }
    return result;
}


bool MyCompositeBodyIK::calcRemainingPartForwardKinematicsForInverseKinematics()
{
    bool result = false;
    if(parentLinkIK){
        if(!parentLinkIK->calcRemainingPartForwardKinematicsForInverseKinematics()){
            bodyItemImpl->linkedParentBodyItem->body()->calcForwardKinematics();
        }
        result = true;
    }
    return result;
}


void BodyItem::doPutProperties(PutPropertyFunction& putProperty)
{
    impl->doPutProperties(putProperty);
}


void BodyItem::Impl::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Model name"), body->modelName());
    putProperty(_("Num links"), body->numLinks());
    putProperty(_("Num joints"), body->numJoints());
    putProperty(_("Num devices"), (int)body->devices().size());
    putProperty(_("Root link"), body->rootLink()->name());
    putProperty(_("Base link"), currentBaseLink ? currentBaseLink->name() : "none");
    putProperty.decimals(3)(_("Mass"), body->mass());
    putProperty(_("Center of mass"), str(self->centerOfMass()));
    putProperty(_("Model type"), body->isStaticModel() ? _("Static") : _("Dynamic"));

    putProperty(_("Root fixed"), body->isFixedRootModel(),
                [this](bool on){
                    body->setRootLinkFixed(on);
                    notifyModelUpdate(LinkSpecUpdate);
                    return true;
                });

    if(isLinkableToParentBody()){
        Selection linkage = { _("Unlinked"), _("Coordinated"), _("Attached") };
        linkage.select(self->preferredParentBodyLinkage_);
        putProperty(_("Parent body linkage"), linkage,
                    [this](int linkageType){
                        return self->setPreferredParentBodyLinkage(linkageType, true);
                    });
    }
    putProperty(_("Parent link"), parentLinkName,
                [this](const string& name){ setParentLink(name); return true; });

    putProperty(_("Lock location"), self->isLocationLocked(),
                [this](bool on){ setLocationLocked(on, true, true); return true; });
    putProperty(_("Scene sensitive"), self->isSceneSensitive(),
                [this](bool on){ self->setSceneSensitive(on); return true; });

    if(body->numJoints() > 0){
        putProperty(_("Enable joint limits"), isJointRangeLimitEnabled,
                    [this](bool on){ self->setJointRangeLimitEnabled(on); return true; });
    }
    
    putProperty(_("Collision detection"), isCollisionDetectionEnabled,
                [this](bool on){ return setCollisionDetectionEnabled(on); });
    putProperty(_("Self-collision detection"), isSelfCollisionDetectionEnabled,
                [this](bool on){ return setSelfCollisionDetectionEnabled(on); });

    putProperty.range(0.0, 0.9).decimals(1);
    putProperty(_("Transparency"), transparency,
                [this](float value){ setTransparency(value); return true; });
    putProperty(_("Visible link selection"), self->isVisibleLinkSelectionMode_,
                changeProperty(self->isVisibleLinkSelectionMode_));

    putProperty(_("Multiplexing number"), body->numMultiplexBodies());
    putProperty(_("Existence"), body->existence(),
                [this](bool on){ body->setExistence(on); return true; });
}


bool BodyItem::store(Archive& archive)
{
    return impl->store(archive);
}


bool BodyItem::Impl::store(Archive& archive)
{
    archive.writeFileInformation(self);

    archive.setFloatingNumberFormat("%.9g");

    if(currentBaseLink){
        archive.write("currentBaseLink", currentBaseLink->name(), DOUBLE_QUOTED);
    }

    auto rootLink = body->rootLink();
    /// \todo Improve the following for current / initial position representations
    write(archive, "rootPosition", rootLink->p());
    write(archive, "rootAttitude", Matrix3(rootLink->R()));

    Listing* qs;
    
    // New format uses degree
    int n = body->numAllJoints();
    if(n > 0){
        bool doWriteInitialJointDisplacements = false;
        auto initialJointDisplacements = initialState.jointDisplacements();
        int numInitialJointDisplacements = initialState.numJointDisplacements();
        qs = archive.createFlowStyleListing("jointDisplacements");
        qs->setFloatingNumberFormat("%g");
        for(int i=0; i < n; ++i){
            double q = body->joint(i)->q();
            qs->append(degree(q), 10, n);
            if(!doWriteInitialJointDisplacements){
                if(i < numInitialJointDisplacements && q != initialJointDisplacements[i]){
                    doWriteInitialJointDisplacements = true;
                }
            }
        }
        if(doWriteInitialJointDisplacements){
            qs = archive.createFlowStyleListing("initialJointDisplacements");
            qs->setFloatingNumberFormat("%g");
            for(size_t i=0; i < numInitialJointDisplacements; ++i){
                qs->append(degree(initialJointDisplacements[i]), 10, n);
            }
        }
    }

    //! \todo replace the following code with the ValueTree serialization function of BodyState
    if(initialState.hasLinkPositions()){
        auto initialRootPosition = initialState.rootLinkPosition();
        write(archive, "initialRootPosition", initialRootPosition.translation());
        write(archive, "initialRootAttitude", Matrix3(initialRootPosition.rotation()));
    }

    archive.write("fix_root", body->isFixedRootModel());
    archive.write("collisionDetection", isCollisionDetectionEnabled);
    archive.write("selfCollisionDetection", isSelfCollisionDetectionEnabled);
    archive.write("lock_location", self->isLocationLocked());
    archive.write("scene_sensitive", self->isSceneSensitive());

    if(!isJointRangeLimitEnabled){
        archive.write("enable_joint_limits", isJointRangeLimitEnabled);
    }
    
    if(self->isVisibleLinkSelectionMode_){
        archive.write("visible_link_selection_mode", true);
    }

    if(kinematicsKitManager){
        MappingPtr kinematicsNode = new Mapping;
        if(kinematicsKitManager->storeState(*kinematicsNode) && !kinematicsNode->empty()){
            archive.insert("link_kinematics", kinematicsNode);
        }
    }

    if(isLinkableToParentBody()){
        const char* linkage = nullptr;
        if(self->preferredParentBodyLinkage_ == Unlinked){
            linkage = "unlinked";
        } else if(self->preferredParentBodyLinkage_ == Coordinated){
            linkage = "coordinated";
        } else if(self->preferredParentBodyLinkage_ == Attached){
            linkage = "attached";
        }
        if(linkage){
            archive.write("parent_body_linkage", linkage);
            write(archive, "local_translation", rootLink->offsetTranslation());
            writeDegreeAngleAxis(archive, "local_rotation", AngleAxis(rootLink->offsetRotation()));
        }
        if(self->preferredParentBodyLinkage_ == Attached){
            archive.write("enable_attachment", true); // For backward compatibility
        }
    }
    if(!parentLinkName.empty()){
        archive.write("parent_link", parentLinkName, DOUBLE_QUOTED);
    }

    if(transparency > 0.0f){
        archive.write("transparency", transparency);
    }

    return true;
}


bool BodyItem::restore(const Archive& archive)
{
    return impl->restore(archive);
}


bool BodyItem::Impl::restore(const Archive& archive)
{
    isBeingRestored = true;

    string filepath;
    if(archive.read({ "file", "modelFile" }, filepath)){
        bool loaded = false;
        filepath = archive.resolveRelocatablePath(filepath);
        if(!filepath.empty()){
            loaded = archive.loadFileTo(self, filepath);
        }
        if(!loaded){
            isBeingRestored = false;
            return false;
        }
    }

    auto rootLink = body->rootLink();
    Vector3 p = Vector3::Zero();
    Matrix3 R = Matrix3::Identity();
        
    if(read(archive, "rootPosition", p)){
        rootLink->p() = p;
    }
    if(read(archive, "rootAttitude", R)){
        rootLink->R() = R;
    }

    //! \todo replace the following code with the ValueTree serialization function of BodyState
    initialState.allocate(1, 0);

    read(archive, "initialRootPosition", p);
    read(archive, "initialRootAttitude", R);
    initialState.rootLinkPosition().set(SE3(p, R));

    restoreNonRootLinkStates(archive);

    bool on;
    if(archive.read("fix_root", on)){
        body->setRootLinkFixed(on);
    } else if(archive.read("staticModel", on)){ // Old format
        // Incomplete restoration for the old format
        if(on){
            body->setRootLinkFixed(true);
        } else if(body->isStaticModel()){
            body->setRootLinkFixed(false);
        }
    }
    if(archive.read("collisionDetection", on)){
        setCollisionDetectionEnabled(on);
    }
    if(archive.read("selfCollisionDetection", on)){
        setSelfCollisionDetectionEnabled(on);
    }
    if(archive.read("lock_location", on)){
        setLocationLocked(on, false, false);
    } else if(archive.read("location_editable", on) ||
       archive.read("isEditable", on) ||
       archive.read("isSceneBodyDraggable", on)){
        setLocationLocked(!on, false, false);
    }
    if(archive.read("scene_sensitive", on)){
        self->setSceneSensitive(on);
    }
    if(archive.read("enable_joint_limits", on)){
        self->setJointRangeLimitEnabled(on);
    }
    archive.read("visible_link_selection_mode", self->isVisibleLinkSelectionMode_);

    string linkage;
    if(archive.read("parent_body_linkage", linkage)){
        if(linkage == "unlinked"){
            self->preferredParentBodyLinkage_ = Unlinked;
        } else if(linkage == "coordinated"){
            self->preferredParentBodyLinkage_ = Coordinated;
        } else if(linkage == "attached"){
            self->preferredParentBodyLinkage_ = Attached;
        } else {
            archive.throwException(_("Unknown parent body linkage type"));
        }
        if(self->preferredParentBodyLinkage_ == Coordinated ||
           self->preferredParentBodyLinkage_ == Attached){
            Vector3 translation;
            if(read(archive, "local_translation", translation)){
                rootLink->setOffsetTranslation(translation);
                isLocalPositionRestored = true;
            }
            AngleAxis rot;
            if(readDegreeAngleAxis(archive, "local_rotation", rot)){
                rootLink->setOffsetRotation(rot);
                isLocalPositionRestored = true;
            }
        }
    } else if(archive.read("enable_attachment", on)){ // For backward compatibility
        self->preferredParentBodyLinkage_ = on ? Attached : Unlinked;
    }
    archive.read("parent_link", parentLinkName);

    double t;
    if(archive.read("transparency", t)){
        setTransparency(t);
    }

    isUpdateNotificationOnSubTreeRestoredRequested = false;
    isNonRootLinkStateRestorationOnSubTreeRestoredRequested = false;

    archive.addProcessOnSubTreeRestored(
        [this, &archive](){
            isBeingRestored = false;

            bool doNotifyUpdate = false;
            bool doNotifyKinematicStateChange = false;

            // The attachment is updated after the sub tree is restored
            if(updateParentBodyLinkage(false)){
                doNotifyUpdate = true;
            }
            isLocalPositionRestored = false;
            
            if(isNonRootLinkStateRestorationOnSubTreeRestoredRequested){
                restoreNonRootLinkStates(archive);
                doNotifyUpdate = true;
                doNotifyKinematicStateChange = true;
                isNonRootLinkStateRestorationOnSubTreeRestoredRequested = false;
            }
            if(isUpdateNotificationOnSubTreeRestoredRequested){
                doNotifyUpdate = true;
                isUpdateNotificationOnSubTreeRestoredRequested = false;
            }

            if(doNotifyUpdate || doNotifyKinematicStateChange){
                // Update notifications should be done after all the post processes
                archive.addProcessOnSubTreeRestored(
                    [this, doNotifyUpdate, doNotifyKinematicStateChange](){
                        if(doNotifyUpdate){
                            self->notifyUpdate();
                        }
                        if(doNotifyKinematicStateChange){
                            self->notifyKinematicStateChange(true);
                        }
                    });
            }
        });

    return true;
}


void BodyItem::Impl::restoreNonRootLinkStates(const Archive& archive)
{
    Listing* qs;
    bool useNewJointDisplacementFormat = false;

    qs = archive.findListing("jointDisplacements");
    Listing* qs_initial = archive.findListing("initialJointDisplacements");
    if(qs->isValid()){
        useNewJointDisplacementFormat = true;
        int nj = std::min(qs->size(), body->numAllJoints());
        initialState.allocate(1, nj);
        auto q_initial = initialState.jointDisplacements();
        for(int i=0; i < nj; ++i){
            double q = radian((*qs)[i].toDouble());
            body->joint(i)->q() = q;
            if(qs_initial->isValid() && i < qs_initial->size()){
                q_initial[i] = radian((*qs_initial)[i].toDouble());
            } else {
                q_initial[i] = q;
            }
        }
    }

    if(!useNewJointDisplacementFormat){
        qs = archive.findListing("jointPositions");
        if(qs->isValid()){
            int nj = body->numAllJoints();
            if(qs->size() != nj){
                if(qs->size() != body->numJoints()){
                    MessageView::instance()->putln(
                        formatR(_("Mismatched size of the stored joint positions for {}"), self->displayName()),
                        MessageView::Warning);
                }
                nj = std::min(qs->size(), nj);
            }
            for(int i=0; i < nj; ++i){
                body->joint(i)->q() = (*qs)[i].toDouble();
            }
        }
        qs = archive.findListing("initialJointPositions");
        if(qs->isValid()){
            int n = body->numAllJoints();
            int m = qs->size();
            if(m != n){
                if(m != body->numJoints()){
                    MessageView::instance()->putln(
                        formatR(_("Mismatched size of the stored initial joint positions for {}"), self->displayName()),
                        MessageView::Warning);
                }
                m = std::min(m, n);
            }
            initialState.allocate(1, n);
            auto q_initial = initialState.jointDisplacements();
            for(int i = 0; i < m; ++i){
                q_initial[i] = (*qs)[i].toDouble();
            }
            for(int i = m; i < n; ++i){
                q_initial[i] = body->joint(i)->q();
            }
        }
    }

    body->calcForwardKinematics();
    string baseLinkName;
    if(archive.read("currentBaseLink", baseLinkName)){
        setCurrentBaseLink(body->link(baseLinkName), false, false);
    }

    auto kinematicsNode = archive.findMapping("link_kinematics");
    if(kinematicsNode->isValid()){
        getOrCreateKinematicsKitManager()->restoreState(*kinematicsNode);
    }
}


bool BodyItem::isBeingRestored() const
{
    return impl->isBeingRestored;
}


void BodyItem::requestUpdateNotificationOnSubTreeRestored()
{
    impl->isUpdateNotificationOnSubTreeRestoredRequested = true;
}


void BodyItem::requestNonRootLinkStatesRestorationOnSubTreeRestored()
{
    impl->isNonRootLinkStateRestorationOnSubTreeRestoredRequested = true;
}


RenderableItemUtil* BodyItem::Impl::getOrCreateRenderableItemUtil()
{
    if(!renderableItemUtil){
        renderableItemUtil = make_unique<RenderableItemUtil>();
    }
    renderableItemUtil->setItem(self);
    return renderableItemUtil.get();
}


void BodyItem::getDependentFiles(std::vector<std::string>& out_files)
{
    auto& fp = filePath();
    if(!fp.empty()){
        out_files.push_back(fp);
    }
    auto util = impl->getOrCreateRenderableItemUtil();
    for(auto& link : impl->body->links()){
        util->getSceneFilesForArchiving(link->shape(), out_files);
        if(link->hasDedicatedCollisionShape()){
            util->getSceneFilesForArchiving(link->collisionShape(), out_files);
        }
    }
}


void BodyItem::relocateDependentFiles
(std::function<std::string(const std::string& path)> getRelocatedFilePath)
{
    auto util = impl->getOrCreateRenderableItemUtil();
    util->initializeSceneObjectUrlRelocation();
    for(auto& link : impl->body->links()){
        util->relocateSceneObjectUris(link->shape(), getRelocatedFilePath);
        if(link->hasDedicatedCollisionShape()){
            util->relocateSceneObjectUris(link->collisionShape(), getRelocatedFilePath);
        }
    }
}


KinematicStateRecord::KinematicStateRecord(BodyItem::Impl* bodyItemImpl)
    : EditRecord(bodyItemImpl->self),
      bodyItem(bodyItemImpl->self),
      bodyItemImpl(bodyItemImpl)
{
    bodyItem->storeKinematicState(newState);
    oldState = newState;
}


KinematicStateRecord::KinematicStateRecord(BodyItem::Impl* bodyItemImpl, const BodyState& oldState)
    : EditRecord(bodyItemImpl->self),
      bodyItem(bodyItemImpl->self),
      bodyItemImpl(bodyItemImpl),
      oldState(oldState)
{
    bodyItem->storeKinematicState(newState);
}


KinematicStateRecord::KinematicStateRecord(const KinematicStateRecord& org)
    : EditRecord(org),
      bodyItem(org.bodyItem),
      bodyItemImpl(org.bodyItemImpl),
      newState(org.newState),
      oldState(org.oldState)
{

}


EditRecord* KinematicStateRecord::clone() const
{
    return new KinematicStateRecord(*this);
}


std::string KinematicStateRecord::label() const
{
    if(!isReverse()){
        return formatR(_("Change the position of \"{0}\""), bodyItem->displayName());
    } else {
        return formatR(_("Restore the position of \"{0}\""), bodyItem->displayName());
    }
}


bool KinematicStateRecord::undo()
{
    bodyItem->restoreKinematicState(oldState);
    bodyItem->storeKinematicState(bodyItemImpl->lastEditState);
    bodyItemImpl->notifyKinematicStateChange(false, false, false, true);
    return true;
}


bool KinematicStateRecord::redo()
{
    bodyItem->restoreKinematicState(newState);
    bodyItem->storeKinematicState(bodyItemImpl->lastEditState);
    bodyItemImpl->notifyKinematicStateChange(false, false, false, true);
    return true;
}
