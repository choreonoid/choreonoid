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
#include <fmt/format.h>
#include <bitset>
#include <algorithm>
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

const bool TRACE_FUNCTIONS = false;

BodyState kinematicStateCopy;

class BodyLocation : public LocationProxy
{
public:
    BodyItem::Impl* impl;

    BodyLocation(BodyItem::Impl* impl);
    void updateLocationType();
    virtual Isometry3 getLocation() const override;
    virtual bool isLocked() const override;
    virtual void setLocked(bool on) override;
    virtual bool isDoingContinuousUpdate() const override;
    virtual bool setLocation(const Isometry3& T) override;
    virtual void finishLocationEditing() override;
    virtual Item* getCorrespondingItem() override;
    virtual LocationProxyPtr getParentLocationProxy() const override;
    virtual SignalProxy<void()> sigLocationChanged() override;
};
    
class LinkLocation : public LocationProxy
{
public:
    weak_ref_ptr<BodyItem> refBodyItem;
    weak_ref_ptr<Link> refLink;

    LinkLocation();
    LinkLocation(BodyItem* bodyItem, Link* link);
    void setTarget(BodyItem* bodyItem, Link* link);
    virtual std::string getName() const override;
    virtual Isometry3 getLocation() const override;
    virtual bool isLocked() const override;
    virtual Item* getCorrespondingItem() override;
    virtual LocationProxyPtr getParentLocationProxy() const override;
    virtual SignalProxy<void()> sigLocationChanged() override;
};

class MyCompositeBodyIK : public CompositeBodyIK
{
public:
    MyCompositeBodyIK(BodyItem::Impl* bodyItemImpl);
    virtual bool calcInverseKinematics(const Isometry3& T) override;
    virtual std::shared_ptr<InverseKinematics> getParentBodyIK() override;

    BodyItem::Impl* bodyItemImpl;
    AttachmentDevicePtr attachment;
    shared_ptr<InverseKinematics> holderIK;
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
    bool isUpdateNotificationOnSubTreeRestoredRequested;
    bool isNonRootLinkStateRestorationOnSubTreeRestoredRequested;
    bool isSharingShapes;
    bool isLocationLocked;
    bool isKinematicStateChangeNotifiedByParentBodyItem;
    bool isProcessingInverseKinematicsIncludingParentBody;
    bool isAttachmentEnabled;
    bool isFkRequested;
    bool isVelFkRequested;
    bool isAccFkRequested;
    bool isCollisionDetectionEnabled;
    bool isSelfCollisionDetectionEnabled;

    ScopedConnection bodyExistenceConnection;
    
    BodyItem* parentBodyItem;
    ref_ptr<BodyLocation> bodyLocation;
    ref_ptr<LinkLocation> parentLinkLocation;
    AttachmentDevicePtr attachmentToParent;
    ScopedConnection parentBodyItemConnection;

    enum { UF_POSITIONS, UF_VELOCITIES, UF_ACCELERATIONS, UF_CM, UF_ZMP, NUM_UPUDATE_ELEMENTS };
    std::bitset<NUM_UPUDATE_ELEMENTS> updateElements;

    LazySignal<Signal<void()>> sigKinematicStateChanged;
    Signal<void()> sigKinematicStateUpdated;

    LinkPtr currentBaseLink;
    LinkTraverse fkTraverse;
    unique_ptr<BodyItemKinematicsKitManager> kinematicsKitManager;
    shared_ptr<PinDragIK> pinDragIK;

    BodyState initialState;
    BodyState lastEditState;
            
    OperableSceneBodyPtr sceneBody;
    float transparency;
    Signal<void(int flags)> sigModelUpdated;

    Signal<void(bool on)> sigContinuousKinematicUpdateStateChanged;

    LeggedBodyHelperPtr legged;
    Vector3 zmp;

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
    bool updateAttachment(bool on, bool doNotifyUpdate);
    bool isAttachable() const;
    void setParentBodyItem(BodyItem* bodyItem);
    Link* attachToBodyItem(BodyItem* bodyItem);
    void setRelativeOffsetPositionFromParentBody();
    void onParentBodyKinematicStateChanged();
    void doPutProperties(PutPropertyFunction& putProperty);
    bool store(Archive& archive);
    bool restore(const Archive& archive);
    void restoreNonRootLinkStates(const Archive& archive);
    RenderableItemUtil* getOrCreateRenderableItemUtil();
};

unique_ptr<RenderableItemUtil> BodyItem::Impl::renderableItemUtil;

}


static void onSigOptionsParsed(boost::program_options::variables_map& variables)
{
    if(variables.count("body")){
    	vector<string> bodyFileNames = variables["body"].as<vector<string>>();
    	for(size_t i=0; i < bodyFileNames.size(); ++i){
            BodyItemPtr item(new BodyItem);
            auto rootItem = RootItem::instance();
            if(item->load(bodyFileNames[i], rootItem, "CHOREONOID-BODY")){
                item->setChecked(true);
                rootItem->addChildItem(item);
            }
    	}
    }
}


void BodyItem::initializeClass(ExtensionManager* ext)
{
    ItemManager* im = &ext->itemManager();
    im->registerClass<BodyItem>(N_("BodyItem"));

    // Implemented in BodyItemFileIO.cpp
    registerBodyItemFileIoSet(im);

    OptionManager& om = ext->optionManager();
    om.addOption("body", boost::program_options::value< vector<string> >(), "load a body file");
    om.sigOptionsParsed(1).connect(onSigOptionsParsed);
}


BodyItem::BodyItem()
{
    setAttributes(FileImmutable | Reloadable);

    impl = new Impl(this);
    impl->init(false);

    continuousKinematicUpdateCounter = 0;
    isAttachedToParentBody_ = false;
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
    isAttachmentEnabled = true;
    isCollisionDetectionEnabled = true;
    isSelfCollisionDetectionEnabled = false;
}


BodyItem::Impl::Impl(BodyItem* self, Body* body, bool isSharingShapes)
    : self(self),
      body(body),
      isBeingRestored(false),
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

    continuousKinematicUpdateCounter = 0;
    isAttachedToParentBody_ = false;
    isVisibleLinkSelectionMode_ = org.isVisibleLinkSelectionMode_;

    setChecked(org.isChecked());
}


BodyItem::Impl::Impl(BodyItem* self, const Impl& org, CloneMap* cloneMap)
    : Impl(self, CloneMap::getClone(org.body, cloneMap), true)
{
    isAttachmentEnabled = org.isAttachmentEnabled;
    transparency = org.transparency;
    zmp = org.zmp;
    isCollisionDetectionEnabled = org.isCollisionDetectionEnabled;
    isSelfCollisionDetectionEnabled = org.isSelfCollisionDetectionEnabled;

    if(org.currentBaseLink){
        setCurrentBaseLink(body->link(org.currentBaseLink->index()), true, false);
    } else {
        setCurrentBaseLink(nullptr, true, false);
    }

    initialState = org.initialState;
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
    parentBodyItem = nullptr;
    isKinematicStateChangeNotifiedByParentBodyItem = false;
    isProcessingInverseKinematicsIncludingParentBody = false;
    
    if(pinDragIK){
        pinDragIK.reset();
    }

    if(!calledFromCopyConstructor){
        setCurrentBaseLink(nullptr, true, false);
        zmp.setZero();
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
    self->isVisibleLinkSelectionMode_ = srcBodyItem->isVisibleLinkSelectionMode_;
    isAttachmentEnabled = srcImpl->isAttachmentEnabled;
    isLocationLocked = srcImpl->isLocationLocked;
    transparency = srcImpl->transparency;
    zmp = srcImpl->zmp;
    isCollisionDetectionEnabled = srcImpl->isCollisionDetectionEnabled;
    isSelfCollisionDetectionEnabled = srcImpl->isSelfCollisionDetectionEnabled;
    
    // copy the base link property
    Link* baseLink = nullptr;
    Link* srcBaseLink = srcBodyItem->currentBaseLink();
    if(srcBaseLink){
        baseLink = body->link(srcBaseLink->name());
        if(baseLink){
            setCurrentBaseLink(baseLink, false, false);
        }
    }
    // copy the current kinematic state
    Body* srcBody = srcBodyItem->body();
    for(int i=0; i < srcBody->numLinks(); ++i){
        Link* srcLink = srcBody->link(i);
        Link* link = body->link(srcLink->name());
        if(link){
            link->q() = srcLink->q();
        }
    }
    if(baseLink){
        baseLink->p() = srcBaseLink->p();
        baseLink->R() = srcBaseLink->R();
    } else {
        body->rootLink()->p() = srcBody->rootLink()->p();
        body->rootLink()->R() = srcBody->rootLink()->R();
    }
    
    initialState = srcImpl->initialState;
    
    self->notifyKinematicStateChange(true);

    return true;
}


void BodyItem::onTreePathChanged()
{
    auto worldItem = findOwnerItem<WorldItem>();
    if(!worldItem){
        clearCollisions();
    }

    /*
      If the item is being restored in loading a project, the attachment update is
      processed when all the child items are restored so that an attachment device
      dynamically added by a DeviceOverwriteItem can work in the attachment update.
      In that case, the attachment update is processed by the callback function
      given to the Archive::addProcessOnSubTreeRestored in the restore function.
    */
    if(!impl->isBeingRestored){
        impl->updateAttachment(true, true);
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
    if(flags & LinkSetUpdate){
        resetLinkCollisions();
        setCurrentBaseLink(currentBaseLink, true, false);
        if(kinematicsKitManager){
            kinematicsKitManager->clearKinematicsKits();
        }
    }

    if(sceneBody){
        if(flags & (LinkSetUpdate | LinkSpecUpdate | ShapeUpdate)){
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


void BodyItem::copyKinematicState()
{
    storeKinematicState(kinematicStateCopy);
}


void BodyItem::pasteKinematicState()
{
    restoreKinematicState(kinematicStateCopy);
    notifyKinematicStateChange(false);    
}


void BodyItem::storeKinematicState(BodyState& state)
{
    state.storePositions(*impl->body);
    state.setZMP(impl->zmp);
}


/**
   @return false if the restored state is same as the current state
*/
bool BodyItem::restoreKinematicState(const BodyState& state)
{
    state.getZMP(impl->zmp);
    state.restorePositions(*impl->body);
    return true;
}


void BodyItem::storeInitialState()
{
    Item::setConsistentWithProjectArchive(false);
    storeKinematicState(impl->initialState);
}


void BodyItem::restoreInitialState(bool doNotify)
{
    bool restored = restoreKinematicState(impl->initialState);
    if(restored && doNotify){
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
    }
    return kinematicsKitManager.get();
}


BodyItemKinematicsKit* BodyItem::findPresetKinematicsKit(Link* targetLink)
{
    return impl->getOrCreateKinematicsKitManager()->findPresetKinematicsKit(targetLink);
}
    

std::shared_ptr<InverseKinematics> BodyItem::findPresetIK(Link* targetLink)
{
    if(isAttachedToParentBody_ && targetLink->isFixedToRoot()){
        return make_shared<MyCompositeBodyIK>(impl);
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
    if(isAttachedToParentBody_ && targetLink->isFixedToRoot()){
        ik = make_shared<MyCompositeBodyIK>(impl);
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
        pos = zmp;

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


const Vector3& BodyItem::zmp() const
{
    return impl->zmp;
}


void BodyItem::setZmp(const Vector3& zmp)
{
    impl->zmp = zmp;
}


void BodyItem::editZmp(const Vector3& zmp)
{
    setZmp(zmp);
    notifyKinematicStateUpdate();
}


void BodyItem::Impl::notifyKinematicStateChange(bool requestFK, bool requestVelFK, bool requestAccFK, bool isDirect)
{
    updateElements.reset();

    if(isProcessingInverseKinematicsIncludingParentBody){
        isProcessingInverseKinematicsIncludingParentBody = false;
        if(parentBodyItem){
            parentBodyItem->impl->notifyKinematicStateChange(
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
        if(parentBodyItem && !attachmentToParent){
            setRelativeOffsetPositionFromParentBody();
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

    if(isAttachedToParentBody_){
        impl->parentBodyItem->notifyKinematicStateUpdate(false);
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
        worldItem->updateCollisionDetectorLater();
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


void BodyItem::clearCollisions()
{
    collisions_.clear();

    for(size_t i=0; i < collisionLinkBitSet_.size(); ++i){
        if(collisionLinkBitSet_[i]){
            collisionsOfLink_[i].clear();
            collisionLinkBitSet_[i] = false;
        }
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
    return impl->isLocationLocked || isAttachedToParentBody_;
}


void BodyItem::setLocationLocked(bool on)
{
    impl->setLocationLocked(on, true, true);
}


void BodyItem::Impl::setLocationLocked(bool on, bool updateInitialPositionWhenLocked, bool doNotiyUpdate)
{
    if(self->isAttachedToParentBody_ && !on){
        return;
    }
    
    if(on != isLocationLocked){
        isLocationLocked = on;

        if(on && updateInitialPositionWhenLocked){
            initialState.setRootLinkPosition(body->rootLink()->T());
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
    : LocationProxy(impl->attachmentToParent ? OffsetLocation : GlobalLocation),
      impl(impl)
{

}


void BodyLocation::updateLocationType()
{
    if(impl->attachmentToParent){
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


bool BodyLocation::isDoingContinuousUpdate() const
{
    return impl->self->isDoingContinuousKinematicUpdate();
}


bool BodyLocation::setLocation(const Isometry3& T)
{
    auto rootLink = impl->body->rootLink();
    switch(locationType()){
    case OffsetLocation:
        rootLink->setOffsetPosition(T);
        impl->parentBodyItem->notifyKinematicStateChange(true);
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


Item* BodyLocation::getCorrespondingItem()
{
    return impl->self;
}


LocationProxyPtr BodyLocation::getParentLocationProxy() const
{
    if(impl->parentBodyItem){
        if(impl->attachmentToParent){
            if(!impl->parentLinkLocation){
                impl->parentLinkLocation = new LinkLocation;
            }
            auto parentLink = impl->body->parentBodyLink();
            impl->parentLinkLocation->setTarget(impl->parentBodyItem, parentLink);
            return impl->parentLinkLocation;
        } else {
            return impl->parentBodyItem->getLocationProxy();
        }
    }
    return nullptr;
}


SignalProxy<void()> BodyLocation::sigLocationChanged()
{
    return impl->sigKinematicStateChanged.signal();
}


LinkLocation::LinkLocation()
    : LocationProxy(GlobalLocation)
{

}


LinkLocation::LinkLocation(BodyItem* bodyItem, Link* link)
    : LocationProxy(GlobalLocation),
      refBodyItem(bodyItem),
      refLink(link)
{

}


void LinkLocation::setTarget(BodyItem* bodyItem, Link* link)
{
    refBodyItem = bodyItem;
    refLink = link;
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


Item* LinkLocation::getCorrespondingItem()
{
    return refBodyItem.lock();
}


LocationProxyPtr LinkLocation::getParentLocationProxy() const
{
    if(auto body = refBodyItem.lock()){
        body->getLocationProxy();
    }
    return nullptr;
}


SignalProxy<void()> LinkLocation::sigLocationChanged()
{
    if(auto bodyItem = refBodyItem.lock()){
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


BodyItem* BodyItem::parentBodyItem()
{
    return impl->parentBodyItem;
}


void BodyItem::setAttachmentEnabled(bool on, bool doNotifyUpdate)
{
    impl->isAttachmentEnabled = on;
    if(on != isAttachedToParentBody_){
        impl->updateAttachment(on, doNotifyUpdate);
    }
}


bool BodyItem::Impl::updateAttachment(bool on, bool doNotifyUpdate)
{
    bool updated = false;
    BodyItem* newParentBodyItem = nullptr;
    if(on && isAttachmentEnabled){
        newParentBodyItem = self->findOwnerItem<BodyItem>();
    }
    if(newParentBodyItem != parentBodyItem || (newParentBodyItem && !self->isAttachedToParentBody_)){
        setParentBodyItem(newParentBodyItem);
        updated = true;
        if(doNotifyUpdate){
            self->notifyUpdate();
        }
    }
    return updated;
}


bool BodyItem::isAttachmentEnabled() const
{
    return impl->isAttachmentEnabled;
}


bool BodyItem::Impl::isAttachable() const
{
    for(auto& attachment : body->devices<AttachmentDevice>()){
        if(attachment->link()->isRoot()){
            return true;
        }
    }
    return false;
}


bool BodyItem::attachToParentBody(bool doNotifyUpdate)
{
    if(!impl->isAttachmentEnabled){
        setAttachmentEnabled(true, doNotifyUpdate);
    } else {
        impl->updateAttachment(true, doNotifyUpdate);
    }
    return isAttachedToParentBody_;
}


/*
void BodyItem::resetParentBodyItem()
{
    impl->setParentBodyItem(findOwnerItem<BodyItem>());
}
*/


void BodyItem::Impl::setParentBodyItem(BodyItem* bodyItem)
{
    if(!bodyItem){
        if(attachmentToParent){
            auto holderLink = attachmentToParent->holder()->link();
            mvout(false) << format(_("{0} has been detached from {1} of {2}."),
                                   self->displayName(), holderLink->name(), holderLink->body()->name()) << endl;
        }
    }

    parentBodyItem = bodyItem;
    body->resetParent();
    parentBodyItemConnection.disconnect();
    attachmentToParent = nullptr;
    self->isAttachedToParentBody_ = false;
    bool detached = false;
    for(auto& attachment : body->devices<AttachmentDevice>()){
        if(attachment->isAttaching()){
            attachment->detach();
            detached = true;
        }
        attachment->on(false);
    }

    if(parentBodyItem){
        auto linkToAttach = attachToBodyItem(parentBodyItem);
        if(!linkToAttach){
            setRelativeOffsetPositionFromParentBody();
        }
        parentBodyItemConnection =
            parentBodyItem->sigKinematicStateChanged().connect(
                [&](){ onParentBodyKinematicStateChanged(); });
        onParentBodyKinematicStateChanged();

    } else if(detached){
        setLocationLocked(false, false, true);
        notifyKinematicStateChange(false, false, false, true);
    }

    if(bodyLocation){
        bodyLocation->updateLocationType();
        // Notify the change of the parent location proxy
        bodyLocation->notifyAttributeChange();
    }
}


Link* BodyItem::Impl::attachToBodyItem(BodyItem* bodyItem)
{
    Link* linkToAttach = nullptr;
    for(auto& attachment : body->devices<AttachmentDevice>()){
        if(attachment->link()->isRoot()){
            for(auto& holder : bodyItem->body()->devices<HolderDevice>()){
                if(attachment->category() == holder->category()){
                    holder->addAttachment(attachment);
                    holder->on(true);
                    attachment->on(true);
                    attachmentToParent = attachment;
                    self->isAttachedToParentBody_ = true;
                    linkToAttach = holder->link();
                    Isometry3 T_offset = holder->T_local() * attachment->T_local().inverse(Eigen::Isometry);
                    body->rootLink()->setOffsetPosition(T_offset);
                    body->setParent(linkToAttach);
                    setLocationLocked(true, false, true);
                    mvout(false) << format(_("{0} has been attached to {1} of {2}."),
                                           self->displayName(), linkToAttach->name(), bodyItem->displayName()) << endl;
                    goto found;
                }
            }
        }
    }
found:
    return linkToAttach;
}


void BodyItem::Impl::setRelativeOffsetPositionFromParentBody()
{
    auto rootLink = body->rootLink();
    auto T_inv = parentBodyItem->body()->rootLink()->T().inverse(Eigen::Isometry);
    rootLink->setOffsetPosition(T_inv * rootLink->T());
}


void BodyItem::Impl::onParentBodyKinematicStateChanged()
{
    Link* parentLink;
    if(attachmentToParent){
        parentLink = attachmentToParent->holder()->link();
    } else {
        parentLink = parentBodyItem->body()->rootLink();
    }
    auto rootLink = body->rootLink();
    rootLink->setPosition(parentLink->T() * rootLink->Tb());

    isKinematicStateChangeNotifiedByParentBodyItem = true;
    isProcessingInverseKinematicsIncludingParentBody = false;
    //! \todo requestVelFK and requestAccFK should be set appropriately
    notifyKinematicStateChange(true, false, false, true);
}


BodyItem::ContinuousKinematicUpdateEntry BodyItem::startContinuousKinematicUpdate()
{
    return new ContinuousKinematicUpdateRef(this);
}


SignalProxy<void(bool on)> BodyItem::sigContinuousKinematicUpdateStateChanged()
{
    return impl->sigContinuousKinematicUpdateStateChanged;
}


BodyItem::ContinuousKinematicUpdateRef::ContinuousKinematicUpdateRef(BodyItem* item)
    : bodyItemRef(item)
{
    if(++item->continuousKinematicUpdateCounter == 1){
        item->impl->sigContinuousKinematicUpdateStateChanged(true);
        if(auto& bodyLocation = item->impl->bodyLocation){
            bodyLocation->notifyAttributeChange();
        }
    }
}


BodyItem::ContinuousKinematicUpdateRef::~ContinuousKinematicUpdateRef()
{
    if(auto item = bodyItemRef.lock()){
        if(--item->continuousKinematicUpdateCounter == 0){
            item->impl->sigContinuousKinematicUpdateStateChanged(false);
            if(auto& bodyLocation = item->impl->bodyLocation){
                bodyLocation->notifyAttributeChange();
            }
        }
    }
}


MyCompositeBodyIK::MyCompositeBodyIK(BodyItem::Impl* bodyItemImpl)
    : bodyItemImpl(bodyItemImpl),
      attachment(bodyItemImpl->attachmentToParent)
{
    auto holderLink = attachment->holder()->link();
    holderIK = bodyItemImpl->parentBodyItem->getCurrentIK(holderLink);
}


bool MyCompositeBodyIK::calcInverseKinematics(const Isometry3& T)
{
    bool result = false;
    if(holderIK){
        if(auto holder = attachment->holder()){
            Isometry3 Ta = T * attachment->T_local() * holder->T_local().inverse(Eigen::Isometry);
            result = holderIK->calcInverseKinematics(Ta);
            if(result){
                bodyItemImpl->isProcessingInverseKinematicsIncludingParentBody = true;
            }
        }
    }
    return result;
}


std::shared_ptr<InverseKinematics> MyCompositeBodyIK::getParentBodyIK()
{
    return holderIK;
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
    
    putProperty(_("Collision detection"), isCollisionDetectionEnabled,
                [this](bool on){ return setCollisionDetectionEnabled(on); });
    putProperty(_("Self-collision detection"), isSelfCollisionDetectionEnabled,
                [this](bool on){ return setSelfCollisionDetectionEnabled(on); });
    putProperty(_("Lock location"), self->isLocationLocked(),
                [this](bool on){ setLocationLocked(on, true, true); return true; });
    putProperty(_("Scene sensitive"), self->isSceneSensitive(),
                [this](bool on){ self->setSceneSensitive(on); return true; });
    putProperty.range(0.0, 0.9).decimals(1);
    putProperty(_("Transparency"), transparency,
                [this](float value){ setTransparency(value); return true; });
    putProperty(_("Visible link selection"), self->isVisibleLinkSelectionMode_,
                changeProperty(self->isVisibleLinkSelectionMode_));

    if(isAttachable()){
        putProperty(_("Enable attachment"), isAttachmentEnabled,
                    [this](bool on){ self->setAttachmentEnabled(on, false); return true; });
    }

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

    /// \todo Improve the following for current / initial position representations
    write(archive, "rootPosition", body->rootLink()->p());
    write(archive, "rootAttitude", Matrix3(body->rootLink()->R()));

    Listing* qs;
    
    // New format uses degree
    int n = body->numAllJoints();
    if(n > 0){
        bool doWriteInitialJointDisplacements = false;
        BodyState::Data& initialJointDisplacements = initialState.data(BodyState::JOINT_POSITIONS);
        qs = archive.createFlowStyleListing("jointDisplacements");
        qs->setFloatingNumberFormat("%g");
        for(int i=0; i < n; ++i){
            double q = body->joint(i)->q();
            qs->append(degree(q), 10, n);
            if(!doWriteInitialJointDisplacements){
                if(i < initialJointDisplacements.size() && q != initialJointDisplacements[i]){
                    doWriteInitialJointDisplacements = true;
                }
            }
        }
        if(doWriteInitialJointDisplacements){
            qs = archive.createFlowStyleListing("initialJointDisplacements");
            qs->setFloatingNumberFormat("%g");
            for(size_t i=0; i < initialJointDisplacements.size(); ++i){
                qs->append(degree(initialJointDisplacements[i]), 10, n);
            }
        }
    }

    // Old format. Remove this after version 1.8 is released.
    qs = archive.createFlowStyleListing("jointPositions");
    qs->setFloatingNumberFormat("%g");
    n = body->numAllJoints();
    for(int i=0; i < n; ++i){
        qs->append(body->joint(i)->q(), 10, n);
    }

    //! \todo replace the following code with the ValueTree serialization function of BodyState
    SE3 initialRootPosition;
    if(initialState.getRootLinkPosition(initialRootPosition)){
        write(archive, "initialRootPosition", initialRootPosition.translation());
        write(archive, "initialRootAttitude", Matrix3(initialRootPosition.rotation()));
    }

    // Old format. Remove this after version 1.8 is released.
    BodyState::Data& initialJointPositions = initialState.data(BodyState::JOINT_POSITIONS);
    if(!initialJointPositions.empty()){
        qs = archive.createFlowStyleListing("initialJointPositions");
        qs->setFloatingNumberFormat("%g");
        for(size_t i=0; i < initialJointPositions.size(); ++i){
            qs->append(initialJointPositions[i], 10, n);
        }
    }

    archive.write("fix_root", body->isFixedRootModel());
    archive.write("collisionDetection", isCollisionDetectionEnabled);
    archive.write("selfCollisionDetection", isSelfCollisionDetectionEnabled);
    archive.write("lock_location", self->isLocationLocked());
    archive.write("scene_sensitive", self->isSceneSensitive());
    if(self->isVisibleLinkSelectionMode_){
        archive.write("visible_link_selection_mode", true);
    }

    if(kinematicsKitManager){
        MappingPtr kinematicsNode = new Mapping;
        if(kinematicsKitManager->storeState(*kinematicsNode) && !kinematicsNode->empty()){
            archive.insert("link_kinematics", kinematicsNode);
        }
    }

    if(isAttachable()){
        archive.write("enable_attachment", isAttachmentEnabled);
    }

    if(transparency > 0.0f){
        archive.write("transparency", transparency);
    }

    write(archive, "zmp", zmp);

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
            
    Vector3 p = Vector3::Zero();
    Matrix3 R = Matrix3::Identity();
        
    if(read(archive, "rootPosition", p)){
        body->rootLink()->p() = p;
    }
    if(read(archive, "rootAttitude", R)){
        body->rootLink()->R() = R;
    }

    //! \todo replace the following code with the ValueTree serialization function of BodyState
    initialState.clear();

    read(archive, "initialRootPosition", p);
    read(archive, "initialRootAttitude", R);
    initialState.setRootLinkPosition(SE3(p, R));

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
    archive.read("visible_link_selection_mode", self->isVisibleLinkSelectionMode_);
    archive.read("enable_attachment", isAttachmentEnabled);

    double t;
    if(archive.read("transparency", t)){
        setTransparency(t);
    }

    read(archive, "zmp", zmp);

    isUpdateNotificationOnSubTreeRestoredRequested = false;
    isNonRootLinkStateRestorationOnSubTreeRestoredRequested = false;

    archive.addProcessOnSubTreeRestored(
        [this, &archive](){
            isBeingRestored = false;

            bool doNotifyUpdate = false;
            bool doNotifyKinematicStateChange = false;

            // The attachment is updated after the sub tree is restored
            if(updateAttachment(true, false)){
                doNotifyUpdate = true;
            }
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
        BodyState::Data& q_initial = initialState.data(BodyState::JOINT_POSITIONS);
        q_initial.resize(nj);
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
                        format(_("Mismatched size of the stored joint positions for {}"), self->displayName()),
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
            BodyState::Data& q = initialState.data(BodyState::JOINT_POSITIONS);
            int n = body->numAllJoints();
            int m = qs->size();
            if(m != n){
                if(m != body->numJoints()){
                    MessageView::instance()->putln(
                        format(_("Mismatched size of the stored initial joint positions for {}"), self->displayName()),
                        MessageView::Warning);
                }
                m = std::min(m, n);
            }
            q.resize(n);
            for(int i=0; i < m; ++i){
                q[i] = (*qs)[i].toDouble();
            }
            for(int i=m; i < n; ++i){
                q[i] = body->joint(i)->q();
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

        auto util = impl->getOrCreateRenderableItemUtil();
        for(auto& link : impl->body->links()){
            util->getSceneFilesForArchiving(link->shape(), out_files);
            if(link->hasDedicatedCollisionShape()){
                util->getSceneFilesForArchiving(link->collisionShape(), out_files);
            }
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
        return format(_("Change the position of \"{0}\""), bodyItem->displayName());
    } else {
        return format(_("Restore the position of \"{0}\""), bodyItem->displayName());
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
