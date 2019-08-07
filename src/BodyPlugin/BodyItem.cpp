/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "BodyItem.h"
#include "WorldItem.h"
#include "KinematicsBar.h"
#include "EditableSceneBody.h"
#include "LinkSelectionView.h"
#include <cnoid/LeggedBodyHelper>
#include <cnoid/YAMLReader>
#include <cnoid/EigenArchive>
#include <cnoid/Archive>
#include <cnoid/RootItem>
#include <cnoid/ConnectionSet>
#include <cnoid/LazySignal>
#include <cnoid/LazyCaller>
#include <cnoid/MessageView>
#include <cnoid/TimeBar>
#include <cnoid/ItemManager>
#include <cnoid/OptionManager>
#include <cnoid/MenuManager>
#include <cnoid/PutPropertyFunction>
#include <cnoid/JointPath>
#include <cnoid/BodyLoader>
#include <cnoid/BodyState>
#include <cnoid/InverseKinematics>
#include <cnoid/CompositeIK>
#include <cnoid/CompositeBodyIK>
#include <cnoid/PinDragIK>
#include <cnoid/PenetrationBlocker>
#include <cnoid/FileUtil>
#include <cnoid/EigenArchive>
#include <fmt/format.h>
#include <bitset>
#include <deque>
#include <iostream>
#include <algorithm>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

const bool TRACE_FUNCTIONS = false;

BodyLoader bodyLoader;
BodyState kinematicStateCopy;

struct AttachmentInfo {
    string category;
    Link* link;
    Position T;
};

struct Attachment {
    BodyItem* baseBodyItem;
    Link* baseLink;
    Position T_base_local;
    Position T_local;
    bool isBaseKinematicStateChangedNotificationReqruied;
    string category;
    ScopedConnection connection;
};

class MyCompositeBodyIK : public CompositeBodyIK
{
public:
    MyCompositeBodyIK(BodyItemImpl* bodyItemImpl);
    virtual bool calcInverseKinematics(const Position& T) override;
    virtual std::shared_ptr<InverseKinematics> getParentBodyIK() override;

    BodyItemImpl* bodyItemImpl;
    unique_ptr<Attachment>& attachment;
    shared_ptr<InverseKinematics> baseIK;
};

}

namespace cnoid {

class BodyItemImpl
{
public:
    BodyItem* self;
    BodyPtr body;
    
    enum { UF_POSITIONS, UF_VELOCITIES, UF_ACCELERATIONS, UF_CM, UF_ZMP, NUM_UPUDATE_FLAGS };
    std::bitset<NUM_UPUDATE_FLAGS> updateFlags;

    LazySignal<Signal<void()>> sigKinematicStateChanged;
    LazySignal<Signal<void()>> sigKinematicStateEdited;

    LinkPtr currentBaseLink;
    LinkTraverse fkTraverse;
    shared_ptr<PinDragIK> pinDragIK;

    bool isEditable;
    bool isCallingSlotsOnKinematicStateEdited;
    bool isFkRequested;
    bool isVelFkRequested;
    bool isAccFkRequested;
    bool isCollisionDetectionEnabled;
    bool isSelfCollisionDetectionEnabled;

    BodyState initialState;
            
    typedef std::shared_ptr<BodyState> BodyStatePtr;
    std::deque<BodyStatePtr> kinematicStateHistory;
    size_t currentHistoryIndex;
    bool isCurrentKinematicStateInHistory;
    bool needToAppendKinematicStateToHistory;

    bool isOriginalModelStatic;

    KinematicsBar* kinematicsBar;
    EditableSceneBodyPtr sceneBody;

    Signal<void()> sigModelUpdated;

    unique_ptr<Attachment> attachment;

    LeggedBodyHelperPtr legged;
    Vector3 zmp;

    BodyItemImpl(BodyItem* self);
    BodyItemImpl(BodyItem* self, const BodyItemImpl& org);
    BodyItemImpl(BodyItem* self, Body* body);
    ~BodyItemImpl();
    void init(bool calledFromCopyConstructor);
    void initBody(bool calledFromCopyConstructor);
    bool loadModelFile(const std::string& filename);
    void setBody(Body* body);
    void setCurrentBaseLink(Link* link);
    void appendKinematicStateToHistory();
    bool undoKinematicState();
    bool redoKinematicState();
    void getCurrentIK(Link* targetLink, shared_ptr<InverseKinematics>& ik);
    void getDefaultIK(Link* targetLink, shared_ptr<InverseKinematics>& ik);
    void createPenetrationBlocker(Link* link, bool excludeSelfCollisions, shared_ptr<PenetrationBlocker>& blocker);
    void setPresetPose(BodyItem::PresetPoseID id);
    bool doLegIkToMoveCm(const Vector3& c, bool onlyProjectionToFloor);
    bool setStance(double width);
    void getParticularPosition(BodyItem::PositionType position, stdx::optional<Vector3>& pos);
    void notifyKinematicStateChange(bool requestFK, bool requestVelFK, bool requestAccFK, bool isDirect);
    void emitSigKinematicStateChanged();
    void emitSigKinematicStateEdited();
    bool enableCollisionDetection(bool on);
    bool enableSelfCollisionDetection(bool on);
    void updateCollisionDetectorLater();
    void doAssign(Item* srcItem);
    bool onStaticModelPropertyChanged(bool on);
    void createSceneBody();
    bool onEditableChanged(bool on);
    void tryToAttachToBodyItem(BodyItem* bodyItem);
    bool attachToBodyItem(BodyItem* bodyItem, const AttachmentInfo& baseInfo, const AttachmentInfo& info);
    void clearAttachment();
    void onBaseBodyKinematicStateChanged();
    void doPutProperties(PutPropertyFunction& putProperty);
    bool store(Archive& archive);
    bool restore(const Archive& archive);
};

}


static bool loadBodyItem(BodyItem* item, const std::string& filename)
{
    if(item->loadModelFile(filename)){
        if(item->name().empty()){
            item->setName(item->body()->modelName());
        }
        item->setEditable(!item->body()->isStaticModel());
        return true;
    }
    return false;
}
    

static void onSigOptionsParsed(boost::program_options::variables_map& variables)
{
    if(variables.count("hrpmodel")){
        vector<string> modelFileNames = variables["hrpmodel"].as< vector<string> >();
        for(size_t i=0; i < modelFileNames.size(); ++i){
            BodyItemPtr item(new BodyItem());
            if(item->load(modelFileNames[i], "OpenHRP-VRML-MODEL")){
                RootItem::mainInstance()->addChildItem(item);
            }
        }
    }
    else if(variables.count("body")){
    	vector<string> bodyFileNames = variables["body"].as<vector<string>>();
    	for(size_t i=0; i < bodyFileNames.size(); ++i){
    		BodyItemPtr item(new BodyItem());
    		if(item->load(bodyFileNames[i], "OpenHRP-VRML-MODEL")){
    			RootItem::mainInstance()->addChildItem(item);
    		}
    	}
    }
}


void BodyItem::initializeClass(ExtensionManager* ext)
{
    static bool initialized = false;

    if(!initialized){
        ItemManager& im = ext->itemManager();
        im.registerClass<BodyItem>(N_("BodyItem"));
        im.addLoader<BodyItem>(
            _("Body"), "OpenHRP-VRML-MODEL", "body;scen;wrl;yaml;yml;dae;stl",
            [](BodyItem* item, const std::string& filename, std::ostream&, Item*){
                return loadBodyItem(item, filename); });

        OptionManager& om = ext->optionManager();
        om.addOption("hrpmodel", boost::program_options::value< vector<string> >(), "load an OpenHRP model file");
        om.addOption("body", boost::program_options::value< vector<string> >(), "load a body file");
        om.sigOptionsParsed().connect(onSigOptionsParsed);

        initialized = true;
    }
}


BodyItem::BodyItem()
{
    impl = new BodyItemImpl(this);
    impl->init(false);
}
    

BodyItemImpl::BodyItemImpl(BodyItem* self)
    : BodyItemImpl(self, new Body)
{
    isEditable = true;
    isCollisionDetectionEnabled = true;
    isSelfCollisionDetectionEnabled = false;
}


BodyItem::BodyItem(const BodyItem& org)
    : Item(org)
{
    impl = new BodyItemImpl(this, *org.impl);
    impl->init(true);
}


BodyItemImpl::BodyItemImpl(BodyItem* self, const BodyItemImpl& org)
    : BodyItemImpl(self, org.body->clone())
{
    if(org.currentBaseLink){
        setCurrentBaseLink(body->link(org.currentBaseLink->index()));
    }
    zmp = org.zmp;
    isEditable = org.isEditable;
    isOriginalModelStatic = org.isOriginalModelStatic;
    isCollisionDetectionEnabled = org.isCollisionDetectionEnabled;
    isSelfCollisionDetectionEnabled = org.isSelfCollisionDetectionEnabled;

    initialState = org.initialState;
}


BodyItemImpl::BodyItemImpl(BodyItem* self, Body* body)
    : self(self),
      body(body),
      sigKinematicStateChanged([&](){ emitSigKinematicStateChanged(); }),
      sigKinematicStateEdited([&](){ emitSigKinematicStateEdited(); })
{

}


BodyItem::~BodyItem()
{
    delete impl;
}


BodyItemImpl::~BodyItemImpl()
{

}


void BodyItemImpl::init(bool calledFromCopyConstructor)
{
    self->setAttribute(Item::LOAD_ONLY);
    
    kinematicsBar = KinematicsBar::instance();
    isFkRequested = isVelFkRequested = isAccFkRequested = false;
    currentHistoryIndex = 0;
    isCurrentKinematicStateInHistory = false;
    needToAppendKinematicStateToHistory = false;
    isCallingSlotsOnKinematicStateEdited = false;

    initBody(calledFromCopyConstructor);
}


void BodyItemImpl::initBody(bool calledFromCopyConstructor)
{
    if(pinDragIK){
        pinDragIK.reset();
    }

    int n = body->numLinks();

    self->collisionsOfLink_.resize(n);
    self->collisionLinkBitSet_.resize(n);
    
    isOriginalModelStatic = body->isStaticModel();

    if(!calledFromCopyConstructor){
        setCurrentBaseLink(body->rootLink());
        zmp.setZero();
        self->storeInitialState();
    }
}


Body* BodyItem::body() const
{
    return impl->body.get();
}


bool BodyItem::isEditable() const
{
    return impl->isEditable;
}

    
void BodyItem::setEditable(bool on)
{
    if(on != impl->isEditable){
        impl->isEditable = on;
        notifyUpdate();
    }
}


SignalProxy<void()> BodyItem::sigKinematicStateChanged()
{
    return impl->sigKinematicStateChanged.signal();
}


SignalProxy<void()> BodyItem::sigKinematicStateEdited()
{
    return impl->sigKinematicStateEdited.signal();
}


bool BodyItem::loadModelFile(const std::string& filename)
{
    return impl->loadModelFile(filename);
}


bool BodyItemImpl::loadModelFile(const std::string& filename)
{
    bodyLoader.setMessageSink(MessageView::instance()->cout());

    BodyPtr newBody = new Body;
    newBody->setName(self->name());

    bool loaded = bodyLoader.load(newBody, filename);
    if(loaded){
        body = newBody;
        body->initializePosition();
        body->setCurrentTimeFunction([](){ return TimeBar::instance()->time(); });
    }

    initBody(false);

    return loaded;
}


void BodyItem::setBody(Body* body)
{
    impl->setBody(body);
}


void BodyItemImpl::setBody(Body* body_)
{
    body = body_;
    body->initializePosition();

    initBody(false);
}


void BodyItem::setName(const std::string& name)
{
    if(impl->body){
        impl->body->setName(name);
    }
    Item::setName(name);
}


SignalProxy<void()> BodyItem::sigModelUpdated()
{
    return impl->sigModelUpdated;
}


void BodyItem::notifyModelUpdate()
{
    impl->sigModelUpdated();
}


Link* BodyItem::currentBaseLink() const
{
    return impl->currentBaseLink;
}


void BodyItem::setCurrentBaseLink(Link* link)
{
    impl->setCurrentBaseLink(link);
}


void BodyItemImpl::setCurrentBaseLink(Link* link)
{
    if(link != currentBaseLink){
        if(link){
            fkTraverse.find(link, true, true);
        } else {
            fkTraverse.find(body->rootLink());
        }
    }
    currentBaseLink = link;
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
    BodyState currentState;
    storeKinematicState(currentState);

    state.getZMP(impl->zmp);
    state.restorePositions(*impl->body);

    //cout << "(currentState == state):" << (currentState == state) << endl;
    //return (currentState == state);
    return true;
}


void BodyItem::storeInitialState()
{
    storeKinematicState(impl->initialState);
}


void BodyItem::restoreInitialState(bool doNotify)
{
    bool restored = restoreKinematicState(impl->initialState);
    if(restored && doNotify){
        notifyKinematicStateChange(false);
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

    if(!impl->isCurrentKinematicStateInHistory){
        impl->appendKinematicStateToHistory();
    }
}


void BodyItemImpl::appendKinematicStateToHistory()
{
    if(TRACE_FUNCTIONS){
        cout << "BodyItem::appendKinematicStateToHistory()" << endl;
    }

    BodyStatePtr state = std::make_shared<BodyState>();
    self->storeKinematicState(*state);

    if(kinematicStateHistory.empty() || (currentHistoryIndex == kinematicStateHistory.size() - 1)){
        kinematicStateHistory.push_back(state);
        currentHistoryIndex = kinematicStateHistory.size() - 1;
    } else {
        ++currentHistoryIndex;
        kinematicStateHistory.resize(currentHistoryIndex + 1);
        kinematicStateHistory[currentHistoryIndex] = state;
    }
        
    if(kinematicStateHistory.size() > 20){
        kinematicStateHistory.pop_front();
        currentHistoryIndex--;
    }

    isCurrentKinematicStateInHistory = true;
}


void BodyItem::cancelKinematicStateEdit()
{
    if(TRACE_FUNCTIONS){
        cout << "BodyItem::cancelKinematicStateEdit()" << endl;
    }

    if(impl->isCurrentKinematicStateInHistory){
        restoreKinematicState(*impl->kinematicStateHistory[impl->currentHistoryIndex]);
        impl->kinematicStateHistory.pop_back();
        if(impl->currentHistoryIndex > 0){
            --impl->currentHistoryIndex;
        }
        impl->isCurrentKinematicStateInHistory = false;
    }
}
        

void BodyItem::acceptKinematicStateEdit()
{
    if(TRACE_FUNCTIONS){
        cout << "BodyItem::acceptKinematicStateEdit()" << endl;
    }

    //appendKinematicStateToHistory();
    impl->needToAppendKinematicStateToHistory = true;
    impl->sigKinematicStateEdited.request();
}


bool BodyItem::undoKinematicState()
{
    if(TRACE_FUNCTIONS){
        cout << "BodyItem::undoKinematicState()" << endl;
    }

    return impl->undoKinematicState();
}


bool BodyItemImpl::undoKinematicState()
{
    bool done = false;
    bool modified = false;

    if(!isCurrentKinematicStateInHistory){
        if(currentHistoryIndex < kinematicStateHistory.size()){
            done = true;
            modified = self->restoreKinematicState(*kinematicStateHistory[currentHistoryIndex]);
        }
    } else {
        if(currentHistoryIndex > 0){
            done = true;
            modified = self->restoreKinematicState(*kinematicStateHistory[--currentHistoryIndex]);
        }
    }

    if(done){
        if(modified){
            self->notifyKinematicStateChange(false);
            isCurrentKinematicStateInHistory = true;
            sigKinematicStateEdited.request();
        } else {
            isCurrentKinematicStateInHistory = true;
            done = undoKinematicState();
        }
    }

    return done;
}


bool BodyItem::redoKinematicState()
{
    if(TRACE_FUNCTIONS){
        cout << "BodyItem::redoKinematicState()" << endl;
    }

    return impl->redoKinematicState();
}


bool BodyItemImpl::redoKinematicState()
{
    if(currentHistoryIndex + 1 < kinematicStateHistory.size()){
        self->restoreKinematicState(*kinematicStateHistory[++currentHistoryIndex]);
        self->notifyKinematicStateChange(false);
        isCurrentKinematicStateInHistory = true;
        sigKinematicStateEdited.request();
        return true;
    }
    return false;
}
        

std::shared_ptr<PinDragIK> BodyItem::pinDragIK()
{
    if(!impl->pinDragIK){
        impl->pinDragIK = std::make_shared<PinDragIK>(impl->body);
    }
    return impl->pinDragIK;
}


std::shared_ptr<InverseKinematics> BodyItem::getCurrentIK(Link* targetLink)
{
    shared_ptr<InverseKinematics> ik;
    impl->getCurrentIK(targetLink, ik);
    return ik;
}


void BodyItemImpl::getCurrentIK(Link* targetLink, shared_ptr<InverseKinematics>& ik)
{
    auto rootLink = body->rootLink();
    
    if(attachment && targetLink == rootLink){
        ik = make_shared<MyCompositeBodyIK>(this);
    }

    if(!ik){
        if(KinematicsBar::instance()->mode() == KinematicsBar::AUTO_MODE){
            getDefaultIK(targetLink, ik);
        }
    }

    if(!ik){
        self->pinDragIK(); // create if not created
        if(pinDragIK->numPinnedLinks() > 0){
            pinDragIK->setTargetLink(targetLink, true);
            if(pinDragIK->initialize()){
                ik = pinDragIK;
            }
        }
    }
    if(!ik){
        auto baseLink = currentBaseLink ? currentBaseLink.get() : rootLink;
        ik = getCustomJointPath(body, baseLink, targetLink);
    }
}


std::shared_ptr<InverseKinematics> BodyItem::getDefaultIK(Link* targetLink)
{
    shared_ptr<InverseKinematics> ik;
    impl->getDefaultIK(targetLink, ik);
    return ik;
}


void BodyItemImpl::getDefaultIK(Link* targetLink, shared_ptr<InverseKinematics>& ik)
{
    const Mapping& setupMap = *body->info()->findMapping("defaultIKsetup");

    if(targetLink && setupMap.isValid()){
        const Listing& setup = *setupMap.findListing(targetLink->name());
        if(setup.isValid() && !setup.empty()){
            Link* baseLink = body->link(setup[0].toString());
            if(baseLink){
                if(setup.size() == 1){
                    ik = getCustomJointPath(body, baseLink, targetLink);
                } else {
                    auto compositeIK = make_shared<CompositeIK>(body, targetLink);
                    ik = compositeIK;
                    for(int i=0; i < setup.size(); ++i){
                        Link* baseLink = body->link(setup[i].toString());
                        if(baseLink){
                            if(!compositeIK->addBaseLink(baseLink)){
                                ik.reset();
                                break;
                            }
                        }
                    }
                }
            }
        }
    }
}


std::shared_ptr<PenetrationBlocker> BodyItem::createPenetrationBlocker(Link* link, bool excludeSelfCollisions)
{
    shared_ptr<PenetrationBlocker> blocker;
    impl->createPenetrationBlocker(link, excludeSelfCollisions, blocker);
    return blocker;
}


void BodyItemImpl::createPenetrationBlocker(Link* link, bool excludeSelfCollisions, shared_ptr<PenetrationBlocker>& blocker)
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
        blocker->setDepth(kinematicsBar->penetrationBlockDepth());
        blocker->start();
    }
}


void BodyItem::moveToOrigin()
{
    beginKinematicStateEdit();
    
    impl->body->rootLink()->T() = impl->body->defaultPosition();
    impl->body->calcForwardKinematics();
    
    notifyKinematicStateChange(false);
    acceptKinematicStateEdit();
}


void BodyItem::setPresetPose(PresetPoseID id)
{
    impl->setPresetPose(id);
}


void BodyItemImpl::setPresetPose(BodyItem::PresetPoseID id)
{
    int jointIndex = 0;

    self->beginKinematicStateEdit();
    
    if(id == BodyItem::STANDARD_POSE){
        const Listing& pose = *body->info()->findListing("standardPose");
        if(pose.isValid()){
            const int n = std::min(pose.size(), body->numJoints());
            while(jointIndex < n){
                body->joint(jointIndex)->q() = pose[jointIndex].toAngle();
                jointIndex++;
            }
        }
    }

    const int n = body->numAllJoints();
    while(jointIndex < n){
        Link* joint = body->joint(jointIndex++);
        joint->q() = joint->q_initial();
    }

    fkTraverse.calcForwardKinematics();
    self->notifyKinematicStateChange(false);
    self->acceptKinematicStateEdit();
}


const Vector3& BodyItem::centerOfMass()
{
    if(!impl->updateFlags.test(BodyItemImpl::UF_CM)){
        impl->body->calcCenterOfMass();
        impl->updateFlags.set(BodyItemImpl::UF_CM);
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


bool BodyItemImpl::doLegIkToMoveCm(const Vector3& c, bool onlyProjectionToFloor)
{
    bool result = false;

    if(self->isLeggedBody()){
        
        BodyState orgKinematicState;
        self->storeKinematicState(orgKinematicState);
        self->beginKinematicStateEdit();
        
        result = legged->doLegIkToMoveCm(c, onlyProjectionToFloor);

        if(result){
            self->notifyKinematicStateChange();
            self->acceptKinematicStateEdit();
            updateFlags.set(UF_CM);
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


bool BodyItemImpl::setStance(double width)
{
    bool result = false;
    
    if(self->isLeggedBody()){
        
        BodyState orgKinematicState;
        self->storeKinematicState(orgKinematicState);
        self->beginKinematicStateEdit();
        
        result = legged->setStance(width, currentBaseLink);

        if(result){
            self->notifyKinematicStateChange();
            self->acceptKinematicStateEdit();
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


void BodyItemImpl::getParticularPosition(BodyItem::PositionType position, stdx::optional<Vector3>& pos)
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
                    pos = legged->homeCopOfSole((position == BodyItem::RIGHT_HOME_COP) ? 0 : 1);
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
    beginKinematicStateEdit();
    setZmp(zmp);
    notifyKinematicStateChange(false);
    acceptKinematicStateEdit();
}


void BodyItemImpl::notifyKinematicStateChange(bool requestFK, bool requestVelFK, bool requestAccFK, bool isDirect)
{
    if(!isCallingSlotsOnKinematicStateEdited){
        isCurrentKinematicStateInHistory = false;
    }

    updateFlags.reset();

    if(attachment && attachment->isBaseKinematicStateChangedNotificationReqruied){
        attachment->isBaseKinematicStateChangedNotificationReqruied = false;
        attachment->baseBodyItem->impl->notifyKinematicStateChange(
            requestFK, requestVelFK, requestAccFK, isDirect);

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


void BodyItemImpl::emitSigKinematicStateChanged()
{
    if(isFkRequested){
        fkTraverse.calcForwardKinematics(isVelFkRequested, isAccFkRequested);
        isFkRequested = isVelFkRequested = isAccFkRequested = false;
    }

    sigKinematicStateChanged.signal()();

    if(needToAppendKinematicStateToHistory){
        appendKinematicStateToHistory();
        needToAppendKinematicStateToHistory = false;
    }
}


void BodyItem::notifyKinematicStateChange(bool requestFK, bool requestVelFK, bool requestAccFK)
{
    impl->notifyKinematicStateChange(requestFK, requestVelFK,requestAccFK, true);
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


void BodyItemImpl::emitSigKinematicStateEdited()
{
    isCallingSlotsOnKinematicStateEdited = true;
    sigKinematicStateEdited.signal()();
    isCallingSlotsOnKinematicStateEdited = false;
    
    if(!sigKinematicStateEdited.isPending() && needToAppendKinematicStateToHistory){
        appendKinematicStateToHistory();
        needToAppendKinematicStateToHistory = false;
    }
}


void BodyItem::enableCollisionDetection(bool on)
{
    impl->enableCollisionDetection(on);
}


bool BodyItemImpl::enableCollisionDetection(bool on)
{
    if(on != isCollisionDetectionEnabled){
        isCollisionDetectionEnabled = on;
        updateCollisionDetectorLater();
        return true;
    }
    return false;
}


void BodyItem::enableSelfCollisionDetection(bool on)
{
    impl->enableSelfCollisionDetection(on);
}


bool BodyItemImpl::enableSelfCollisionDetection(bool on)
{
    if(on != isSelfCollisionDetectionEnabled){
        isSelfCollisionDetectionEnabled = on;
        updateCollisionDetectorLater();
        return true;
    }
    return false;
}


void BodyItemImpl::updateCollisionDetectorLater()
{
    if(TRACE_FUNCTIONS){
        cout << "BodyItemImpl::updateCollisionDetectorLater(): " << self->name() << endl;
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


Item* BodyItem::doDuplicate() const
{
    return new BodyItem(*this);
}


void BodyItem::doAssign(Item* srcItem)
{
    Item::doAssign(srcItem);
    impl->doAssign(srcItem);
}


void BodyItemImpl::doAssign(Item* srcItem)
{
    BodyItem* srcBodyItem = dynamic_cast<BodyItem*>(srcItem);
    if(srcBodyItem){
        // copy the base link property
        Link* baseLink = nullptr;
        Link* srcBaseLink = srcBodyItem->currentBaseLink();
        if(srcBaseLink){
            baseLink = body->link(srcBaseLink->name());
            if(baseLink){
                setCurrentBaseLink(baseLink);
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
        zmp = srcBodyItem->impl->zmp;

        initialState = srcBodyItem->impl->initialState;
        
        self->notifyKinematicStateChange(true);
    }
}


void BodyItem::onPositionChanged()
{
    auto worldItem = findOwnerItem<WorldItem>();
    if(!worldItem){
        clearCollisions();
    }

    auto ownerBodyItem = findOwnerItem<BodyItem>();
    if(!impl->attachment || (impl->attachment->baseBodyItem != ownerBodyItem)){
        impl->clearAttachment();
        if(ownerBodyItem){
            impl->tryToAttachToBodyItem(ownerBodyItem);
        }
    }
}


bool BodyItemImpl::onStaticModelPropertyChanged(bool on)
{
    if(on){
        if(!body->isStaticModel() && body->numLinks() == 1){
            body->rootLink()->setJointType(Link::FIXED_JOINT);
            body->updateLinkTree();
            return body->isStaticModel();
        }
    } else if(body->isStaticModel()){
        body->rootLink()->setJointType(Link::FREE_JOINT);
        body->updateLinkTree();
        return !body->isStaticModel();
    }
    return false;
}
        

EditableSceneBody* BodyItem::sceneBody()
{
    if(!impl->sceneBody){
        impl->createSceneBody();
    }
    return impl->sceneBody;
}


void BodyItemImpl::createSceneBody()
{
    sceneBody = new EditableSceneBody(self);
    sceneBody->setSceneDeviceUpdateConnection(true);
}


SgNode* BodyItem::getScene()
{
    return sceneBody();
}


EditableSceneBody* BodyItem::existingSceneBody()
{
    return impl->sceneBody;
}


bool BodyItemImpl::onEditableChanged(bool on)
{
    self->setEditable(on);
    return true;
}


static stdx::optional<AttachmentInfo> readAttachmentInfo(Mapping& node, const Body* body, bool isBase)
{
    AttachmentInfo info;
    if(node.read("category", info.category)){
        if(isBase){
            string linkName;
            if(node.read("link", linkName)){
                info.link = body->link(linkName);
                if(!info.link){
                    return stdx::nullopt;
                }
            }
        } else {
            info.link = nullptr;
        }
        info.T.setIdentity();
        Vector3 translation;
        if(read(node, "translation", translation)){
            info.T.translation() = translation;
        }
        AngleAxis rotation;
        if(read(node, "rotation", rotation)){
            info.T.linear() = rotation.toRotationMatrix();
        }
        return info;
    }
    return stdx::nullopt;
}


static vector<AttachmentInfo> readAttachmentBases(const Body* body)
{
    vector<AttachmentInfo> infos;
    auto nodes = body->info()->findListing("attachmentBases");
    if(nodes->isValid()){
        infos.reserve(nodes->size());
        for(auto& node : *nodes){
            if(node->isMapping()){
                auto info = readAttachmentInfo(*node->toMapping(), body, true);
                if(info){
                    infos.push_back(*info);
                }
            }
        }
    }
    return infos;
}


void BodyItemImpl::tryToAttachToBodyItem(BodyItem* bodyItem)
{
    bool attached = false;
    
    auto node = body->info()->findMapping("attachment");
    if(node->isValid()){
        auto attachment = readAttachmentInfo(*node, body, false);
        if(attachment){
            auto attachmentBases = readAttachmentBases(bodyItem->body());
            if(!attachmentBases.empty()){
                for(auto& base : attachmentBases){
                    if(base.category == (*attachment).category){
                        if(attachToBodyItem(bodyItem, base, *attachment)){
                            attached = true;
                            break;
                        }
                    }
                }
            }
        }
    }

    if(!attached){
        mvout() << format(_("{0} cannot be attached to {1}."),
                          self->name(), bodyItem->name()) << endl;
    }
}


bool BodyItemImpl::attachToBodyItem
(BodyItem* bodyItem, const AttachmentInfo& baseInfo, const AttachmentInfo& info)
{
    attachment.reset(new Attachment);
    
    attachment->baseBodyItem = bodyItem;
    attachment->baseLink = baseInfo.link;
    attachment->T_base_local = baseInfo.T;
    attachment->T_local = info.T;
    attachment->isBaseKinematicStateChangedNotificationReqruied = false;
    attachment->category = info.category;

    mvout() << format(_("{0} has been attached to {1} of {2}."),
                      self->name(), baseInfo.link->name(), bodyItem->name()) << endl;

    attachment->connection =
        bodyItem->sigKinematicStateChanged().connect(
            [&](){ onBaseBodyKinematicStateChanged(); });

    onBaseBodyKinematicStateChanged();

    return true;
}


void BodyItemImpl::clearAttachment()
{
    if(attachment){
        auto baseLink = attachment->baseLink;
        if(baseLink){
            mvout() << format(_("{0} has been detached from {1} of {2}."),
                              self->name(), baseLink->name(), baseLink->body()->name()) << endl;
        }
        attachment.reset();
    }
}


void BodyItemImpl::onBaseBodyKinematicStateChanged()
{
    Position T_base = attachment->baseLink->T() * attachment->T_base_local;
    body->rootLink()->T() = T_base * attachment->T_local.inverse(Eigen::Isometry);
    attachment->isBaseKinematicStateChangedNotificationReqruied = false;

    //! \todo requestVelFK and requestAccFK should be set appropriately
    notifyKinematicStateChange(true, false, false, true);
}


MyCompositeBodyIK::MyCompositeBodyIK(BodyItemImpl* bodyItemImpl)
    : bodyItemImpl(bodyItemImpl),
      attachment(bodyItemImpl->attachment)
{
    baseIK = attachment->baseBodyItem->getCurrentIK(attachment->baseLink);
}


bool MyCompositeBodyIK::calcInverseKinematics(const Position& T)
{
    bool result = false;
    if(baseIK){
        if(!attachment){
            baseIK.reset();
        } else {
            Position Ta = T * attachment->T_local * attachment->T_base_local.inverse(Eigen::Isometry);
            result = baseIK->calcInverseKinematics(Ta);
            if(result){
                attachment->isBaseKinematicStateChangedNotificationReqruied = true;
            }
        }
    }
    return result;
}


std::shared_ptr<InverseKinematics> MyCompositeBodyIK::getParentBodyIK()
{
    return baseIK;
}


void BodyItem::doPutProperties(PutPropertyFunction& putProperty)
{
    impl->doPutProperties(putProperty);
}


void BodyItemImpl::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Model name"), body->modelName());
    putProperty(_("Num links"), body->numLinks());
    putProperty(_("Num joints"), body->numJoints());
    putProperty(_("Num devices"), (int)body->devices().size());
    putProperty(_("Root link"), body->rootLink()->name());
    putProperty(_("Base link"), currentBaseLink ? currentBaseLink->name() : "none");
    putProperty.decimals(3)(_("Mass"), body->mass());
    putProperty(_("Static model"), body->isStaticModel(),
                [&](bool on){ return onStaticModelPropertyChanged(on); });
    putProperty(_("Collision detection"), isCollisionDetectionEnabled,
                [&](bool on){ return enableCollisionDetection(on); });
    putProperty(_("Self-collision detection"), isSelfCollisionDetectionEnabled,
                [&](bool on){ return enableSelfCollisionDetection(on); });
    putProperty(_("Editable"), isEditable,
                [&](bool on){ return onEditableChanged(on); });
}


bool BodyItem::store(Archive& archive)
{
    return impl->store(archive);
}


bool BodyItemImpl::store(Archive& archive)
{
    archive.setDoubleFormat("% .6f");

    archive.writeRelocatablePath("modelFile", self->filePath());
    archive.write("currentBaseLink", (currentBaseLink ? currentBaseLink->name() : ""), DOUBLE_QUOTED);

    /// \todo Improve the following for current / initial position representations
    write(archive, "rootPosition", body->rootLink()->p());
    write(archive, "rootAttitude", Matrix3(body->rootLink()->R()));
    Listing* qs = archive.createFlowStyleListing("jointPositions");
    int n = body->numAllJoints();
    for(int i=0; i < n; ++i){
        qs->append(body->joint(i)->q(), 10, n);
    }

    //! \todo replace the following code with the ValueTree serialization function of BodyState
    SE3 initialRootPosition;
    if(initialState.getRootLinkPosition(initialRootPosition)){
        write(archive, "initialRootPosition", initialRootPosition.translation());
        write(archive, "initialRootAttitude", Matrix3(initialRootPosition.rotation()));
    }
    BodyState::Data& initialJointPositions = initialState.data(BodyState::JOINT_POSITIONS);
    if(!initialJointPositions.empty()){
        qs = archive.createFlowStyleListing("initialJointPositions");
        for(size_t i=0; i < initialJointPositions.size(); ++i){
            qs->append(initialJointPositions[i], 10, n);
        }
    }

    write(archive, "zmp", zmp);

    if(isOriginalModelStatic != body->isStaticModel()){
        archive.write("staticModel", body->isStaticModel());
    }

    archive.write("collisionDetection", isCollisionDetectionEnabled);
    archive.write("selfCollisionDetection", isSelfCollisionDetectionEnabled);
    archive.write("isEditable", isEditable);

    return true;
}


bool BodyItem::restore(const Archive& archive)
{
    return impl->restore(archive);
}


bool BodyItemImpl::restore(const Archive& archive)
{
    if(!archive.loadItemFile(self, "modelFile")){
        return false;
    }

    Vector3 p = Vector3::Zero();
    Matrix3 R = Matrix3::Identity();
        
    if(read(archive, "rootPosition", p)){
        body->rootLink()->p() = p;
    }
    if(read(archive, "rootAttitude", R)){
        body->rootLink()->R() = R;
    }
    Listing* qs = archive.findListing("jointPositions");
    if(qs->isValid()){
        int nj = body->numAllJoints();
        if(qs->size() != nj){
            if(qs->size() != body->numJoints()){
                MessageView::instance()->putln(
                    format(_("Mismatched size of the stored joint positions for {}"), self->name()),
                    MessageView::WARNING);
            }
            nj = std::min(qs->size(), nj);
        }
        for(int i=0; i < nj; ++i){
            body->joint(i)->q() = (*qs)[i].toDouble();
        }
    }

    //! \todo replace the following code with the ValueTree serialization function of BodyState
    initialState.clear();

    read(archive, "initialRootPosition", p);
    read(archive, "initialRootAttitude", R);
    initialState.setRootLinkPosition(SE3(p, R));
    
    qs = archive.findListing("initialJointPositions");
    if(qs->isValid()){
        BodyState::Data& q = initialState.data(BodyState::JOINT_POSITIONS);
        int n = body->numAllJoints();
        int m = qs->size();
        if(m != n){
            if(m != body->numJoints()){
                MessageView::instance()->putln(
                    format(_("Mismatched size of the stored initial joint positions for {}"), self->name()),
                    MessageView::WARNING);
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

    read(archive, "zmp", zmp);
        
    body->calcForwardKinematics();
    setCurrentBaseLink(body->link(archive.get("currentBaseLink", "")));

    bool staticModel;
    if(archive.read("staticModel", staticModel)){
        onStaticModelPropertyChanged(staticModel);
    }

    bool on;
    if(archive.read("collisionDetection", on)){
        enableCollisionDetection(on);
    }
    if(archive.read("selfCollisionDetection", on)){
        enableSelfCollisionDetection(on);
    }

    archive.read("isEditable", isEditable);

    self->notifyKinematicStateChange();

    return true;
}
