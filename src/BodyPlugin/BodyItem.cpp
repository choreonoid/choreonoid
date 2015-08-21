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
#include <cnoid/ItemManager>
#include <cnoid/OptionManager>
#include <cnoid/MenuManager>
#include <cnoid/PutPropertyFunction>
#include <cnoid/JointPath>
#include <cnoid/BodyLoader>
#include <cnoid/BodyState>
#include <cnoid/InverseKinematics>
#include <cnoid/CompositeIK>
#include <cnoid/PinDragIK>
#include <cnoid/PenetrationBlocker>
#include <cnoid/FileUtil>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <bitset>
#include <deque>
#include <iostream>
#include <algorithm>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

const bool TRACE_FUNCTIONS = false;

BodyLoader bodyLoader;

Action* linkVisibilityCheck;
Action* showVisualShapeCheck;
Action* showCollisionShapeCheck;
BodyState kinematicStateCopy;

/// \todo move this to hrpUtil ?
inline double radian(double deg) { return (3.14159265358979 * deg / 180.0); }

bool loadBodyItem(BodyItem* item, const std::string& filename)
{
    if(item->loadModelFile(filename)){
        if(item->name().empty()){
            item->setName(item->body()->modelName());
        }
        return true;
    }
    return false;
}
    
void onSigOptionsParsed(boost::program_options::variables_map& variables)
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
}

}


namespace cnoid {

class BodyItemImpl
{
public:
    BodyItem* self;
    BodyPtr body;
    LeggedBodyHelperPtr legged;
    Vector3 zmp;
    
    enum { UF_POSITIONS, UF_VELOCITIES, UF_ACCELERATIONS, UF_CM, UF_ZMP, NUM_UPUDATE_FLAGS };
    std::bitset<NUM_UPUDATE_FLAGS> updateFlags;

    LazySignal< Signal<void()> > sigKinematicStateChanged;
    LazySignal< Signal<void()> > sigKinematicStateEdited;

    bool isEditable;
    bool isCallingSlotsOnKinematicStateEdited;
    bool isFkRequested;
    bool isVelFkRequested;
    bool isAccFkRequested;
    Link* currentBaseLink;
    LinkTraverse fkTraverse;
    PinDragIKptr pinDragIK;

    bool isCollisionDetectionEnabled;
    bool isSelfCollisionDetectionEnabled;

    BodyState initialState;
            
    typedef boost::shared_ptr<BodyState> BodyStatePtr;
    std::deque<BodyStatePtr> kinematicStateHistory;
    size_t currentHistoryIndex;
    bool isCurrentKinematicStateInHistory;
    bool needToAppendKinematicStateToHistory;

    bool isOriginalModelStatic;

    KinematicsBar* kinematicsBar;
    EditableSceneBodyPtr sceneBody;

    ScopedConnection connectionToSigLinkSelectionChanged;
    ScopedConnectionSet optionCheckConnections;

    Signal<void()> sigModelUpdated;

    BodyItemImpl(BodyItem* self);
    BodyItemImpl(BodyItem* self, const BodyItemImpl& org);
    ~BodyItemImpl();
        
    void init(bool calledFromCopyConstructor);
    void initBody(bool calledFromCopyConstructor);
    bool loadModelFile(const std::string& filename);
    void setCurrentBaseLink(Link* link);
    void emitSigKinematicStateChanged();
    void emitSigKinematicStateEdited();
    bool enableCollisionDetection(bool on);
    bool enableSelfCollisionDetection(bool on);
    void updateCollisionDetectorLater();
    void appendKinematicStateToHistory();
    bool onStaticModelPropertyChanged(bool on);
    void createSceneBody();
    void onLinkVisibilityCheckToggled();
    void onVisibleShapeTypesChanged();
    void onLinkSelectionChanged();
    void onPositionChanged();
    bool undoKinematicState();
    bool redoKinematicState();
    void getCurrentIK(Link* targetLink, InverseKinematicsPtr& ik);
    void getDefaultIK(Link* targetLink, InverseKinematicsPtr& ik);
    void createPenetrationBlocker(Link* link, bool excludeSelfCollisions, PenetrationBlockerPtr& blocker);
    void setPresetPose(BodyItem::PresetPoseID id);
    bool doLegIkToMoveCm(const Vector3& c, bool onlyProjectionToFloor);
    bool setStance(double width);
    void getParticularPosition(BodyItem::PositionType position, boost::optional<Vector3>& pos);
    void doAssign(Item* srcItem);
    bool onEditableChanged(bool on);
    void doPutProperties(PutPropertyFunction& putProperty);
    bool store(Archive& archive);
    bool restore(const Archive& archive);
};

}
    

void BodyItem::initializeClass(ExtensionManager* ext)
{
    static bool initialized = false;

    if(!initialized){
        ItemManager& im = ext->itemManager();
        im.registerClass<BodyItem>(N_("BodyItem"));
        im.addLoader<BodyItem>(
            _("OpenHRP Model File"), "OpenHRP-VRML-MODEL", "wrl;yaml;dae;stl", boost::bind(loadBodyItem, _1, _2));

        OptionManager& om = ext->optionManager();
        om.addOption("hrpmodel", boost::program_options::value< vector<string> >(), "load an OpenHRP model file");
        om.sigOptionsParsed().connect(onSigOptionsParsed);

        MenuManager& mm = ext->menuManager().setPath("/Options/Scene View");
        linkVisibilityCheck = mm.addCheckItem(_("Show selected links only"));
        showVisualShapeCheck = mm.addCheckItem(_("Show visual shapes"));
        showVisualShapeCheck->setChecked(true);
        showCollisionShapeCheck = mm.addCheckItem(_("Show collision shapes"));

        initialized = true;
    }
}


BodyItem::BodyItem()
{
    impl = new BodyItemImpl(this);
    impl->init(false);
}
    

BodyItemImpl::BodyItemImpl(BodyItem* self)
    : self(self),
      sigKinematicStateChanged(boost::bind(&BodyItemImpl::emitSigKinematicStateChanged, this)),
      sigKinematicStateEdited(boost::bind(&BodyItemImpl::emitSigKinematicStateEdited, this))
{
    body = new Body();
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
    : self(self),
      body(org.body->clone()),
      sigKinematicStateChanged(boost::bind(&BodyItemImpl::emitSigKinematicStateChanged, this)),
      sigKinematicStateEdited(boost::bind(&BodyItemImpl::emitSigKinematicStateEdited, this)),
      initialState(org.initialState)
{
    if(org.currentBaseLink){
        setCurrentBaseLink(body->link(org.currentBaseLink->index()));
    }
    zmp = org.zmp;
    isEditable = org.isEditable;
    isOriginalModelStatic = org.isOriginalModelStatic;
    isCollisionDetectionEnabled = org.isCollisionDetectionEnabled;
    isSelfCollisionDetectionEnabled = org.isSelfCollisionDetectionEnabled;
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

    self->sigPositionChanged().connect(boost::bind(&BodyItemImpl::onPositionChanged, this));
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


BodyItem::~BodyItem()
{
    delete impl;
}


BodyItemImpl::~BodyItemImpl()
{
    connectionToSigLinkSelectionChanged.disconnect();
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


void BodyItemImpl::onPositionChanged()
{
    WorldItem* worldItem = self->findOwnerItem<WorldItem>();
    if(!worldItem){
        self->clearCollisions();
    }
}


bool BodyItem::loadModelFile(const std::string& filename)
{
    return impl->loadModelFile(filename);
}


bool BodyItemImpl::loadModelFile(const std::string& filename)
{
    MessageView* mv = MessageView::instance();
    mv->beginStdioRedirect();
    bodyLoader.setMessageSink(mv->cout(true));

    BodyPtr newBody = bodyLoader.load(filename);

    mv->endStdioRedirect();
    
    if(newBody){
        body = newBody;
        body->setName(self->name());
        body->initializeState();
    }

    initBody(false);

    return (newBody);
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
    bool modified = false;

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


void BodyItem::restoreInitialState()
{
    if(restoreKinematicState(impl->initialState)){
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


void BodyItem::acceptKinematicStateEdit()
{
    if(TRACE_FUNCTIONS){
        cout << "BodyItem::acceptKinematicStateEdit()" << endl;
    }

    //appendKinematicStateToHistory();
    impl->needToAppendKinematicStateToHistory = true;
    impl->sigKinematicStateEdited.request();
}


void BodyItemImpl::appendKinematicStateToHistory()
{
    if(TRACE_FUNCTIONS){
        cout << "BodyItem::appendKinematicStateToHistory()" << endl;
    }

    BodyStatePtr state = boost::make_shared<BodyState>();
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
        

PinDragIKptr BodyItem::pinDragIK()
{
    if(!impl->pinDragIK){
        impl->pinDragIK = boost::make_shared<PinDragIK>(impl->body);
    }
    return impl->pinDragIK;
}


InverseKinematicsPtr BodyItem::getCurrentIK(Link* targetLink)
{
    InverseKinematicsPtr ik;
    impl->getCurrentIK(targetLink, ik);
    return ik;
}


void BodyItemImpl::getCurrentIK(Link* targetLink, InverseKinematicsPtr& ik)
{
    if(KinematicsBar::instance()->mode() == KinematicsBar::AUTO_MODE){
        getDefaultIK(targetLink, ik);
    }

    if(!ik){
        self->pinDragIK(); // create if not created
        if(pinDragIK->numPinnedLinks() > 0 || !currentBaseLink){
            pinDragIK->setTargetLink(targetLink, true);
            if(pinDragIK->initialize()){
                ik = pinDragIK;
            }
        }
    }
    if(!ik){
        if(currentBaseLink){
            ik = getCustomJointPath(body, currentBaseLink, targetLink);
        }
    }
}


InverseKinematicsPtr BodyItem::getDefaultIK(Link* targetLink)
{
    InverseKinematicsPtr ik;
    impl->getDefaultIK(targetLink, ik);
    return ik;
}


void BodyItemImpl::getDefaultIK(Link* targetLink, InverseKinematicsPtr& ik)
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
                    CompositeIKPtr compositeIK(new CompositeIK(body, targetLink));
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


PenetrationBlockerPtr BodyItem::createPenetrationBlocker(Link* link, bool excludeSelfCollisions)
{
    PenetrationBlockerPtr blocker;
    impl->createPenetrationBlocker(link, excludeSelfCollisions, blocker);
    return blocker;
}


void BodyItemImpl::createPenetrationBlocker(Link* link, bool excludeSelfCollisions, PenetrationBlockerPtr& blocker)
{
    WorldItem* worldItem = self->findOwnerItem<WorldItem>();
    if(worldItem){
        blocker = boost::make_shared<PenetrationBlocker>(worldItem->collisionDetector()->clone(), link);
        const ItemList<BodyItem>& bodyItems = worldItem->collisionBodyItems();
        for(int i=0; i < bodyItems.size(); ++i){
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
                body->joint(jointIndex)->q() = radian(pose[jointIndex].toDouble());
                jointIndex++;
            }
        }
    }

    const int n = body->numJoints();
    while(jointIndex < n){
        body->joint(jointIndex++)->q() = 0.0;
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
    return (impl->legged->numFeet() > 0);
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

    LeggedBodyHelperPtr legged = getLeggedBodyHelper(body);

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
                

boost::optional<Vector3> BodyItem::getParticularPosition(PositionType position)
{
    boost::optional<Vector3> pos;
    impl->getParticularPosition(position, pos);
    return pos;
}


void BodyItemImpl::getParticularPosition(BodyItem::PositionType position, boost::optional<Vector3>& pos)
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


void BodyItem::notifyKinematicStateChange(bool requestFK, bool requestVelFK, bool requestAccFK)
{
    if(!impl->isCallingSlotsOnKinematicStateEdited){
        impl->isCurrentKinematicStateInHistory = false;
    }

    if(requestFK){
        impl->isFkRequested |= requestFK;
        impl->isVelFkRequested |= requestVelFK;
        impl->isAccFkRequested |= requestAccFK;
    }
    impl->updateFlags.reset();

    impl->sigKinematicStateChanged.request();
}


void BodyItem::notifyKinematicStateChange
(Connection& connectionToBlock, bool requestFK, bool requestVelFK, bool requestAccFK)
{
    impl->sigKinematicStateChanged.requestBlocking(connectionToBlock);
    notifyKinematicStateChange(requestFK, requestVelFK, requestAccFK);
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
    enableCollisionDetection(on);
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
    enableSelfCollisionDetection(on);
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
        }
    }
    collisionLinkBitSet_.reset();
}


ItemPtr BodyItem::doDuplicate() const
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
        Link* baseLink = 0;
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

    optionCheckConnections.add(
        linkVisibilityCheck->sigToggled().connect(
            boost::bind(&BodyItemImpl::onLinkVisibilityCheckToggled, this)));
    onLinkVisibilityCheckToggled();

    optionCheckConnections.add(
        showVisualShapeCheck->sigToggled().connect(
            boost::bind(&BodyItemImpl::onVisibleShapeTypesChanged, this)));

    optionCheckConnections.add(
        showCollisionShapeCheck->sigToggled().connect(
            boost::bind(&BodyItemImpl::onVisibleShapeTypesChanged, this)));
    
    if(!showVisualShapeCheck->isChecked() || showCollisionShapeCheck->isChecked()){
        onVisibleShapeTypesChanged();
    }
}


SgNode* BodyItem::getScene()
{
    return sceneBody();
}


EditableSceneBody* BodyItem::existingSceneBody()
{
    return impl->sceneBody;
}


void BodyItemImpl::onLinkVisibilityCheckToggled()
{
    LinkSelectionView* selectionView = LinkSelectionView::mainInstance();

    connectionToSigLinkSelectionChanged.disconnect();
    
    if(linkVisibilityCheck->isChecked()){
        if(sceneBody){
            sceneBody->setLinkVisibilities(selectionView->getLinkSelection(self));
        }
        connectionToSigLinkSelectionChanged.reset(
            selectionView->sigSelectionChanged(self).connect(
                boost::bind(&BodyItemImpl::onLinkSelectionChanged, this)));
    } else {
        if(sceneBody){
            boost::dynamic_bitset<> visibilities;
            visibilities.resize(body->numLinks(), true);
            sceneBody->setLinkVisibilities(visibilities);
        }
    }
}


void BodyItemImpl::onVisibleShapeTypesChanged()
{
    if(sceneBody){
        sceneBody->setVisibleShapeTypes(
            showVisualShapeCheck->isChecked(),
            showCollisionShapeCheck->isChecked());
    }
}


void BodyItemImpl::onLinkSelectionChanged()
{
    if(sceneBody && linkVisibilityCheck->isChecked()){
        sceneBody->setLinkVisibilities(LinkSelectionView::mainInstance()->getLinkSelection(self));
    }
}


bool BodyItemImpl::onEditableChanged(bool on)
{
    self->setEditable(on);
    return true;
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
                (boost::bind(&BodyItemImpl::onStaticModelPropertyChanged, this, _1)));
    putProperty(_("Model file"), getFilename(boost::filesystem::path(self->filePath())));
    putProperty(_("Collision detection"), isCollisionDetectionEnabled,
                (boost::bind(&BodyItemImpl::enableCollisionDetection, this, _1)));
    putProperty(_("Self-collision detection"), isSelfCollisionDetectionEnabled,
                (boost::bind(&BodyItemImpl::enableSelfCollisionDetection, this, _1)));
    putProperty(_("Editable"), isEditable, boost::bind(&BodyItemImpl::onEditableChanged, this, _1));
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
    int n = body->numJoints();
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
        for(int i=0; i < initialJointPositions.size(); ++i){
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
    bool restored = false;
    
    string modelFile;
    /*
      if(archive.readRelocatablePath("modelFile", modelFile)){
      restored = modelFile.empty() || load(modelFile);
      }
    */
    if(archive.readRelocatablePath("modelFile", modelFile)){
        restored = self->load(modelFile);
    }

    if(restored){

        Vector3 p;
        if(read(archive, "rootPosition", p)){
            body->rootLink()->p() = p;
        }
        Matrix3 R;
        if(read(archive, "rootAttitude", R)){
            body->rootLink()->R() = R;
        }
        Listing* qs = archive.findListing("jointPositions");
        if(qs->isValid()){
            for(int i=0; i < qs->size(); ++i){
                body->joint(i)->q() = (*qs)[i].toDouble();
            }
        }

        //! \todo replace the following code with the ValueTree serialization function of BodyState
        initialState.clear();

        if(read(archive, "initialRootPosition", p) && read(archive, "initialRootAttitude", R)){
            initialState.setRootLinkPosition(SE3(p, R));
        }
        qs = archive.findListing("initialJointPositions");
        if(qs->isValid()){
            BodyState::Data& q = initialState.data(BodyState::JOINT_POSITIONS);
            q.resize(qs->size());
            for(int i=0; i < qs->size(); ++i){
                q[i] = (*qs)[i].toDouble();
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
    }

    return restored;
}
