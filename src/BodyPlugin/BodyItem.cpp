/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "BodyItem.h"
#include "WorldItem.h"
#include "KinematicsBar.h"
#include "EditableSceneBody.h"
#include "LinkSelectionView.h"
#include "LinkKinematicsKitManager.h"
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
#include <cnoid/ItemFileIO>
#include <cnoid/ComboBox>
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
#include <cnoid/AttachmentDevice>
#include <cnoid/HolderDevice>
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

BodyState kinematicStateCopy;

class MyCompositeBodyIK : public CompositeBodyIK
{
public:
    MyCompositeBodyIK(BodyItem::Impl* bodyItemImpl);
    virtual bool calcInverseKinematics(const Position& T) override;
    virtual std::shared_ptr<InverseKinematics> getParentBodyIK() override;

    BodyItem::Impl* bodyItemImpl;
    AttachmentDevicePtr attachment;
    shared_ptr<InverseKinematics> holderIK;
};

}

namespace cnoid {

class BodyItem::Impl
{
public:
    BodyItem* self;
    BodyPtr body;

    BodyItem* parentBodyItem;
    AttachmentDevicePtr attachmentToParent;
    ScopedConnection parentBodyItemConnection;
    bool isKinematicStateChangeNotifiedByParentBodyItem;
    bool isProcessingInverseKinematicsIncludingParentBody;

    enum { UF_POSITIONS, UF_VELOCITIES, UF_ACCELERATIONS, UF_CM, UF_ZMP, NUM_UPUDATE_FLAGS };
    std::bitset<NUM_UPUDATE_FLAGS> updateFlags;

    LazySignal<Signal<void()>> sigKinematicStateChanged;
    LazySignal<Signal<void()>> sigKinematicStateEdited;

    LinkPtr currentBaseLink;
    LinkTraverse fkTraverse;
    unique_ptr<LinkKinematicsKitManager> linkKinematicsKitManager;
    shared_ptr<PinDragIK> pinDragIK;

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

    KinematicsBar* kinematicsBar;
    EditableSceneBodyPtr sceneBody;
    bool isSceneBodyDraggableByDefault;

    Signal<void()> sigModelUpdated;

    LeggedBodyHelperPtr legged;
    Vector3 zmp;

    Impl(BodyItem* self);
    Impl(BodyItem* self, const Impl& org);
    Impl(BodyItem* self, Body* body);
    ~Impl();
    void init(bool calledFromCopyConstructor);
    void initBody(bool calledFromCopyConstructor);
    bool loadModelFile(const std::string& filename);
    void setBody(Body* body);
    bool makeBodyStatic(bool makeAllJointsFixed = false);
    bool makeBodyDynamic();
    void setCurrentBaseLink(Link* link, bool forceUpdate = false);
    void appendKinematicStateToHistory();
    bool undoKinematicState();
    bool redoKinematicState();
    LinkKinematicsKitManager* getOrCreateLinkKinematicsKitManager();
    LinkKinematicsKit* getLinkKinematicsKit(Link* baseLink, Link* endLink);
    std::shared_ptr<InverseKinematics> getCurrentIK(Link* targetLink);
    std::shared_ptr<InverseKinematics> getDefaultIK(Link* targetLink);
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
    void createSceneBody();
    bool isSceneBodyDraggable() const;
    void setSceneBodyDraggable(bool on, bool doNotify);
    void setParentBodyItem(BodyItem* bodyItem);
    Link* attachToBodyItem(BodyItem* bodyItem);
    void setRelativeOffsetPositionFromParentBody();
    void onParentBodyKinematicStateChanged();
    void doPutProperties(PutPropertyFunction& putProperty);
    bool store(Archive& archive);
    bool restore(const Archive& archive);
};

}

namespace {

class BodyItemFileIO : public ItemFileIOBase<BodyItem>
{
    BodyLoader bodyLoader;
    Selection meshLengthUnitHint;
    Selection meshUpperAxisHint;
    QWidget* optionPanel;
    ComboBox* unitCombo;
    ComboBox* axisCombo;
    
public:
    BodyItemFileIO()
        : ItemFileIOBase<BodyItem>(
            "CHOREONOID-BODY",
            Load | Options | OptionPanelForLoading),
          meshLengthUnitHint(BodyLoader::NumLengthUnitIds),
          meshUpperAxisHint(BodyLoader::NumUpperAxisIds, CNOID_GETTEXT_DOMAIN_NAME)
    {
        setCaption(_("Body"));
        setExtensions({ "body", "scen", "wrl", "yaml", "yml", "dae", "stl" });
        addFormatIdAlias("OpenHRP-VRML-MODEL");

        bodyLoader.setMessageSink(os());
        
        meshLengthUnitHint.setSymbol(BodyLoader::Meter, "meter");
        meshLengthUnitHint.setSymbol(BodyLoader::Millimeter, "millimeter");
        meshLengthUnitHint.setSymbol(BodyLoader::Inch, "inch");

        meshUpperAxisHint.setSymbol(BodyLoader::Z, "Z");
        meshUpperAxisHint.setSymbol(BodyLoader::Y, "Y");

        optionPanel = nullptr;
        unitCombo = nullptr;
        axisCombo = nullptr;
    }

    ~BodyItemFileIO()
    {
        if(optionPanel){
            delete optionPanel;
        }
    }

    virtual void resetOptions() override
    {
        meshLengthUnitHint.select(BodyLoader::Meter);
        meshUpperAxisHint.select(BodyLoader::Z);
    }
        
    virtual bool restoreOptions(const Mapping* archive) override
    {
        string value;
        if(archive->read("meshLengthUnitHint", value)){
            meshLengthUnitHint.select(value);
        }
        if(archive->read("meshUpperAxisHint", value)){
            meshUpperAxisHint.select(value);
        }
        return true;
    }
        
    virtual void storeOptions(Mapping* archive) override
    {
        if(meshLengthUnitHint != BodyLoader::Meter){
            archive->write("meshLengthUnitHint", meshLengthUnitHint.selectedSymbol());
        }
        if(meshUpperAxisHint != BodyLoader::Z){
            archive->write("meshUpperAxisHint", meshUpperAxisHint.selectedSymbol());
        }
    }
        
    virtual QWidget* getOptionPanelForLoading() override
    {
        if(!optionPanel){
            optionPanel = new QWidget;
            auto hbox = new QHBoxLayout;
            hbox->setContentsMargins(0, 0, 0, 0);
            optionPanel->setLayout(hbox);
            hbox->addWidget(new QLabel(_("[ Mesh import hints ]")));
            hbox->addWidget(new QLabel(_("Unit:")));
            unitCombo = new ComboBox;
            unitCombo->addItem(_("Meter"));
            unitCombo->addItem(_("Millimeter"));
            unitCombo->addItem(_("Inch"));
            hbox->addWidget(unitCombo);
            hbox->addWidget(new QLabel(_("Upper axis:")));
            axisCombo = new ComboBox;
            for(int i=0; i < BodyLoader::NumUpperAxisIds; ++i){
                axisCombo->addItem(meshUpperAxisHint.label(i));
            }
            hbox->addWidget(axisCombo);
        }
        return optionPanel;
    }
        
    virtual void fetchOptionPanelForLoading() override
    {
        meshLengthUnitHint.select(unitCombo->currentIndex());
        meshUpperAxisHint.select(axisCombo->currentIndex());
    }
    
    virtual bool load(BodyItem* item, const std::string& filename) override
    {
        bodyLoader.setMeshImportHint(
            (BodyLoader::LengthUnit)meshLengthUnitHint.which(),
            (BodyLoader::UpperAxis)meshUpperAxisHint.which());
        
        BodyPtr newBody = new Body;
        if(!bodyLoader.load(newBody, filename)){
            return false;
        }
        if(item->name().empty()){
            item->setName(newBody->modelName());
        } else {
            newBody->setName(item->name());
        }
        item->setBody(newBody);

        auto itype = invocationType();
        if(itype == Dialog || itype == DragAndDrop){
            item->setChecked(true);
        }
        
        return true;
    }
};

}


static void onSigOptionsParsed(boost::program_options::variables_map& variables)
{
    if(variables.count("hrpmodel")){
        vector<string> modelFileNames = variables["hrpmodel"].as<vector<string>>();
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
        im.registerFileIO<BodyItem>(new BodyItemFileIO);

        OptionManager& om = ext->optionManager();
        om.addOption("body", boost::program_options::value< vector<string> >(), "load a body file");
        om.sigOptionsParsed().connect(onSigOptionsParsed);

        initialized = true;
    }
}


BodyItem::BodyItem()
{
    impl = new Impl(this);
    impl->init(false);
}
    

BodyItem::Impl::Impl(BodyItem* self)
    : Impl(self, new Body)
{
    isCollisionDetectionEnabled = true;
    isSelfCollisionDetectionEnabled = false;
    isSceneBodyDraggableByDefault = true;
}


BodyItem::BodyItem(const BodyItem& org)
    : Item(org)
{
    impl = new Impl(this, *org.impl);
    impl->init(true);
}


BodyItem::Impl::Impl(BodyItem* self, const Impl& org)
    : Impl(self, org.body->clone())
{
    if(org.currentBaseLink){
        setCurrentBaseLink(body->link(org.currentBaseLink->index()), true);
    } else {
        setCurrentBaseLink(nullptr, true);
    }
        
    zmp = org.zmp;
    isCollisionDetectionEnabled = org.isCollisionDetectionEnabled;
    isSelfCollisionDetectionEnabled = org.isSelfCollisionDetectionEnabled;
    isSceneBodyDraggableByDefault = org.isSceneBodyDraggableByDefault;

    initialState = org.initialState;
}


BodyItem::Impl::Impl(BodyItem* self, Body* body)
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


BodyItem::Impl::~Impl()
{

}


void BodyItem::Impl::init(bool calledFromCopyConstructor)
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


void BodyItem::Impl::initBody(bool calledFromCopyConstructor)
{
    parentBodyItem = nullptr;
    isKinematicStateChangeNotifiedByParentBodyItem = false;
    isProcessingInverseKinematicsIncludingParentBody = false;
    
    if(pinDragIK){
        pinDragIK.reset();
    }

    int n = body->numLinks();

    self->collisionsOfLink_.resize(n);
    self->collisionLinkBitSet_.resize(n);
    
    if(!calledFromCopyConstructor){
        setCurrentBaseLink(nullptr, true);
        zmp.setZero();
        self->storeInitialState();
    }
}


Body* BodyItem::body() const
{
    return impl->body.get();
}


void BodyItem::setBody(Body* body)
{
    impl->setBody(body);
}


void BodyItem::Impl::setBody(Body* body_)
{
    body = body_;
    body->initializePosition();
    body->setCurrentTimeFunction([](){ return TimeBar::instance()->time(); });

    if(!sceneBody){
        isSceneBodyDraggableByDefault = !body->isStaticModel();
    }
    self->setEditable(!body->isStaticModel());

    initBody(false);
}


void BodyItem::setName(const std::string& name)
{
    if(impl->body){
        impl->body->setName(name);
    }
    Item::setName(name);
}


bool BodyItem::makeBodyStatic()
{
    return impl->makeBodyStatic(false);
}


bool BodyItem::Impl::makeBodyStatic(bool makeAllJointsFixed)
{
    bool isStatic = body->isStaticModel();
    if(!isStatic){
        bool hasMovableJoint = false;
        if(body->numLinks() > 1){
            if(makeAllJointsFixed){
                for(auto& link : body->links()){
                    link->setJointType(Link::FixedJoint);
                }
            } else {
                for(auto& link : body->links()){
                    if(!link->isFixedJoint()){
                        hasMovableJoint = true;
                        break;
                    }
                }
            }
        }
        if(!hasMovableJoint){
            body->rootLink()->setJointType(Link::FIXED_JOINT);
            body->updateLinkTree();
            isStatic = true;
        }
    }
    return isStatic;
}
     
    
bool BodyItem::makeBodyDynamic()
{
    return impl->makeBodyDynamic();
}


bool BodyItem::Impl::makeBodyDynamic()
{
    bool isDynamic = !body->isStaticModel();
    if(!isDynamic){
        bool hasFixedJoint = false;
        if(body->numLinks() > 1){
            for(auto& link : body->links()){
                if(link->isFixedJoint()){
                    hasFixedJoint = true;
                    break;
                }
            }
        }
        // If the body has a fixed joint, the conversion to the dynamic
        // model is impossible because which joint type shoulde be used
        // for each fixed joint is unkown.
        if(!hasFixedJoint){
            body->rootLink()->setJointType(Link::FREE_JOINT);
            body->updateLinkTree();
            isDynamic = true;
        }
    }
    return isDynamic;
}


bool BodyItem::isEditable() const
{
    return impl->isSceneBodyDraggable();
}


void BodyItem::setEditable(bool on)
{
    impl->setSceneBodyDraggable(on, true);
}


SignalProxy<void()> BodyItem::sigKinematicStateChanged()
{
    return impl->sigKinematicStateChanged.signal();
}


SignalProxy<void()> BodyItem::sigKinematicStateEdited()
{
    return impl->sigKinematicStateEdited.signal();
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
    impl->setCurrentBaseLink(link, false);
}


void BodyItem::Impl::setCurrentBaseLink(Link* link, bool forceUpdate)
{
    if(link != currentBaseLink || forceUpdate){
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


void BodyItem::Impl::appendKinematicStateToHistory()
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


bool BodyItem::Impl::undoKinematicState()
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


bool BodyItem::Impl::redoKinematicState()
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


LinkKinematicsKitManager* BodyItem::Impl::getOrCreateLinkKinematicsKitManager()
{
    if(!linkKinematicsKitManager){
        linkKinematicsKitManager.reset(new LinkKinematicsKitManager(self));
        self->sceneBody()->addChild(linkKinematicsKitManager->scene(), true);
    }
    return linkKinematicsKitManager.get();
}


LinkKinematicsKit* BodyItem::getLinkKinematicsKit(Link* targetLink, Link* baseLink)
{
    return impl->getLinkKinematicsKit(targetLink, baseLink);
}


LinkKinematicsKit* BodyItem::Impl::getLinkKinematicsKit(Link* targetLink, Link* baseLink)
{
    LinkKinematicsKit* kit = nullptr;

    if(!targetLink){
        targetLink = body->findUniqueEndLink();
    }
    if(targetLink){
        getOrCreateLinkKinematicsKitManager();
        if(baseLink){
            kit = linkKinematicsKitManager->getOrCreateKinematicsKit(targetLink, baseLink);
        } else {
            if(auto ik = getCurrentIK(targetLink)){
                kit = linkKinematicsKitManager->getOrCreateKinematicsKit(targetLink, ik);
            }
        }
    }

    return kit;
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
    return impl->getCurrentIK(targetLink);
}


std::shared_ptr<InverseKinematics> BodyItem::Impl::getCurrentIK(Link* targetLink)
{
    std::shared_ptr<InverseKinematics> ik;
    
    auto rootLink = body->rootLink();
    
    if(attachmentToParent && targetLink == rootLink){
        ik = make_shared<MyCompositeBodyIK>(this);
    }

    if(!ik){
        if(KinematicsBar::instance()->mode() == KinematicsBar::AUTO_MODE){
            ik = getDefaultIK(targetLink);
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
        ik = JointPath::getCustomPath(body, baseLink, targetLink);
    }

    return ik;
}


std::shared_ptr<InverseKinematics> BodyItem::getDefaultIK(Link* targetLink)
{
    return impl->getDefaultIK(targetLink);
}


std::shared_ptr<InverseKinematics> BodyItem::Impl::getDefaultIK(Link* targetLink)
{
    if(!targetLink){
        return nullptr;
    }

    if(attachmentToParent && targetLink == body->rootLink()){
        return make_shared<MyCompositeBodyIK>(this);
    }
    
    std::shared_ptr<InverseKinematics> ik;
    const Mapping& setupMap = *body->info()->findMapping("defaultIKsetup");
    if(setupMap.isValid()){
        const Listing& setup = *setupMap.findListing(targetLink->name());
        if(setup.isValid() && !setup.empty()){
            Link* baseLink = body->link(setup[0].toString());
            if(baseLink){
                if(setup.size() == 1){
                    ik = JointPath::getCustomPath(body, baseLink, targetLink);
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
    return ik;
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


void BodyItem::Impl::setPresetPose(BodyItem::PresetPoseID id)
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
    if(!impl->updateFlags.test(BodyItem::Impl::UF_CM)){
        impl->body->calcCenterOfMass();
        impl->updateFlags.set(BodyItem::Impl::UF_CM);
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


bool BodyItem::Impl::setStance(double width)
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


void BodyItem::Impl::notifyKinematicStateChange(bool requestFK, bool requestVelFK, bool requestAccFK, bool isDirect)
{
    if(!isCallingSlotsOnKinematicStateEdited){
        isCurrentKinematicStateInHistory = false;
    }

    updateFlags.reset();

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


void BodyItem::Impl::emitSigKinematicStateEdited()
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


bool BodyItem::Impl::enableCollisionDetection(bool on)
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


bool BodyItem::Impl::enableSelfCollisionDetection(bool on)
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


Item* BodyItem::doDuplicate() const
{
    return new BodyItem(*this);
}


void BodyItem::doAssign(Item* srcItem)
{
    Item::doAssign(srcItem);
    impl->doAssign(srcItem);
}


void BodyItem::Impl::doAssign(Item* srcItem)
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
    auto ownerBodyItem = findOwnerItem<BodyItem>();
    if(ownerBodyItem != impl->parentBodyItem){
        impl->setParentBodyItem(ownerBodyItem);
    }
    auto worldItem = findOwnerItem<WorldItem>();
    if(!worldItem){
        clearCollisions();
    }
}


SignalProxy<void()> BodyItem::sigLocationChanged()
{
    return impl->sigKinematicStateChanged.signal();
}


Position BodyItem::getLocation() const
{
    return impl->body->rootLink()->position();
}


void BodyItem::setLocation(const Position& T)
{
    impl->body->rootLink()->setPosition(T);
    notifyKinematicStateChange(true);
}


bool BodyItem::isLocationEditable() const
{
    return !isAttachedToParentBody();
}
        

EditableSceneBody* BodyItem::sceneBody()
{
    if(!impl->sceneBody){
        impl->createSceneBody();
    }
    return impl->sceneBody;
}


void BodyItem::Impl::createSceneBody()
{
    sceneBody = new EditableSceneBody(self);
    sceneBody->setSceneDeviceUpdateConnection(true);
    sceneBody->setDraggable(isSceneBodyDraggableByDefault);
}


SgNode* BodyItem::getScene()
{
    return sceneBody();
}


EditableSceneBody* BodyItem::existingSceneBody()
{
    return impl->sceneBody;
}


bool BodyItem::Impl::isSceneBodyDraggable() const
{
    return sceneBody ? sceneBody->isDraggable() : isSceneBodyDraggableByDefault;
}

    
void BodyItem::Impl::setSceneBodyDraggable(bool on, bool doNotify)
{
    if(on != isSceneBodyDraggable()){
        if(sceneBody){
            sceneBody->setDraggable(on);
        } else {
            isSceneBodyDraggableByDefault = on;
        }
        if(doNotify){
            self->notifyUpdate();
        }
    }
}


BodyItem* BodyItem::parentBodyItem()
{
    return impl->parentBodyItem;
}


bool BodyItem::isAttachedToParentBody() const
{
    return impl->attachmentToParent != nullptr;
}


void BodyItem::resetParentBodyItem()
{
    impl->setParentBodyItem(findOwnerItem<BodyItem>());
}


void BodyItem::Impl::setParentBodyItem(BodyItem* bodyItem)
{
    if(!bodyItem){
        if(attachmentToParent){
            auto holderLink = attachmentToParent->holder()->link();
            mvout() << format(_("{0} has been detached from {1} of {2}."),
                              self->name(), holderLink->name(), holderLink->body()->name()) << endl;
        }
    }

    parentBodyItem = bodyItem;
    body->resetParent();
    parentBodyItemConnection.disconnect();
    attachmentToParent = nullptr;
    for(auto& attachment : body->devices<AttachmentDevice>()){
        attachment->detach();
    }

    if(parentBodyItem){
        auto linkToAttach = attachToBodyItem(parentBodyItem);
        if(!linkToAttach){
            linkToAttach = parentBodyItem->body()->rootLink();
            setRelativeOffsetPositionFromParentBody();
        }
        body->setParent(linkToAttach);
        parentBodyItemConnection =
            parentBodyItem->sigKinematicStateChanged().connect(
                [&](){ onParentBodyKinematicStateChanged(); });
        onParentBodyKinematicStateChanged();
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
                    attachmentToParent = attachment;
                    linkToAttach = holder->link();
                    Position T_offset = holder->T_local() * attachment->T_local().inverse(Eigen::Isometry);
                    body->rootLink()->setOffsetPosition(T_offset);
                    mvout() << format(_("{0} has been attached to {1} of {2}."),
                                      self->name(), linkToAttach->name(), bodyItem->name()) << endl;
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


MyCompositeBodyIK::MyCompositeBodyIK(BodyItem::Impl* bodyItemImpl)
    : bodyItemImpl(bodyItemImpl),
      attachment(bodyItemImpl->attachmentToParent)
{
    auto holderLink = attachment->holder()->link();
    holderIK = bodyItemImpl->parentBodyItem->getCurrentIK(holderLink);
}


bool MyCompositeBodyIK::calcInverseKinematics(const Position& T)
{
    bool result = false;
    if(holderIK){
        if(auto holder = attachment->holder()){
            Position Ta = T * attachment->T_local() * holder->T_local().inverse(Eigen::Isometry);
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
    putProperty(_("Static"), body->isStaticModel(),
                [&](bool on){ return on ? makeBodyStatic() : makeBodyDynamic(); });
    putProperty(_("Collision detection"), isCollisionDetectionEnabled,
                [&](bool on){ return enableCollisionDetection(on); });
    putProperty(_("Self-collision detection"), isSelfCollisionDetectionEnabled,
                [&](bool on){ return enableSelfCollisionDetection(on); });
    putProperty(_("Draggable"), isSceneBodyDraggable(),
                [&](bool on){ setSceneBodyDraggable(on, false); return true; });
}


bool BodyItem::store(Archive& archive)
{
    return impl->store(archive);
}


bool BodyItem::Impl::store(Archive& archive)
{
    archive.setDoubleFormat("% .6f");

    archive.writeRelocatablePath("modelFile", self->filePath());
    archive.write("format", self->fileFormat());
    if(auto fileOptions = self->fileOptions()){
        archive.insert(fileOptions);
    }

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
            for(size_t i=0; i < initialJointDisplacements.size(); ++i){
                qs->append(degree(initialJointDisplacements[i]), 10, n);
            }
        }
    }

    // Old format. Remove this after version 1.8 is released.
    qs = archive.createFlowStyleListing("jointPositions");
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
        for(size_t i=0; i < initialJointPositions.size(); ++i){
            qs->append(initialJointPositions[i], 10, n);
        }
    }

    archive.write("staticModel", body->isStaticModel());
    archive.write("collisionDetection", isCollisionDetectionEnabled);
    archive.write("selfCollisionDetection", isSelfCollisionDetectionEnabled);
    archive.write("isSceneBodyDraggable", isSceneBodyDraggable());

    if(linkKinematicsKitManager){
        MappingPtr kinematicsNode = new Mapping;
        if(linkKinematicsKitManager->storeState(*kinematicsNode) && !kinematicsNode->empty()){
            archive.insert("linkKinematics", kinematicsNode);
        }
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
    if(!archive.loadItemFile(self, "modelFile", "format")){
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

    //! \todo replace the following code with the ValueTree serialization function of BodyState
    initialState.clear();

    read(archive, "initialRootPosition", p);
    read(archive, "initialRootAttitude", R);
    initialState.setRootLinkPosition(SE3(p, R));

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
                        format(_("Mismatched size of the stored joint positions for {}"), self->name()),
                        MessageView::WARNING);
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
    }

    body->calcForwardKinematics();
    string baseLinkName;
    archive.read("currentBaseLink", baseLinkName);
    setCurrentBaseLink(body->link(baseLinkName));

    bool on;
    if(archive.read("staticModel", on)){
        if(on){
            makeBodyStatic(true);
        } else {
            makeBodyDynamic();
        }
    }

    if(archive.read("collisionDetection", on)){
        enableCollisionDetection(on);
    }
    if(archive.read("selfCollisionDetection", on)){
        enableSelfCollisionDetection(on);
    }

    if(archive.read("isSceneBodyDraggable", isSceneBodyDraggableByDefault) ||
       archive.read("isEditable", isSceneBodyDraggableByDefault)){
        setSceneBodyDraggable(isSceneBodyDraggableByDefault, false);
    }
       
    auto kinematicsNode = archive.findMapping("linkKinematics");
    if(kinematicsNode->isValid()){
        getOrCreateLinkKinematicsKitManager()->restoreState(*kinematicsNode);
    }

    read(archive, "zmp", zmp);
        
    self->notifyKinematicStateChange();

    return true;
}
