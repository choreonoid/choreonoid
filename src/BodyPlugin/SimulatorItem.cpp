#include "SimulatorItem.h"
#include "WorldItem.h"
#include "ControllerItem.h"
#include "SubSimulatorItem.h"
#include "SimulationScriptItem.h"
#include "BodyMotionItem.h"
#include "BodyMotionEngine.h"
#include "MultiDeviceStateSeqEngine.h"
#include "WorldLogFileItem.h"
#include "CollisionSeqItem.h"
#include "CollisionSeqEngine.h"
#include <cnoid/ExtensionManager>
#include <cnoid/ItemManager>
#include <cnoid/MenuManager>
#include <cnoid/TimeSyncItemEngine>
#include <cnoid/RootItem>
#include <cnoid/ItemTreeView>
#include <cnoid/ControllerIO>
#include <cnoid/BodyState>
#include <cnoid/App>
#include <cnoid/TimeBar>
#include <cnoid/MessageView>
#include <cnoid/LazyCaller>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Archive>
#include <cnoid/MultiDeviceStateSeq>
#include <cnoid/ReferencedObjectSeqItem>
#include <cnoid/Timer>
#include <cnoid/ConnectionSet>
#include <cnoid/FloatingNumberString>
#include <cnoid/SceneGraph>
#include <cnoid/SceneView>
#include <cnoid/CloneMap>
#include <cnoid/CollisionDetector>
#include <QThread>
#include <QMutex>
#include <QElapsedTimer>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <set>
#include <deque>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

enum { RESOLUTION_TIMESTEP, RESOLUTION_FRAMERATE, RESOLUTION_TIMEBAR, N_TEMPORARL_RESOLUTION_TYPES };

const char* realtimeSyncModeSymbols[] = { "off", "compensatory", "conservative" };
static const char* timeRangeModeSymbols[] = { "unlimited", "specified", "timebar" };

typedef map<weak_ref_ptr<BodyItem>, SimulationBodyPtr> BodyItemToSimBodyMap;

struct FunctionSet
{
    struct FunctionInfo {
        int id;
        std::function<void()> function;
    };
    vector<FunctionInfo> functions;
    std::mutex mutex;
    SimulatorItem::Impl* simImpl;
    int idCounter;
    bool needToUpdate;
    vector<FunctionInfo> functionsToAdd;
    set<int> registerdIds;
    vector<int> idsToRemove;
        
    FunctionSet(SimulatorItem::Impl* simImpl) : simImpl(simImpl) {
        clear();
    }
    void clear() {
        idCounter = 0;
        needToUpdate = false;
        functions.clear();
        functionsToAdd.clear();
        registerdIds.clear();
        idsToRemove.clear();
    }

    void call(){
        if(needToUpdate){
            updateFunctions();
        }
        const size_t n = functions.size();
        for(size_t i=0; i < n; ++i){
            functions[i].function();
        }
    }

    int add(std::function<void()>& func);
    void remove(int id);
    void updateFunctions();
};

class ControllerInfo : public Referenced, public ControllerIO
{
public:
    ControllerItemPtr controller;
    SimulationBody::Impl* simBodyImpl;
    Body* body_;
    SimulatorItem::Impl* simImpl;

    std::thread controlThread;
    std::condition_variable controlCondition;
    std::mutex controlMutex;
    bool isExitingControlLoopRequested;
    bool isControlRequested;
    bool isControlFinished;
    bool isControlToBeContinued;

    std::mutex logMutex;
    ReferencedPtr lastLogFrameObject;
    unique_ptr<ReferencedObjectSeq> logBuf;
    int logBufFrameOffset;
    ReferencedObjectSeqItemPtr logItem;
    shared_ptr<ReferencedObjectSeq> log;
    bool isLogEnabled_;
    bool isSimulationFromInitialState_;

    ControllerInfo(ControllerItem* controller, SimulationBody::Impl* simBodyImpl);
    ~ControllerInfo();

    virtual std::string controllerName() const override;
    virtual Body* body() override;
    std::ostream& os() const override;
    virtual double timeStep() const override;
    virtual double currentTime() const override;
    virtual std::string optionString() const override;

    virtual std::shared_ptr<BodyMotion> logBodyMotion() override;
    virtual SignalProxy<void()> sigLogFlushRequested() override;

    virtual bool enableLog() override;
    bool isLogEnabled() const;
    virtual void outputLogFrame(Referenced* logFrame) override;
    void flushLog();

    virtual bool isNoDelayMode() const override;
    virtual bool setNoDelayMode(bool on) override;
    virtual bool isSimulationFromInitialState() const override;

    bool waitForControlInThreadToFinish();
    void concurrentControlLoop();    
};

typedef ref_ptr<ControllerInfo> ControllerInfoPtr;

class SimulationLogEngine : public TimeSyncItemEngine
{
public:
    SimulatorItem::Impl* itemImpl;
    TimeSyncItemEngineManager* manager;
    vector<TimeSyncItemEnginePtr> subEngines;;
    CollisionSeqEnginePtr collisionSeqEngine;
    vector<BodyItem::ContinuousKinematicUpdateEntry> bodyItemEntries;
    bool doKeepPlayback;

    SimulationLogEngine(SimulatorItem::Impl* itemImpl);
    void clearSubEngines();
    void addSubEnginesFor(Item* item);
    void addCollisionSeqEngine(CollisionSeqItem* collisionSeqItem);
    virtual void onPlaybackStarted(double /* time */) override;
    virtual bool onTimeChanged(double time) override;
    virtual double onPlaybackStopped(double time, bool isStoppedManually) override;
    virtual bool isTimeSyncAlwaysMaintained() const override;
    void setupBodyItems(bool doStartPlayback);
};

typedef ref_ptr<SimulationLogEngine> SimulationLogEnginePtr;

}

namespace cnoid {

class SimulationBody::Impl
{
public:
    SimulationBody* self;
    BodyPtr body_;
    BodyItemPtr bodyItem;
    BodyItem::ContinuousKinematicUpdateEntry continuousKinematicUpdateEntry;

    vector<ControllerInfoPtr> controllerInfos;
    SimulatorItem::Impl* simImpl;

    bool isActive;
    bool isDynamic;
    bool areShapesCloned;

    unique_ptr<BodyPositionSeq> positionBuf;
    int currentPositionBufIndex;
    int numLinksToRecord;
    int numJointsToRecord;
    
    ScopedConnectionSet deviceStateConnections;
    vector<bool> deviceStateChangeFlag;
    unique_ptr<MultiDeviceStateSeq> deviceStateBuf;

    // For the direct output without recording mode
    unique_ptr<BodyMotionEngineCore> bodyMotionEngine;
    unique_ptr<BodyPositionSeq> lastPositionBuf;
    unique_ptr<MultiDeviceStateSeqEngineCore> deviceStateEngine;
    unique_ptr<MultiDeviceStateSeq> lastDeviceStateBuf;
    bool hasLastPosition;
    bool hasLastDeviceStates;

    ItemPtr parentOfRecordItems;
    string recordItemPrefix;
    shared_ptr<BodyMotion> motion;
    shared_ptr<BodyPositionSeq> positionRecord;
    shared_ptr<MultiDeviceStateSeq> deviceStateRecord;

    Impl(SimulationBody* self, Body* body);
    bool initialize(SimulatorItem* simulatorItem, BodyItem* bodyItem);
    bool initialize(SimulatorItem::Impl* simImpl, ControllerItem* controllerItem);
    void extractAssociatedItems(bool doReset);
    void extractControllerItems(Item* item, ControllerItem* parentControllerItem);
    void copyStateToBodyItem();
    void initializeRecording();
    void initializeRecordBuffers();
    void initializeRecordItems();
    void bufferRecords();
    void bufferBodyPosition(Body* body, BodyPositionSeqFrameBlock& block);
    void flushRecords();
    void flushRecordsToBodyMotionItems();
    void flushRecordsToLastStateBuffers();
    void updateFrontendBodyStatelWithLastRecords(double time);
    void flushRecordsToWorldLogFile(int bufferFrame);
};


class SimulatorItem::Impl : public QThread
{
public:
    SimulatorItem* self;
    WorldItem* worldItem;

    vector<SimulationBodyPtr> allSimBodies;
    vector<SimulationBody*> simBodiesWithBody;
    vector<SimulationBody*> activeSimBodies;
    vector<ControllerInfoPtr> loggedControllerInfos;

    BodyItemToSimBodyMap simBodyMap;

    Selection temporalResolutionType;
    FloatingNumberString timeStepProperty;
    int frameRateProperty;

    int currentFrame;
    double currentTime_;
    double worldFrameRate;
    double worldTimeStep_;
    int frameAtLastBufferWriting;
    int numBufferedFrames;
    Timer flushTimer;
    Signal<void()> sigLogFlushRequested;

    FunctionSet preDynamicsFunctions;
    FunctionSet midDynamicsFunctions;
    FunctionSet postDynamicsFunctions;
    
    vector<SimulationBody::Impl*> simBodyImplsToNotifyRecords;
    ItemList<SubSimulatorItem> subSimulatorItems;

    vector<ControllerInfoPtr> activeControllerInfos;
    bool doStopSimulationWhenNoActiveControllers;
    bool hasControllers; // Includes non-active controllers

    CollisionDetectorPtr collisionDetector;

    shared_ptr<CollisionSeq> collisionSeq;
    deque<shared_ptr<CollisionLinkPairList>> collisionPairsBuf;

    Selection recordingMode;
    Selection timeRangeMode;
    Selection realtimeSyncMode;
    double timeLength;
    int maxFrame;
    int ringBufferSize;
    int currentRealtimeSyncMode;
    bool isRecordingEnabled;
    bool isRingBufferMode;
    bool isActiveControlTimeRangeMode;
    bool useControllerThreads;
    bool useControllerThreadsProperty;
    bool isAllLinkPositionOutputMode;
    bool isDeviceStateOutputEnabled;
    bool isDoingSimulationLoop;
    volatile bool stopRequested;
    volatile bool pauseRequested;
    bool isCollisionDataRecordingEnabled;
    bool doRecordCollisionData;
    bool isSceneViewEditModeBlockedDuringSimulation;

    string controllerOptionString_;

    TimeBar* timeBar;
    QMutex recordBufMutex;
    double actualSimulationTime;
    double finishTime;
    MessageView* mv;

    bool isSimulationFromInitialState;
    bool isWaitingForSimulationToStop;
    bool isForcedToStopSimulation;
    Signal<void()> sigSimulationAboutToBeStarted;
    Signal<void()> sigSimulationStarted;
    Signal<void()> sigSimulationPaused;
    Signal<void()> sigSimulationResumed;
    Signal<void(bool isForced)> sigSimulationFinished;

    WorldLogFileItemPtr worldLogFileItem;
    int nextLogFrame;
    double nextLogTime;
    double logTimeStep;
    
    stdx::optional<int> extForceFunctionId;
    std::mutex extForceMutex;
    struct ExtForceInfo {
        Link* link;
        Vector3 point;
        Vector3 f;
        double time;
    };
    ExtForceInfo extForceInfo;

    stdx::optional<int> virtualElasticStringFunctionId;
    std::mutex virtualElasticStringMutex;
    struct VirtualElasticString {
        Link* link;
        double kp;
        double kd;
        double f_max;
        Vector3 point;
        Vector3 goal;
    };
    VirtualElasticString virtualElasticString;

    SimulationLogEnginePtr logEngine;
    
    Connection aboutToQuitConnection;

    CloneMap cloneMap;
        
    Impl(SimulatorItem* self);
    Impl(SimulatorItem* self, const Impl& org);
    ~Impl();
    void findTargetItems(Item* item, bool isUnderBodyItem, ItemList<Item>& out_targetItems);
    void clearSimulation();
    void setSimulatorItemToControllerItem(ControllerItem* controllerItem);
    void resetSimulatorItemForControllerItem(ControllerItem* controllerItem);
    bool startSimulation(bool doReset);
    bool initializeSimulation(bool doReset);
    virtual void run() override;
    void onSimulationLoopStarted();
    void updateSimBodyLists();
    bool stepSimulationMain();
    void bufferRecords();
    void bufferCollisionRecords();
    void startFlushTimer();
    void flushRecords();
    int flushMainRecords();
    void stopSimulation(bool isForced, bool doSync);
    void pauseSimulation();
    void restartSimulation();
    void onSimulationLoopStopped(bool isForced);
    bool isActive() const;
    void setExternalForce(BodyItem* bodyItem, Link* link, const Vector3& point, const Vector3& f, double time);
    void doSetExternalForce();
    void setVirtualElasticString(
        BodyItem* bodyItem, Link* link, const Vector3& attachmentPoint, const Vector3& endPoint);
    void setVirtualElasticStringForce();
    void onSlowerThanRealtimeEnabledChanged(bool on);
    bool onAllLinkPositionOutputModeChanged(bool on);
    void doPutProperties(PutPropertyFunction& putProperty);
    bool store(Archive& archive);
    bool restore(const Archive& archive);
    void restoreTimeSyncItemEngines(const Archive& archive);
    SimulationLogEngine* getOrCreateLogEngine();
};

}

namespace {

/**
   This is a class for executing a script as a controller
   \todo Reimplement this class as a SubSimulatorItem?
*/
class ScriptControllerItem : public ControllerItem
{
    ControllerIO* io;
    double time;
    double timeStep_;
    double delay;
    SimulationScriptItemPtr scriptItem;
    LazyCaller executeLater;
    bool doExecAfterInit;

public:
    ScriptControllerItem(SimulationScriptItem* scriptItem){
        this->scriptItem = scriptItem;
        doExecAfterInit = false;
    }

protected:
    virtual Item* doCloneItem(CloneMap* cloneMap) const override { return nullptr; }

    virtual bool initialize(ControllerIO* io) override {
        this->io = io;
        return true;
    }
    
    virtual bool start() override {
        timeStep_ = io->timeStep();
        if(scriptItem->executionTiming() == SimulationScriptItem::DURING_INITIALIZATION){
            scriptItem->executeAsSimulationScript();
        } else if(scriptItem->executionTiming() == SimulationScriptItem::AFTER_INITIALIZATION){
            doExecAfterInit = true;
            time = 0.0;
            delay = scriptItem->executionDelay();
            executeLater.setFunction([&](){ execute(); });
        }
        return true;
    }
    
    virtual double timeStep() const override {
        return timeStep_;
    }
    
    virtual void input() override {
    }
    
    virtual bool control() override {
        if(doExecAfterInit){
            if(time >= delay){
                executeLater();
                doExecAfterInit = false;
            }
            time += timeStep_;
            return true;
        }
        return false;
    }
    
    void execute(){
        scriptItem->executeAsSimulationScript();
    }
    
    virtual void output() override {
    }
    
    virtual void stop() override {
        if(scriptItem->executionTiming() == SimulationScriptItem::DURING_FINALIZATION){
            scriptItem->executeAsSimulationScript();
        }
    }
};

}


void SimulatorItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerAbstractClass<SimulatorItem>();

    TimeSyncItemEngineManager::instance()
        ->registerFactory<SimulatorItem, SimulationLogEngine>(
            [](SimulatorItem* item, SimulationLogEngine* /* engine0 */){
                return item->impl->getOrCreateLogEngine();
            });

    ItemTreeView::customizeContextMenu<SimulatorItem>(
        [](SimulatorItem* item, MenuManager& menuManager, ItemFunctionDispatcher menuFunction){
            menuManager.setPath("/").setPath(_("Simulation"));
            menuManager.addItem(_("Start"))->sigTriggered().connect(
                [item](){ item->startSimulation(); });
            menuManager.addItem(_("Pause"))->sigTriggered().connect(
                [item](){ item->pauseSimulation(); });
            menuManager.addItem(_("Resume"))->sigTriggered().connect(
                [item](){ item->restartSimulation(); });
            menuManager.addItem(_("Finish"))->sigTriggered().connect(
                [item](){ item->stopSimulation(true); });
            menuManager.setPath("/");
            menuManager.addSeparator();
            menuFunction.dispatchAs<Item>(item);
        });
}


SimulatorItem* SimulatorItem::findActiveSimulatorItemFor(Item* item)
{
    SimulatorItem* activeSimulatorItem = nullptr;
    if(item){
        if(auto worldItem = item->findOwnerItem<WorldItem>()){
            activeSimulatorItem = worldItem->findItem<SimulatorItem>(
                [](SimulatorItem* item){ return item->isActive(); });
        }
    }
    return activeSimulatorItem;
}


ControllerInfo::ControllerInfo(ControllerItem* controller, SimulationBody::Impl* simBodyImpl)
    : controller(controller),
      simBodyImpl(simBodyImpl),
      body_(simBodyImpl->body_),
      simImpl(simBodyImpl->simImpl),
      isLogEnabled_(false),
      isSimulationFromInitialState_(simImpl->isSimulationFromInitialState)
{
    if(controller){
        // ControllerInfo cannot directly set a simulator item to the controller item
        // because ControllerItem::setSimulatorItem is a private function.
        simImpl->setSimulatorItemToControllerItem(controller);
    }
}


ControllerInfo::~ControllerInfo()
{
    if(controller){
        // ControllerInfo cannot directly reset a simulator item for the controller item
        // because ControllerItem::setSimulatorItem is a private function.
        simImpl->resetSimulatorItemForControllerItem(controller);
    }
}


std::string ControllerInfo::controllerName() const
{
    return controller ? controller->name() : string();
}


Body* ControllerInfo::body()
{
    return body_;
}


std::ostream& ControllerInfo::os() const
{
    return simImpl->mv->cout();
}
    

double ControllerInfo::timeStep() const
{
    return simImpl->worldTimeStep_;
}
    

double ControllerInfo::currentTime() const
{
    return simImpl->currentTime_;
}


std::string ControllerInfo::optionString() const
{
    return simImpl->controllerOptionString_;
}


std::shared_ptr<BodyMotion> ControllerInfo::logBodyMotion()
{
    return simBodyImpl->motion;
}


SignalProxy<void()> ControllerInfo::sigLogFlushRequested()
{
    return simImpl->sigLogFlushRequested;
}


bool ControllerInfo::enableLog()
{
    logBuf.reset(new ReferencedObjectSeq);
    logBuf->setFrameRate(simImpl->worldFrameRate);
    logBufFrameOffset = 0;

    string logName = simImpl->self->name() + "-" + controller->name();
    logItem = controller->findChildItem<ReferencedObjectSeqItem>(logName);
    if(logItem){
        logItem->resetSeq();
        if(!logItem->isTemporary()){
            logItem->setTemporary();
            logItem->notifyUpdate();
        }
    } else {
        logItem = controller->createLogItem();
        logItem->setTemporary();
        logItem->setName(logName);
        controller->addChildItem(logItem);
    }
    log = logItem->seq();
    log->setNumFrames(0);
    log->setFrameRate(simImpl->worldFrameRate);
    log->setOffsetTime(0.0);
    
    simImpl->loggedControllerInfos.push_back(this);
    simImpl->getOrCreateLogEngine()->addSubEnginesFor(logItem);

    isLogEnabled_ = true;

    return true;
}


bool ControllerInfo::isLogEnabled() const
{
    return isLogEnabled_;
}


void ControllerInfo::outputLogFrame(Referenced* logFrame)
{
    std::lock_guard<std::mutex> lock(logMutex);

    if(!lastLogFrameObject){
        lastLogFrameObject = logFrame;
    }

    const int bufFrame = simImpl->currentFrame - logBufFrameOffset;
    const int numFrames = bufFrame + 1;
    const int lastNumFrames = logBuf->numFrames();

    logBuf->setNumFrames(numFrames);

    for(int i = lastNumFrames; i < numFrames - 1; ++i){
        logBuf->at(i) = lastLogFrameObject;
    }
    logBuf->back() = logFrame;

    lastLogFrameObject = logFrame;
}


void ControllerInfo::flushLog()
{
    std::lock_guard<std::mutex> lock(logMutex);

    if(!logBuf->empty()){
        const int numBufFrames = logBuf->numFrames();
        const int offsetFrame = log->numFrames();
        log->setNumFrames(offsetFrame + numBufFrames);
        for(int i=0; i < numBufFrames; ++i){
            log->at(i + offsetFrame) = logBuf->at(i);
        }
        logBuf->clear();
        logBufFrameOffset += numBufFrames;
    }
}


bool ControllerInfo::isNoDelayMode() const
{
    return controller->isNoDelayMode();
}


bool ControllerInfo::setNoDelayMode(bool on)
{
    controller->setNoDelayMode(on);
    return on;
}


bool ControllerInfo::isSimulationFromInitialState() const
{
    return isSimulationFromInitialState_;
}


SimulationBody::SimulationBody(Body* body)
{
    impl = new Impl(this, body);
}


SimulationBody::Impl::Impl(SimulationBody* self, Body* body)
    : self(self),
      body_(body)
{
    simImpl = nullptr;
    areShapesCloned = false;
    isActive = false;
    isDynamic = false;
}


SimulationBody::~SimulationBody()
{
    delete impl;
}


BodyItem* SimulationBody::bodyItem() const
{
    return impl->bodyItem;
}


Body* SimulationBody::body() const
{
    return impl->body_;
}


int SimulationBody::numControllers() const
{
    return impl->controllerInfos.size();
}


ControllerItem* SimulationBody::controller(int index) const
{
    if(index < static_cast<int>(impl->controllerInfos.size())){
        return impl->controllerInfos[index]->controller;
    }
    return nullptr;
}


bool SimulationBody::initialize(SimulatorItem* simulatorItem, BodyItem* bodyItem)
{
    return impl->initialize(simulatorItem, bodyItem);
}


bool SimulationBody::Impl::initialize(SimulatorItem* simulatorItem, BodyItem* bodyItem)
{
    simImpl = simulatorItem->impl;
    this->bodyItem = bodyItem;
    deviceStateConnections.disconnect();
    controllerInfos.clear();
    recordItemPrefix = simImpl->self->name() + "-" + bodyItem->name();

    body_->setCurrentTimeFunction([this](){ return this->simImpl->currentTime_; });
    body_->initializeState();

    isDynamic = !body_->isStaticModel();
    bool doReset = simImpl->isSimulationFromInitialState && isDynamic;
    extractAssociatedItems(doReset);
    
    if(!isDynamic && body_->numDevices() == 0){
        return true;
    }

    isActive = true;
    
    return true;
}


// For a controller which is not associated with a body
bool SimulationBody::Impl::initialize(SimulatorItem::Impl* simImpl, ControllerItem* controllerItem)
{
    bool initialized = false;
    this->simImpl = simImpl;
    simImpl->hasControllers = true;

    ControllerInfoPtr info = new ControllerInfo(controllerItem, this);

    if(controllerItem->initialize(info)){
        controllerInfos.push_back(info);
        initialized = true;
    }

    return initialized;
}


void SimulationBody::Impl::extractAssociatedItems(bool doReset)
{
    extractControllerItems(bodyItem->childItem(), nullptr);
    
    if(controllerInfos.empty()){
        parentOfRecordItems = bodyItem;

    } else if(controllerInfos.size() == 1){
        parentOfRecordItems = controllerInfos.front()->controller;

    } else {
        // find the common owner of all the controllers
        int minDepth = std::numeric_limits<int>::max();
        for(size_t i=0; i < controllerInfos.size(); ++i){
            Item* owner = controllerInfos[i]->controller->parentItem();
            int depth = 0;
            Item* item = owner;
            while(item && item != bodyItem){
                ++depth;
                item = item->parentItem();
            }
            if(depth < minDepth){
                parentOfRecordItems = owner;
                minDepth = depth;
            }
        }
    }
}


void SimulationBody::Impl::extractControllerItems(Item* item, ControllerItem* parentControllerItem)
{
    while(item){
        bool isValidControllerItem = false;
        auto controllerItem = dynamic_cast<ControllerItem*>(item);
        if(controllerItem){
            bool isSubController = false;
            if(parentControllerItem && parentControllerItem->checkIfSubController(controllerItem)){
                isSubController = true;
            }
            if(!isSubController){
                simImpl->hasControllers = true;
                ControllerInfoPtr info = new ControllerInfo(controllerItem, this);
                if(controllerItem->initialize(info)){
                    controllerInfos.push_back(info);
                    isValidControllerItem = true;
                }
            }
        }
        if(auto childItem = item->childItem()){
            if(isValidControllerItem || !bool(dynamic_cast<BodyItem*>(item))){
                extractControllerItems(childItem, controllerItem);
            }
        }
        item = item->nextItem();
    }
}


void SimulationBody::Impl::copyStateToBodyItem()
{
    BodyState state(*body_);
    state.restorePositions(*bodyItem->body());
}


const std::string& SimulationBody::recordItemPrefix() const
{
    return impl->recordItemPrefix;
}


void SimulationBody::Impl::initializeRecording()
{
    self->initializeRecordBuffers();
    
    if(simImpl->isRecordingEnabled){
        self->initializeRecordItems();
    }
}


void SimulationBody::initializeRecordBuffers()
{
    impl->initializeRecordBuffers();
}


void SimulationBody::Impl::initializeRecordBuffers()
{
    currentPositionBufIndex = 0;

    bodyMotionEngine.reset();
    lastPositionBuf.reset();
    hasLastPosition = false;

    if(!isDynamic){
        numLinksToRecord = 0;
        numJointsToRecord = 0;
        positionBuf.reset();
    } else {
        numLinksToRecord = simImpl->isAllLinkPositionOutputMode ? body_->numLinks() : 1;
        numJointsToRecord = body_->numAllJoints();
        positionBuf = make_unique<BodyPositionSeq>();

        if(!simImpl->isRecordingEnabled){
            bodyMotionEngine = make_unique<BodyMotionEngineCore>(bodyItem);
            lastPositionBuf = make_unique<BodyPositionSeq>(1);
        }
    }

    const DeviceList<>& devices = body_->devices();
    const int numDevices = devices.size();
    deviceStateConnections.disconnect();
    deviceStateChangeFlag.clear();
    deviceStateChangeFlag.resize(numDevices, true); // set all the bits to store the initial states
    deviceStateBuf.reset();
    
    deviceStateEngine.reset();
    lastDeviceStateBuf.reset();
    hasLastDeviceStates = false;
    
    if(!devices.empty() && simImpl->isDeviceStateOutputEnabled){
        /**
           Temporary parameters to avoid a bug in storing device states by reserving sufficient buffer size.
           The original bug is in Deque2D's resize operation.
        */
        deviceStateBuf = make_unique<MultiDeviceStateSeq>(100, numDevices);

        // This buf always has the first element to keep unchanged states
        deviceStateBuf->setDimension(1, numDevices); 
        for(size_t i=0; i < devices.size(); ++i){
            deviceStateConnections.add(
                devices[i]->sigStateChanged().connect(
                    [this, i](){
                        /** \note This must be thread safe if notifyStateChange
                                  is called from several threads. */
                        deviceStateChangeFlag[i] = true;
                    }));
            
        }

        if(!simImpl->isRecordingEnabled){
            deviceStateEngine = make_unique<MultiDeviceStateSeqEngineCore>(bodyItem);
            lastDeviceStateBuf = make_unique<MultiDeviceStateSeq>(1, numDevices);
        }
    }
}


void SimulationBody::initializeRecordItems()
{
    impl->initializeRecordItems();
}


void SimulationBody::Impl::initializeRecordItems()
{
    if(!parentOfRecordItems){
        return;
    }

    bool doAddMotionItem = false;
    auto motionItem = parentOfRecordItems->findChildItem<BodyMotionItem>(recordItemPrefix);
    if(motionItem){
        bool updated = false;
        if(!motionItem->isTemporary()){
            motionItem->setTemporary();
            updated = true;
        }
        if(motionItem->motion()->hasExtraSeqs()){
            motionItem->motion()->clearExtraSeqs();
            updated = true;
        }
        if(updated){
            motionItem->notifyUpdate();
        }
    } else {
        motionItem = new BodyMotionItem;
        motionItem->setName(recordItemPrefix);
        motionItem->setTemporary();
        doAddMotionItem = true;
    }

    motion = motionItem->motion();
    motion->setFrameRate(simImpl->worldFrameRate);
    motion->setDimension(0, numJointsToRecord, numLinksToRecord);
    motion->setOffsetTime(0.0);
    positionRecord = motion->positionSeq();

    if(deviceStateBuf){
        deviceStateRecord = getOrCreateMultiDeviceStateSeq(*motion);
        deviceStateRecord->initialize(body_->devices());
    } else {
        clearMultiDeviceStateSeq(*motion);
    }

    if(doAddMotionItem){
        parentOfRecordItems->addChildItem(motionItem);
    }
    simImpl->logEngine->addSubEnginesFor(motionItem);
}


bool SimulationBody::isActive() const
{
    return impl->isActive;
}


void SimulationBody::notifyUnrecordedDeviceStateChange(Device* device)
{
    bool flag = impl->deviceStateChangeFlag[device->index()];
    device->notifyStateChange();
    impl->deviceStateChangeFlag[device->index()] = flag;
}


void SimulationBody::bufferRecords()
{
    impl->bufferRecords();
}


void SimulationBody::Impl::bufferRecords()
{
    if(positionBuf){
        if(currentPositionBufIndex >= positionBuf->numFrames()){
            positionBuf->setNumFrames(currentPositionBufIndex + 1);
        }
        auto& frame = positionBuf->frame(currentPositionBufIndex++);
        if(!body_->existence()){
            frame.clear();
        } else {
            frame.allocate(numLinksToRecord, numJointsToRecord);
            bufferBodyPosition(body_, frame);

            Body* multiplexBody = body_->nextMultiplexBody();
            while(multiplexBody){
                auto block = frame.extend(numLinksToRecord, numJointsToRecord);
                bufferBodyPosition(multiplexBody, block);
                multiplexBody = multiplexBody->nextMultiplexBody();
            }
        }
    }

    if(deviceStateBuf){
        const int prevIndex = std::max(0, deviceStateBuf->numFrames() - 1);
        auto currentFrame = deviceStateBuf->appendFrame();
        auto prevFrame = deviceStateBuf->frame(prevIndex);
        const DeviceList<>& devices = body_->devices();
        for(size_t i=0; i < devices.size(); ++i){
            if(deviceStateChangeFlag[i]){
                currentFrame[i] = devices[i]->cloneState();
                deviceStateChangeFlag[i] = false;
            } else {
                currentFrame[i] = prevFrame[i];
            }
        }
    }
}


void SimulationBody::Impl::bufferBodyPosition(Body* body, BodyPositionSeqFrameBlock& block)
{
    for(int i=0; i < numLinksToRecord; ++i){
        block.linkPosition(i).set(body->link(i)->T());
    }
    auto displacements = block.jointDisplacements();
    for(int i=0; i < numJointsToRecord; ++i){
        displacements[i] = body->joint(i)->q();
    }
}


void SimulationBody::flushRecords()
{
    impl->flushRecords();
}


void SimulationBody::Impl::flushRecords()
{
    if(simImpl->isRecordingEnabled){
        flushRecordsToBodyMotionItems();
    } else {
        flushRecordsToLastStateBuffers();
    }

    currentPositionBufIndex = 0;

    if(deviceStateBuf){
        // keep the last state so that unchanged states can be shared
        const int numFrames = deviceStateBuf->numFrames();
        const int numPops = (numFrames >= 2) ? (numFrames - 1) : 0;
        deviceStateBuf->popFrontFrames(numPops);
    }
}


void SimulationBody::Impl::flushRecordsToBodyMotionItems()
{
    const int ringBufferSize = simImpl->ringBufferSize;
    bool offsetChanged = false;

    for(int i=0; i < currentPositionBufIndex; ++i){
        if(positionRecord->numFrames() < ringBufferSize){
            positionRecord->append();
        } else {
            positionRecord->rotate();
            offsetChanged = true;
        }
        auto& srcFrame = positionBuf->frame(i);
        positionRecord->back() = srcFrame;
    }

    if(deviceStateBuf){
        // This loop begins with the second element to skip the first element to keep the unchanged states
        for(int i=1; i < deviceStateBuf->numFrames(); ++i){ 
            auto buf = deviceStateBuf->frame(i);
            if(deviceStateRecord->numFrames() >= ringBufferSize){
                deviceStateRecord->popFrontFrame();
                offsetChanged = true;
            }
            std::copy(buf.begin(), buf.end(), deviceStateRecord->appendFrame().begin());
        }
    }
    
    if(offsetChanged){
        const int nextFrame = simImpl->currentFrame + 1;
        int offset = nextFrame - ringBufferSize;
        if(positionRecord){
            positionRecord->setOffsetTimeFrame(offset);
        }
        if(deviceStateBuf){
            deviceStateRecord->setOffsetTimeFrame(offset);
        }
    }
}


// This function is called in the no-recording mode.
void SimulationBody::Impl::flushRecordsToLastStateBuffers()
{
    if(currentPositionBufIndex > 0){
        int lastFrame = currentPositionBufIndex - 1;
        lastPositionBuf->frame(0) = positionBuf->frame(lastFrame);
        hasLastPosition = true;
    } else {
        hasLastPosition = false;
    }
    
    if(deviceStateBuf){
        if(!deviceStateBuf->empty()){
            auto lastFrame = deviceStateBuf->lastFrame();
            std::copy(lastFrame.begin(), lastFrame.end(), lastDeviceStateBuf->begin());
            hasLastDeviceStates = true;
        } else {
            hasLastDeviceStates = false;
        }
    }
}


// This function is called in the no-recording mode.
void SimulationBody::Impl::updateFrontendBodyStatelWithLastRecords(double time)
{
    if(hasLastPosition){
        bodyMotionEngine->updateBodyPosition(lastPositionBuf->frame(0));
    }
    if(hasLastDeviceStates){
        deviceStateEngine->updateBodyDeviceStates(simImpl->currentTime_, lastDeviceStateBuf->lastFrame());
    }
}


void SimulationBody::Impl::flushRecordsToWorldLogFile(int bufferFrame)
{
    WorldLogFileItem* log = simImpl->worldLogFileItem;

    log->beginBodyStateOutput();

    if(positionBuf){
        auto& frame = positionBuf->frame(bufferFrame);
        if(numLinksToRecord > 0){
            log->outputLinkPositions(frame.linkPositionData(), numLinksToRecord);
        }
        if(numJointsToRecord > 0){
            log->outputJointPositions(frame.jointDisplacements(), numJointsToRecord);
        }
    }
    
    if(deviceStateBuf){
        // Skip the first element because it is used for sharing an unchanged state
        auto states = deviceStateBuf->frame(bufferFrame + 1);
        log->beginDeviceStateOutput();
        for(int i=0; i < states.size(); ++i){
            log->outputDeviceState(states[i]);
        }
        log->endDeviceStateOutput();
    }

    log->endBodyStateOutput();
}


SimulatorItem::SimulatorItem()
{
    impl = new Impl(this);
}


SimulatorItem::SimulatorItem(const std::string& name)
    : Item(name)
{
    impl = new Impl(this);
}


SimulatorItem::Impl::Impl(SimulatorItem* self)
    : self(self),
      temporalResolutionType(N_TEMPORARL_RESOLUTION_TYPES, CNOID_GETTEXT_DOMAIN_NAME),
      preDynamicsFunctions(this),
      midDynamicsFunctions(this),
      postDynamicsFunctions(this),
      recordingMode(NumRecordingModes, CNOID_GETTEXT_DOMAIN_NAME),
      timeRangeMode(NumTimeRangeModes, CNOID_GETTEXT_DOMAIN_NAME),
      realtimeSyncMode(NumRealtimeSyncModes, CNOID_GETTEXT_DOMAIN_NAME),
      mv(MessageView::instance())
{
    worldItem = nullptr;
    
    temporalResolutionType.setSymbol(RESOLUTION_TIMESTEP, N_("Timestep"));
    temporalResolutionType.setSymbol(RESOLUTION_FRAMERATE, N_("Framerate"));
    temporalResolutionType.setSymbol(RESOLUTION_TIMEBAR, N_("Time bar"));
    timeStepProperty = 0.001;
    frameRateProperty = 1000;

    currentFrame = 0;
    currentTime_ = 0.0;
    worldFrameRate = 1.0;
    worldTimeStep_ = 1.0;
    frameAtLastBufferWriting = 0;
    flushTimer.sigTimeout().connect([&](){ flushRecords(); });

    recordingMode.setSymbol(FullRecording, N_("full"));
    recordingMode.setSymbol(TailRecording, N_("tail"));
    recordingMode.setSymbol(NoRecording, N_("off"));
    recordingMode.select(FullRecording);

    timeRangeMode.setSymbol(UnlimitedTime, N_("Unlimited"));
    timeRangeMode.setSymbol(SpecifiedTime, N_("Specified time"));
    timeRangeMode.setSymbol(TimeBarTime, N_("Time bar range"));
    timeRangeMode.select(UnlimitedTime);

    realtimeSyncMode.setSymbol(NonRealtimeSync, N_("Off"));
    realtimeSyncMode.setSymbol(CompensatoryRealtimeSync, N_("On (Compensatory)"));
    realtimeSyncMode.setSymbol(ConservativeRealtimeSync, N_("On (Conservative)"));
    realtimeSyncMode.select(CompensatoryRealtimeSync);

    timeLength = 180.0; // 3 min.
    useControllerThreadsProperty = true;
    isActiveControlTimeRangeMode = false;
    isAllLinkPositionOutputMode = true;
    isDeviceStateOutputEnabled = true;
    isDoingSimulationLoop = false;
    isCollisionDataRecordingEnabled = false;
    isSceneViewEditModeBlockedDuringSimulation = false;
    isSimulationFromInitialState = false;

    timeBar = TimeBar::instance();
}


SimulatorItem::SimulatorItem(const SimulatorItem& org)
    : Item(org)
{
    impl = new Impl(this, *org.impl);
}


SimulatorItem::Impl::Impl(SimulatorItem* self, const Impl& org)
    : Impl(self)
{
    temporalResolutionType = org.temporalResolutionType;
    timeStepProperty = org.timeStepProperty;
    frameRateProperty = org.frameRateProperty;

    recordingMode = org.recordingMode;
    timeRangeMode = org.timeRangeMode;
    realtimeSyncMode = org.realtimeSyncMode;

    timeLength = org.timeLength;
    useControllerThreadsProperty = org.useControllerThreadsProperty;
    isActiveControlTimeRangeMode = org.isActiveControlTimeRangeMode;
    isAllLinkPositionOutputMode = org.isAllLinkPositionOutputMode;
    isDeviceStateOutputEnabled = org.isDeviceStateOutputEnabled;
    isCollisionDataRecordingEnabled = org.isCollisionDataRecordingEnabled;
    controllerOptionString_ = org.controllerOptionString_;
    isSimulationFromInitialState = false;
}
    

SimulatorItem::~SimulatorItem()
{
    impl->stopSimulation(true, true);
    delete impl;
}


SimulatorItem::Impl::~Impl()
{
    aboutToQuitConnection.disconnect();
}


void SimulatorItem::onTreePathChanged()
{
    impl->worldItem = findOwnerItem<WorldItem>();
}


void SimulatorItem::onDisconnectedFromRoot()
{
    impl->stopSimulation(true, true);
    impl->clearSimulation();
    impl->worldItem = nullptr;
}


WorldItem* SimulatorItem::worldItem()
{
    return impl->worldItem;
}


double SimulatorItem::worldTimeStep()
{
    switch(impl->temporalResolutionType.which()){
    case RESOLUTION_TIMESTEP:
        return impl->timeStepProperty.value();
    case RESOLUTION_FRAMERATE:
        return 1.0 / impl->frameRateProperty;
    case RESOLUTION_TIMEBAR:
    default:
        return impl->timeBar->timeStep();
    }
}


void SimulatorItem::setTimeStep(double step)
{
    if(step > 0.0){
        impl->temporalResolutionType.select(RESOLUTION_TIMESTEP);
        impl->timeStepProperty = step;
    }
}


void SimulatorItem::setRecordingMode(int mode)
{
    impl->recordingMode.select(mode);
}


int SimulatorItem::recordingMode() const
{
    return impl->recordingMode.which();
}


bool SimulatorItem::isRecordingEnabled() const
{
    return impl->isRecordingEnabled;
}


void SimulatorItem::setTimeRangeMode(int mode)
{
    if(mode == ActiveControlTime){
        impl->timeRangeMode.select(UnlimitedTime);
        setActiveControlTimeRangeMode(true);
    } else {
        impl->timeRangeMode.select(mode);
    }
}


void SimulatorItem::setTimeLength(double length)
{
    impl->timeLength = length;
}


void SimulatorItem::setActiveControlTimeRangeMode(bool on)
{
    impl->isActiveControlTimeRangeMode = on;
}


bool SimulatorItem::isActiveControlTimeRangeMode() const
{
    return impl->isActiveControlTimeRangeMode;
}


void SimulatorItem::setRealtimeSyncMode(int mode)
{
    impl->realtimeSyncMode.select(mode);
}


void SimulatorItem::setRealtimeSyncMode(bool on)
{
    impl->realtimeSyncMode.select(on ? CompensatoryRealtimeSync : NonRealtimeSync);
}


void SimulatorItem::setDeviceStateOutputEnabled(bool on)
{
    impl->isDeviceStateOutputEnabled = on;
}


void SimulatorItem::setAllLinkPositionOutputMode(bool on)
{
    impl->isAllLinkPositionOutputMode = on;
}


bool SimulatorItem::isAllLinkPositionOutputMode()
{
    return impl->isAllLinkPositionOutputMode;
}


bool SimulatorItem::isDeviceStateOutputEnabled() const
{
    return impl->isDeviceStateOutputEnabled;
}

CollisionDetector* SimulatorItem::getOrCreateCollisionDetector()
{
    if(impl->collisionDetector){
        return impl->collisionDetector;
    }
    if(impl->worldItem){
        return impl->worldItem->collisionDetector()->clone();
    }
    return CollisionDetector::create(0); // the null collision detector
}


CollisionDetector* SimulatorItem::collisionDetector()
{
    return getOrCreateCollisionDetector();
}


const std::string& SimulatorItem::controllerOptionString() const
{
    return impl->controllerOptionString_;
}


/*
  Extract body items, controller items which are not associated with (not under) a body item,
  and simulation script items which are not under another simulator item
*/
void SimulatorItem::Impl::findTargetItems
(Item* item, bool isUnderBodyItem, ItemList<Item>& out_targetItems)
{
    if(dynamic_cast<BodyItem*>(item)){
        out_targetItems.push_back(item);
        isUnderBodyItem = true;

    } else if(!isUnderBodyItem && dynamic_cast<ControllerItem*>(item)){
        out_targetItems.push_back(item);

    } else if(auto scriptItem = dynamic_cast<SimulationScriptItem*>(item)){
        if(scriptItem->isChecked()){
            if(scriptItem->executionTiming() == SimulationScriptItem::BEFORE_INITIALIZATION){
                scriptItem->executeAsSimulationScript();
            } else {
                out_targetItems.push_back(item);
            }
        }
    }
    auto simulatorItem = dynamic_cast<SimulatorItem*>(item);
    if(!simulatorItem || (simulatorItem == self)){
        for(auto childItem = item->childItem(); childItem; childItem = childItem->nextItem()){
            findTargetItems(childItem, isUnderBodyItem, out_targetItems);
        }
    }
}


int SimulatorItem::addPreDynamicsFunction(std::function<void()> func)
{
    return impl->preDynamicsFunctions.add(func);
}


void SimulatorItem::removePreDynamicsFunction(int id)
{
    impl->preDynamicsFunctions.remove(id);
}


int SimulatorItem::addMidDynamicsFunction(std::function<void()> func)
{
    return impl->midDynamicsFunctions.add(func);
}


void SimulatorItem::removeMidDynamicsFunction(int id)
{
    impl->midDynamicsFunctions.remove(id);
}


int SimulatorItem::addPostDynamicsFunction(std::function<void()> func)
{
    return impl->postDynamicsFunctions.add(func);
}


void SimulatorItem::removePostDynamicsFunction(int id)
{
    impl->postDynamicsFunctions.remove(id);
}


namespace {

int FunctionSet::add(std::function<void()>& func)
{
    std::lock_guard<std::mutex> lock(mutex);
    
    FunctionInfo info;
    info.function = func;
    while(true){
        if(registerdIds.insert(idCounter).second){
            break;
        }
        ++idCounter;
    }
    info.id = idCounter++;
    
    if(!simImpl->isRunning()){
        functions.push_back(info);
    } else {
        functionsToAdd.push_back(info);
        needToUpdate = true;
    }

    return info.id;
}


void FunctionSet::remove(int id)
{
    std::lock_guard<std::mutex> lock(mutex);
    idsToRemove.push_back(id);
    needToUpdate = true;
}


void FunctionSet::updateFunctions()
{
    std::lock_guard<std::mutex> lock(mutex);

    for(size_t i=0; i < functionsToAdd.size(); ++i){
        functions.push_back(functionsToAdd[i]);
    }
    functionsToAdd.clear();

    for(size_t i=0; i < idsToRemove.size(); ++i){

        const int idToRemove = idsToRemove[i];
        auto p = functions.end();
        while(p != functions.begin()){
            --p;
            FunctionInfo& info = *p;
            if(info.id == idToRemove){
                functions.erase(p);
                break;
            }
        }
    }
    needToUpdate = false;
}

}

    
void SimulatorItem::Impl::clearSimulation()
{
    allSimBodies.clear();
    simBodiesWithBody.clear();
    activeSimBodies.clear();
    loggedControllerInfos.clear();
    simBodyMap.clear();

    preDynamicsFunctions.clear();
    midDynamicsFunctions.clear();
    postDynamicsFunctions.clear();

    subSimulatorItems.clear();
    activeControllerInfos.clear();

    hasControllers = false;

    sigLogFlushRequested.disconnectAllSlots();

    self->clearSimulation();
}


void SimulatorItem::clearSimulation()
{

}


SimulationBody* SimulatorItem::createSimulationBody(Body* orgBody, CloneMap& cloneMap)
{
    return nullptr;
}


SimulationBody* SimulatorItem::createSimulationBody(Body* orgBody)
{
    return nullptr;
}


void SimulatorItem::Impl::setSimulatorItemToControllerItem(ControllerItem* controllerItem)
{
    controllerItem->setSimulatorItem(self);
}


void SimulatorItem::Impl::resetSimulatorItemForControllerItem(ControllerItem* controllerItem)
{
    controllerItem->setSimulatorItem(nullptr);
}


bool SimulatorItem::startSimulation(bool doReset)
{
    return impl->startSimulation(doReset);
}


bool SimulatorItem::Impl::startSimulation(bool doReset)
{
    // Check if there is another active simulation in the same world
    for(auto& simulatorItem : worldItem->descendantItems<SimulatorItem>()){
        if(simulatorItem->isRunning()){
            if(simulatorItem == self){
                mv->putln(format(_("{0} is doing its simulation."), self->displayName(), MessageView::Warning));
            } else {
                mv->putln(
                    format(_("{0} cannot start the simulation because {1} in the same world is doing its simulation."),
                           self->displayName(), simulatorItem->displayName()),
                    MessageView::Error);
            }
            return false;
        }
    }
    
    stopSimulation(true, true);

    if(!worldItem){
        mv->putln(format(_("{} must be in a WorldItem to do simulation."), self->displayName()),
                  MessageView::Error);
        return false;
    }

    bool initialized = initializeSimulation(doReset);

    if(!initialized){
        mv->notify(format(_("{0} failed to initialize the simulation."), self->displayName()),
                   MessageView::Error);
        clearSimulation();
    }

    return initialized;
}


/*
  Note that the following function is is not the implementation of the
  SimulatorItem::initializeSimulation function. SimulatorItem::initializeSimulation implements
  the special process for each SimulatorItem sub class and is called in the following function.
*/
bool SimulatorItem::Impl::initializeSimulation(bool doReset)
{
    ItemList<Item> targetItems;
    findTargetItems(worldItem, false, targetItems);
    if(targetItems.empty()){
        return false;
    }

    cloneMap.clear();

    currentFrame = 0;
    currentTime_ = 0.0;
    worldTimeStep_ = self->worldTimeStep();
    worldFrameRate = 1.0 / worldTimeStep_;

    if(recordingMode.is(SimulatorItem::REC_NONE)){
        isRecordingEnabled = false;
        isRingBufferMode = false;
    } else {
        isRecordingEnabled = true;
        isRingBufferMode = recordingMode.is(SimulatorItem::REC_TAIL);
    }

    ringBufferSize = std::numeric_limits<int>::max();

    if(timeRangeMode.is(SimulatorItem::TR_SPECIFIED)){
        maxFrame = std::max(0, static_cast<int>(lround(timeLength / worldTimeStep_) - 1));

    } else if(timeRangeMode.is(SimulatorItem::TR_TIMEBAR)){
        maxFrame = std::max(0, static_cast<int>(lround(timeBar->maxTime() / worldTimeStep_) - 1));

    } else if(isRingBufferMode){
        maxFrame = std::numeric_limits<int>::max();
        ringBufferSize = timeLength / worldTimeStep_;

    } else {
        maxFrame = std::numeric_limits<int>::max();
    }

    currentRealtimeSyncMode = realtimeSyncMode.which();

    clearSimulation();
    getOrCreateLogEngine()->clearSubEngines();

    isSimulationFromInitialState = doReset;
    if(isSimulationFromInitialState){
        for(auto& targetItem : targetItems){
            if(auto bodyItem = dynamic_cast<BodyItem*>(targetItem.get())){
                bodyItem->restoreInitialState(true);
            }
        }
    }

    sigSimulationAboutToBeStarted();

    for(auto& targetItem : targetItems){

        if(auto bodyItem = dynamic_cast<BodyItem*>(targetItem.get())){
            auto orgBody = bodyItem->body();
            SimulationBodyPtr simBody = self->createSimulationBody(orgBody, cloneMap);
            if(!simBody){
                // Old API
                simBody = self->createSimulationBody(orgBody);
            }

            if(!simBody){
                mv->putln(format(_("The clone of {0} for the simulation cannot be created."), orgBody->name()),
                          MessageView::Warning);
            } else {
                if(simBody->body()){
                    simBodyMap[bodyItem] = simBody;
                    
                    if(!simBody->initialize(self, bodyItem)){
                        simBodyMap.erase(bodyItem);

                    } else {
                        // copy the body state overwritten by the controller
                        simBody->impl->copyStateToBodyItem();

                        allSimBodies.push_back(simBody);
                        simBodiesWithBody.push_back(simBody);
                    }
                }
            }
            bodyItem->notifyKinematicStateChange();
            
        } else if(auto controller = dynamic_cast<ControllerItem*>(targetItem.get())){
            // ControllerItem which is not associated with a body
            SimulationBodyPtr simBody = new SimulationBody(nullptr);
            if(simBody->impl->initialize(this, controller)){
                allSimBodies.push_back(simBody);
            }
        } else if(auto script = dynamic_cast<SimulationScriptItem*>(targetItem.get())){
            SimulationBodyPtr simBody = new SimulationBody(nullptr);
            ControllerItemPtr scriptControllerItem = new ScriptControllerItem(script);
            if(simBody->impl->initialize(this, scriptControllerItem)){
                allSimBodies.push_back(simBody);
            }
        }
    }

    extForceFunctionId = stdx::nullopt;
    virtualElasticStringFunctionId = stdx::nullopt;

    cloneMap.replacePendingObjects();
    
    if(!self->initializeSimulation(simBodiesWithBody)){
        return false;
    }

    subSimulatorItems.extractAssociatedItems(self);
    auto p = subSimulatorItems.begin();
    while(p != subSimulatorItems.end()){
        SubSimulatorItem* item = *p;
        bool initialized = false;
        if(item->isEnabled()){
            mv->putln(format(_("SubSimulatorItem \"{}\" has been detected."), item->displayName()));
            if(item->initializeSimulation(self)){
                initialized = true;
            } else {
                mv->putln(format(_("The initialization of \"{}\" failed."), item->displayName()),
                          MessageView::Warning);
            }
        } else {
            mv->putln(format(_("SubSimulatorItem \"{}\" is disabled."), item->displayName()));
        }
        if(initialized){
            ++p;
        } else {
            p = subSimulatorItems.erase(p);
        }
    }

    for(size_t i=0; i < allSimBodies.size(); ++i){
        SimulationBody* simBody = allSimBodies[i];
        if(simBody->impl->isActive){
            activeSimBodies.push_back(simBody);
        }
    }

    // Initialize recording
    numBufferedFrames = 0;
    frameAtLastBufferWriting = 0;
    for(auto& simBody : activeSimBodies){
        if(simBody->body()){
            simBody->impl->initializeRecording();
        }
    }

    doRecordCollisionData = (isRecordingEnabled && isCollisionDataRecordingEnabled);
    if(doRecordCollisionData){
        collisionPairsBuf.clear();
        string collisionSeqName = self->name() + "-collisions";
        auto collisionSeqItem = worldItem->findChildItem<CollisionSeqItem>(collisionSeqName);
        if(collisionSeqItem){
            if(!collisionSeqItem->isTemporary()){
                collisionSeqItem->setTemporary();
                collisionSeqItem->notifyUpdate();
            }
        } else {
            collisionSeqItem = new CollisionSeqItem;
            collisionSeqItem->setTemporary();
            collisionSeqItem->setName(collisionSeqName);
            worldItem->addChildItem(collisionSeqItem);
            logEngine->addCollisionSeqEngine(collisionSeqItem);
        }
        collisionSeq = collisionSeqItem->collisionSeq();
        collisionSeq->setFrameRate(worldFrameRate);
        collisionSeq->setNumParts(1);
        collisionSeq->setNumFrames(1);
        CollisionSeq::Frame frame0 = collisionSeq->frame(0);
        frame0[0]  = std::make_shared<CollisionLinkPairList>();
    }

    for(auto& simBody : allSimBodies){
        auto simBodyImpl = simBody->impl;
        Body* body = simBodyImpl->body_;
        auto& controllerInfos = simBodyImpl->controllerInfos;
        auto iter = controllerInfos.begin();
        while(iter != controllerInfos.end()){
            auto& info = *iter;
            ControllerItem* controller = info->controller;
            bool ready = false;
            if(body){
                ready = controller->start();
                if(!ready){
                    mv->putln(format(_("{0} for {1} failed to start."),
                                     controller->displayName(), simBodyImpl->bodyItem->displayName()),
                              MessageView::Warning);
                }
            } else {
                ready = controller->start();
                if(!ready){
                    mv->putln(format(_("{} failed to start."), controller->displayName()),
                              MessageView::Warning);
                }
            }
            if(ready){
                activeControllerInfos.push_back(info);
                ++iter;
            } else {
                iter = controllerInfos.erase(iter);
            }
        }
    }

    doStopSimulationWhenNoActiveControllers = isActiveControlTimeRangeMode && hasControllers;

    if(doStopSimulationWhenNoActiveControllers && activeControllerInfos.empty()){
        mv->putln(_("The simulation cannot be started because all the controllers are inactive."),
                  MessageView::Error);
        return false;
    }

    if(!self->completeInitializationOfSimulation()){
        return false;
    }

    isDoingSimulationLoop = true;
    isWaitingForSimulationToStop = false;
    isForcedToStopSimulation = false;
    stopRequested = false;
    pauseRequested = false;

    useControllerThreads = useControllerThreadsProperty;
    if(useControllerThreads){
        for(auto& info : activeControllerInfos){
            info->isExitingControlLoopRequested = false;
            info->isControlRequested = false;
            info->isControlFinished = false;
            info->isControlToBeContinued = false;
            info->controlThread = std::thread([info](){ info->concurrentControlLoop(); });
        }
    }

    aboutToQuitConnection.disconnect();
    aboutToQuitConnection = App::sigAboutToQuit().connect(
        [&](){ stopSimulation(true, true); });

    worldLogFileItem = nullptr;
    // Check child items first
    auto worldLogFileItems = self->descendantItems<WorldLogFileItem>();
    if(worldLogFileItems.empty()){
        // Check items in the world secondly
        worldLogFileItems = worldItem->descendantItems<WorldLogFileItem>();
    }
    worldLogFileItem = worldLogFileItems.toSingle(true);
    if(worldLogFileItem){
        if(worldLogFileItem->logFile().empty()){
            worldLogFileItem = nullptr;
        } else {
            mv->putln(format(_("WorldLogFileItem \"{0}\" has been detected. "
                               "A simulation result is recoreded to \"{1}\"."),
                             worldLogFileItem->displayName(), worldLogFileItem->logFile()));

            worldLogFileItem->clearOutput();
            worldLogFileItem->beginHeaderOutput();
            for(size_t i=0; i < activeSimBodies.size(); ++i){
                worldLogFileItem->outputBodyHeader(activeSimBodies[i]->impl->body_->name());
            }
            worldLogFileItem->endHeaderOutput();
            worldLogFileItem->notifyUpdate();
            nextLogFrame = 0;
            nextLogTime = 0.0;
            double r = worldLogFileItem->recordingFrameRate();
            if(r == 0.0){
                r = worldFrameRate;
            }
            logTimeStep = 1.0 / r;
        }
    }

    logEngine->startOngoingTimeUpdate(0.0);
    flushRecords();
    start();
    startFlushTimer();

    mv->notify(format(_("Simulation by {} has started."), self->displayName()));

    sigSimulationStarted();
    
    // For blocking manual user operations for modifying body kinematic state using the builtin GUIs
    for(auto& simBody : simBodiesWithBody){
        auto simpl = simBody->impl;
        simpl->continuousKinematicUpdateEntry = simpl->bodyItem->startContinuousKinematicUpdate();
    }
    if(isSceneViewEditModeBlockedDuringSimulation){
        SceneView::blockEditModeForAllViews(self);
    }

    return true;
}


CloneMap& SimulatorItem::cloneMap()
{
    return impl->cloneMap;
}


bool SimulatorItem::completeInitializationOfSimulation()
{
    return true;
}


void SimulatorItem::initializeSimulationThread()
{

}


void SimulatorItem::finalizeSimulationThread()
{

}


const std::vector<SimulationBody*>& SimulatorItem::simulationBodies()
{
    return impl->simBodiesWithBody;
}


/**
   \todo make thread safe
*/
SimulationBody* SimulatorItem::findSimulationBody(BodyItem* bodyItem)
{
    SimulationBody* simBody = nullptr;
    auto p = impl->simBodyMap.find(bodyItem);
    if(p != impl->simBodyMap.end()){
        simBody = p->second;
    }
    return simBody;
}


/**
   \todo make thread safe
*/
SimulationBody* SimulatorItem::findSimulationBody(const std::string& name)
{
    const int n = impl->allSimBodies.size();
    for(int i=0; i < n; ++i){
        SimulationBody* simBody = impl->allSimBodies[i];
        Body* body = simBody->body();
        if(body && body->name() == name){
            return simBody;
        }
    }
    return nullptr;
}


// Simulation loop
void SimulatorItem::Impl::run()
{
    self->initializeSimulationThread();

    double elapsedTime = 0.0;
    QElapsedTimer timer;
    timer.start();

    int frame = 0;
    bool isOnPause = false;

    if(currentRealtimeSyncMode == NonRealtimeSync){
        while(true){
            if(pauseRequested){
                if(stopRequested){
                    break;
                }
                if(!isOnPause){
                    elapsedTime += timer.elapsed();
                    isOnPause = true;
                    sigSimulationPaused();
                }
                QThread::msleep(50);
            } else {
                if(isOnPause){
                    timer.start();
                    isOnPause = false;
                    sigSimulationResumed();
                }
                if(!stepSimulationMain() || stopRequested || frame++ >= maxFrame){
                    break;
                }
            }
        }
    } else {
        const double dt = worldTimeStep_;
        const double compensationRatio = (dt > 0.1) ? 0.1 : dt;
        const double dtms = dt * 1000.0;
        double compensatedSimulationTime = 0.0;
        while(true){
            if(pauseRequested){
                if(stopRequested){
                    break;
                }
                if(!isOnPause){
                    elapsedTime += timer.elapsed();
                    isOnPause = true;
                    sigSimulationPaused();
                }
                QThread::msleep(50);
            } else {
                if(isOnPause){
                    timer.start();
                    isOnPause = false;
                    sigSimulationResumed();
                }
                if(!stepSimulationMain() || stopRequested || frame >= maxFrame){
                    break;
                }
                double diff = (double)compensatedSimulationTime - (elapsedTime + timer.elapsed());
                if(currentRealtimeSyncMode == ConservativeRealtimeSync){
                    if(diff >= 0.0){
                        QThread::msleep(diff);
                    } else if(diff < 0.0){
                        compensatedSimulationTime += -diff;
                    }
                } else {
                    if(diff >= 1.0){
                        QThread::msleep(diff);
                    } else if(diff < 0.0){
                        const double compensationTime = -diff * compensationRatio;
                        compensatedSimulationTime += compensationTime;
                        diff += compensationTime;
                        const double delayOverThresh = -diff - 100.0;
                        if(delayOverThresh > 0.0){
                            compensatedSimulationTime += delayOverThresh;
                        }
                    }
                }
                compensatedSimulationTime += dtms;
                ++frame;
            }
        }
    }

    if(!isOnPause){
    	elapsedTime += timer.elapsed();
    }
    actualSimulationTime = (elapsedTime / 1000.0);
    finishTime = frame / worldFrameRate;

    isDoingSimulationLoop = false;

    if(useControllerThreads){
        for(auto& info : activeControllerInfos){
            {
                std::lock_guard<std::mutex> lock(info->controlMutex);
                info->isExitingControlLoopRequested = true;
            }
            info->controlCondition.notify_all();
            info->controlThread.join();
        }
    }

    if(!isWaitingForSimulationToStop){
        callLater([&](){ onSimulationLoopStopped(isForcedToStopSimulation); });
    }

    self->finalizeSimulationThread();
}


bool SimulatorItem::Impl::stepSimulationMain()
{
    // Recored the positions at the beginning of the current frame
    bufferRecords();

    bool doContinue = !doStopSimulationWhenNoActiveControllers;

    preDynamicsFunctions.call();

    if(!useControllerThreads){
        for(auto& info : activeControllerInfos){
            auto& controller = info->controller;
            controller->input();
            doContinue |= controller->control();
            if(controller->isNoDelayMode()){
                controller->output();
            }
        }
    } else {
        bool hasNoDelayModeControllers = false;
        for(auto& info : activeControllerInfos){
            auto& controller = info->controller;
            if(controller->isNoDelayMode()){
                hasNoDelayModeControllers = true;
            }
            info->controller->input();
            {
                std::lock_guard<std::mutex> lock(info->controlMutex);                
                info->isControlRequested = true;
            }
            info->controlCondition.notify_all();
        }
        if(hasNoDelayModeControllers){
            // Todo: Process the controller that finishes control earlier first to
            // reduce the total elapsed time before finishing all the output functions.
            for(auto& info : activeControllerInfos){
                if(info->controller->isNoDelayMode()){
                    if(info->waitForControlInThreadToFinish()){
                        doContinue = true;
                    }
                    info->controller->output();
                }
            }
        }
    }

    midDynamicsFunctions.call();

    self->stepSimulation(activeSimBodies);

    if(doRecordCollisionData){
        bufferCollisionRecords();
    }
    
    if(useControllerThreads){
        for(auto& info : activeControllerInfos){
            if(!info->controller->isNoDelayMode()){
                if(info->waitForControlInThreadToFinish()){
                    doContinue = true;
                }
            }
        }
    }

    postDynamicsFunctions.call();

    for(auto& info : activeControllerInfos){
        if(!info->controller->isNoDelayMode()){
            info->controller->output();
        }
    }

    ++currentFrame;
    currentTime_ = currentFrame / worldFrameRate;

    return doContinue;
}


bool ControllerInfo::waitForControlInThreadToFinish()
{
    std::unique_lock<std::mutex> lock(controlMutex);
    while(!isControlFinished){
        controlCondition.wait(lock);
    }
    isControlFinished = false;
    return isControlToBeContinued;
}


void ControllerInfo::concurrentControlLoop()
{
    while(true){
        {
            std::unique_lock<std::mutex> lock(controlMutex);
            while(true){
                if(isExitingControlLoopRequested){
                    goto exitConcurrentControlLoop;
                }
                if(isControlRequested){
                    isControlRequested = false;
                    isControlToBeContinued = false;
                    break;
                }
                controlCondition.wait(lock);
            }
        }

        bool doContinue = controller->control();
        
        {
            std::lock_guard<std::mutex> lock(controlMutex);
            isControlFinished = true;
            isControlToBeContinued = doContinue;
        }
        controlCondition.notify_all();
    }
    
exitConcurrentControlLoop:
    return;
}


void SimulatorItem::Impl::bufferRecords()
{
    recordBufMutex.lock();

    for(size_t i=0; i < activeSimBodies.size(); ++i){
        activeSimBodies[i]->bufferRecords();
    }
    ++numBufferedFrames;
    frameAtLastBufferWriting = currentFrame;

    recordBufMutex.unlock();
}


void SimulatorItem::Impl::bufferCollisionRecords()
{
    recordBufMutex.lock();
    collisionPairsBuf.push_back(self->getCollisions());
    recordBufMutex.unlock();
}


void SimulatorItem::Impl::startFlushTimer()
{
    if(timeBar->isIdleEventDrivenMode()){
        flushTimer.start(0);
    } else {
        flushTimer.start(1000.0 / timeBar->playbackFrameRate());
    }
}


void SimulatorItem::Impl::flushRecords()
{
    int frame = flushMainRecords();

    sigLogFlushRequested();

    for(auto& info : loggedControllerInfos){
        info->flushLog();
    }

    if(!isRecordingEnabled){
        const double time = frame / worldFrameRate;
        for(auto& simBody : activeSimBodies){
            simBody->impl->updateFrontendBodyStatelWithLastRecords(time);
        }
    }

    logEngine->updateOngoingTime(frame / worldFrameRate);
}


int SimulatorItem::Impl::flushMainRecords()
{
    recordBufMutex.lock();

    if(worldLogFileItem){
        if(numBufferedFrames > 0){
            int firstFrame = frameAtLastBufferWriting - (numBufferedFrames - 1);
            for(int bufFrame = 0; bufFrame < numBufferedFrames; ++bufFrame){
                double time = (firstFrame + bufFrame) * worldTimeStep_;
                while(time >= nextLogTime){
                    worldLogFileItem->beginFrameOutput(time);
                    for(size_t i=0; i < activeSimBodies.size(); ++i){
                        activeSimBodies[i]->impl->flushRecordsToWorldLogFile(bufFrame);
                    }
                    worldLogFileItem->endFrameOutput();
                    nextLogTime = ++nextLogFrame * logTimeStep;
                }
            }
        }
    }
    
    for(auto& simBody : activeSimBodies){
        simBody->flushRecords();
    }

    bool offsetChanged;
    if(doRecordCollisionData){
        offsetChanged = false;
        for(size_t i=0 ; i < collisionPairsBuf.size(); ++i){
            if(collisionSeq->numFrames() >= ringBufferSize){
                collisionSeq->popFrontFrame();
                offsetChanged = true;
            }
            CollisionSeq::Frame collisionSeq0 = collisionSeq->appendFrame();
            collisionSeq0[0] = collisionPairsBuf[i];
        }
        if(offsetChanged){
            collisionSeq->setOffsetTimeFrame(currentFrame + 1 - collisionSeq->numFrames());
        }
    }
    collisionPairsBuf.clear();

    int frame = frameAtLastBufferWriting;
    
    numBufferedFrames = 0;
    
    recordBufMutex.unlock();

    return frame;
}


void SimulatorItem::pauseSimulation()
{
    impl->pauseSimulation();
}


void SimulatorItem::Impl::pauseSimulation()
{
    flushTimer.stop();
    pauseRequested = true;
    flushRecords();
    logEngine->stopOngoingTimeUpdate();
}


void SimulatorItem::restartSimulation()
{
    impl->restartSimulation();
}


void SimulatorItem::Impl::restartSimulation()
{
    if(pauseRequested){
        logEngine->startOngoingTimeUpdate();
        pauseRequested = false;
        startFlushTimer();
    }
}


void SimulatorItem::stopSimulation(bool isForced)
{
    impl->stopSimulation(isForced, true);
}


void SimulatorItem::Impl::stopSimulation(bool isForced, bool doSync)
{
    if(isDoingSimulationLoop){
        if(doSync){
            isWaitingForSimulationToStop = true;
        }
        isForcedToStopSimulation = isForced;
        stopRequested = true;
        
        if(doSync){
            wait();
            isWaitingForSimulationToStop = false;
            onSimulationLoopStopped(isForced);
        }
    }
    aboutToQuitConnection.disconnect();
}


void SimulatorItem::finalizeSimulation()
{

}


void SimulatorItem::Impl::onSimulationLoopStopped(bool isForced)
{
    flushTimer.stop();
    
    for(auto& simBody : allSimBodies){
        for(auto& info : simBody->impl->controllerInfos){
            auto& controller = info->controller;
            controller->stop();
        }
    }
    self->finalizeSimulation();

    for(auto& subSimulator : subSimulatorItems){
        subSimulator->finalizeSimulation();
    }

    // Record the final state after processing the last frame
    bufferRecords();
    if(doRecordCollisionData){
        bufferCollisionRecords();
    }

    flushRecords();
    logEngine->stopOngoingTimeUpdate();

    mv->notify(format(_("Simulation by {0} has finished at {1} [s]."), self->displayName(), finishTime));

    if(finishTime > 0.0){
        mv->putln(format(_("Computation time is {0} [s], computation time / simulation time = {1}."),
                         actualSimulationTime, (actualSimulationTime / finishTime)));
    }

    clearSimulation();

    SceneView::unblockEditModeForAllViews(self);

    sigSimulationFinished(isForced);
}


bool SimulatorItem::isRunning() const
{
    return impl->isDoingSimulationLoop;
}


bool SimulatorItem::isPausing() const
{
    return impl->pauseRequested;
}


bool SimulatorItem::isActive() const
{
    return impl->isActive();
}


bool SimulatorItem::Impl::isActive() const
{
    return isDoingSimulationLoop && !pauseRequested;
}


int SimulatorItem::currentFrame() const
{
    return impl->currentFrame;
}


double SimulatorItem::currentTime() const
{
    return impl->currentTime_;
}


int SimulatorItem::simulationFrame() const
{
    QMutexLocker locker(&impl->recordBufMutex);
    return impl->frameAtLastBufferWriting;
}


double SimulatorItem::simulationTime() const
{
    QMutexLocker locker(&impl->recordBufMutex);
    return impl->frameAtLastBufferWriting / impl->worldFrameRate;
}


SignalProxy<void()> SimulatorItem::sigSimulationAboutToBeStarted()
{
    return impl->sigSimulationAboutToBeStarted;
}


SignalProxy<void()> SimulatorItem::sigSimulationStarted()
{
    return impl->sigSimulationStarted;
}


SignalProxy<void()> SimulatorItem::sigSimulationPaused()
{
    return impl->sigSimulationPaused;
}


SignalProxy<void()> SimulatorItem::sigSimulationResumed()
{
    return impl->sigSimulationResumed;
}


SignalProxy<void(bool isForced)> SimulatorItem::sigSimulationFinished()
{
    return impl->sigSimulationFinished;
}


Vector3 SimulatorItem::getGravity() const
{
    return Vector3::Zero();
}


void SimulatorItem::setExternalForce(BodyItem* bodyItem, Link* link, const Vector3& point, const Vector3& f, double time)
{
    impl->setExternalForce(bodyItem, link, point, f, time);
}


void SimulatorItem::Impl::setExternalForce(BodyItem* bodyItem, Link* link, const Vector3& point, const Vector3& f, double time)
{
    if(bodyItem && link){
        SimulationBody* simBody = self->findSimulationBody(bodyItem);
        if(simBody){
            {
                std::lock_guard<std::mutex> lock(extForceMutex);
                extForceInfo.link = simBody->body()->link(link->index());
                extForceInfo.point = point;
                extForceInfo.f = f;
                extForceInfo.time = time;
            }
            if(!extForceFunctionId){
                extForceFunctionId =
                    self->addPreDynamicsFunction(
                        [&](){ doSetExternalForce(); });
            }
        }
    }
}


void SimulatorItem::clearExternalForces()
{
    if(impl->extForceFunctionId){
        removePreDynamicsFunction(*impl->extForceFunctionId);
        impl->extForceFunctionId = stdx::nullopt;
    }
}    


void SimulatorItem::Impl::doSetExternalForce()
{
    std::lock_guard<std::mutex> lock(extForceMutex);
    extForceInfo.link->addExternalForceAtLocalPosition(extForceInfo.f, extForceInfo.point);
    if(extForceInfo.time > 0.0){
        extForceInfo.time -= worldTimeStep_;
        if(extForceInfo.time <= 0.0){
            self->clearExternalForces();
        }
    }
}


void SimulatorItem::setVirtualElasticString
(BodyItem* bodyItem, Link* link, const Vector3& attachmentPoint, const Vector3& endPoint)
{
    impl->setVirtualElasticString(bodyItem, link, attachmentPoint, endPoint);
}


void SimulatorItem::Impl::setVirtualElasticString
(BodyItem* bodyItem, Link* link, const Vector3& attachmentPoint, const Vector3& endPoint)
{
    if(bodyItem && link){
        SimulationBody* simBody = self->findSimulationBody(bodyItem);
        if(simBody){
            {
                std::lock_guard<std::mutex> lock(virtualElasticStringMutex);
                Body* body = simBody->body();
                VirtualElasticString& s = virtualElasticString;
                s.link = body->link(link->index());
                double m = body->mass();
                s.kp = 3.0 * m;
                s.kd = 0.1 * s.kp;
                s.f_max = s.kp;
                s.point = attachmentPoint;
                s.goal = endPoint;
            }
            if(!virtualElasticStringFunctionId){
                virtualElasticStringFunctionId =
                    self->addPreDynamicsFunction(
                        [&](){ setVirtualElasticStringForce(); });
            }
        }
    }
}


void SimulatorItem::clearVirtualElasticStrings()
{
    if(impl->virtualElasticStringFunctionId){
        removePreDynamicsFunction(*impl->virtualElasticStringFunctionId);
        impl->virtualElasticStringFunctionId = stdx::nullopt;
    }
}


void SimulatorItem::Impl::setVirtualElasticStringForce()
{
    std::lock_guard<std::mutex> lock(virtualElasticStringMutex);
    const VirtualElasticString& s = virtualElasticString;
    Link* link = s.link;
    Vector3 a = link->R() * s.point;
    Vector3 p = link->p() + a;
    Vector3 v = link->v() + link->w().cross(a);
    Vector3 f = s.kp * (s.goal - p) + s.kd * (-v);
    if(f.norm() > s.f_max){
        f = s.f_max * f.normalized();
    }
    link->f_ext() += f;
    link->tau_ext() += p.cross(f);
}


void SimulatorItem::setForcedPosition(BodyItem* /* bodyItem */, const Isometry3& /* T */)
{

}


bool SimulatorItem::isForcedPositionActiveFor(BodyItem* /* bodyItem */) const
{
    return false;
}


void SimulatorItem::clearForcedPositions()
{

}


/**
   This function may be overridden.
*/
bool SimulatorItem::Impl::onAllLinkPositionOutputModeChanged(bool on)
{
    self->setAllLinkPositionOutputMode(on);
    return (isAllLinkPositionOutputMode == on);
}


std::shared_ptr<CollisionLinkPairList> SimulatorItem::getCollisions()
{
    return std::make_shared<CollisionLinkPairList>();
}


void SimulatorItem::setSceneViewEditModeBlockedDuringSimulation(bool on)
{
    impl->isSceneViewEditModeBlockedDuringSimulation = on;
}


void SimulatorItem::doPutProperties(PutPropertyFunction& putProperty)
{
    impl->doPutProperties(putProperty);
}


void SimulatorItem::Impl::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Temporal resolution type"), temporalResolutionType,
                [&](int index){ return temporalResolutionType.select(index); });

    if(temporalResolutionType.is(RESOLUTION_TIMESTEP)){
        putProperty(_("Time step"), timeStepProperty,
                    [&](const std::string& s){ return timeStepProperty.setPositiveValue(s); });
    } else if(temporalResolutionType.is(RESOLUTION_FRAMERATE)){
        putProperty.min(1)(_("Frame rate"), frameRateProperty, changeProperty(frameRateProperty));
        putProperty.reset();
    }

    putProperty(_("Realtime sync"), realtimeSyncMode,
                [&](int index){ return realtimeSyncMode.select(index); });
    putProperty(_("Time range"), timeRangeMode,
                [&](int index){ return timeRangeMode.select(index); });
    putProperty.min(0.0);
    putProperty(_("Time length"), timeLength,
                 [&](double length){ self->setTimeLength(length); return true; });
    putProperty(_("Active control period only"), isActiveControlTimeRangeMode,
                [&](bool on){ self->setActiveControlTimeRangeMode(on); return true; });
    putProperty(_("Recording"), recordingMode,
                [&](int index){ return recordingMode.select(index); });
    putProperty(_("All link position recording"), isAllLinkPositionOutputMode,
                [&](bool on){ return onAllLinkPositionOutputModeChanged(on); });
    putProperty(_("Device state output"), isDeviceStateOutputEnabled,
                changeProperty(isDeviceStateOutputEnabled));
    putProperty(_("Record collision data"), isCollisionDataRecordingEnabled,
                changeProperty(isCollisionDataRecordingEnabled));
    putProperty(_("Controller Threads"), useControllerThreadsProperty,
                changeProperty(useControllerThreadsProperty));
    putProperty(_("Controller options"), controllerOptionString_,
                changeProperty(controllerOptionString_));
    putProperty(_("Block scene view edit mode"), isSceneViewEditModeBlockedDuringSimulation,
                [&](bool on){ self->setSceneViewEditModeBlockedDuringSimulation(on); return true; });
}


bool SimulatorItem::store(Archive& archive)
{
    return impl->store(archive);

}


bool SimulatorItem::Impl::store(Archive& archive)
{
    if(temporalResolutionType.is(RESOLUTION_TIMESTEP)){
        archive.write("time_step", timeStepProperty.string());
    } else if(temporalResolutionType.is(RESOLUTION_FRAMERATE)){
        archive.write("frame_rate", frameRateProperty);
    }
    archive.write("realtime_sync_mode", realtimeSyncModeSymbols[realtimeSyncMode.which()]);
    archive.write("recording", recordingMode.selectedSymbol());
    archive.write("time_range_mode", timeRangeModeSymbols[timeRangeMode.which()]);
    archive.write("time_length", timeLength);
    archive.write("is_active_control_time_range_mode", isActiveControlTimeRangeMode);
    archive.write("output_all_link_positions", isAllLinkPositionOutputMode);
    archive.write("output_device_states", isDeviceStateOutputEnabled);
    archive.write("use_controller_threads", useControllerThreadsProperty);
    archive.write("record_collision_data", isCollisionDataRecordingEnabled);
    archive.write("controller_options", controllerOptionString_, DOUBLE_QUOTED);
    archive.write("block_scene_view_edit_mode", isSceneViewEditModeBlockedDuringSimulation);
    
    ListingPtr idseq = new Listing;
    idseq->setFlowStyle(true);
    for(auto& engine : getOrCreateLogEngine()->subEngines){
        if(auto id = archive.getItemIdNode(engine->item())){
            idseq->append(id);
        }
    }
    if(!idseq->empty()){
        archive.insert("time_sync_items", idseq);
    }
    if(auto engine = logEngine->collisionSeqEngine){
        if(auto id = archive.getItemIdNode(engine->collisionSeqItem())){
            archive.insert("collision_seq_item", id);
        }
    }
    
    return true;
}


bool SimulatorItem::restore(const Archive& archive)
{
    return impl->restore(archive);
}


bool SimulatorItem::Impl::restore(const Archive& archive)
{
    bool boolValue;
    string symbol;

    if(archive.read({ "time_step", "timeStep", "timestep" }, symbol)){
        timeStepProperty = symbol;
        temporalResolutionType.select(RESOLUTION_TIMESTEP);
    } else if(archive.read({ "frame_rate", "frameRate", "framerate" }, frameRateProperty)){
        temporalResolutionType.select(RESOLUTION_FRAMERATE);
    } else {
        temporalResolutionType.select(RESOLUTION_TIMEBAR);
    }

    if(archive.read("time_range_mode", symbol)){
        for(int i=0; i < NumTimeRangeModes; ++i){
            if(symbol == timeRangeModeSymbols[i]){
                self->setTimeRangeMode(i);
            }
        }
    } else if(archive.read("timeRangeMode", symbol)){ // Old format
        if(timeRangeMode.select(symbol)){
            self->setTimeRangeMode(timeRangeMode.which());
        } else if(symbol == "Active control period"){
            self->setTimeRangeMode(ActiveControlTime);
        }
    }
    isActiveControlTimeRangeMode =
        archive.get({ "is_active_control_time_range_mode", "active_control_time_range_mode" },
                    isActiveControlTimeRangeMode);

    if(archive.read("recording", symbol)){
        recordingMode.select(symbol);
    } else if(archive.read("recording", boolValue) && boolValue){ // Old format
        recordingMode.select(SimulatorItem::REC_FULL);
    } else if(archive.read("recordingMode", symbol)){ // Old format
        if(symbol == "Direct"){
            recordingMode.select(SimulatorItem::REC_NONE);
            timeRangeMode.select(SimulatorItem::TR_UNLIMITED);
        }
    }
    if(archive.read("realtime_sync_mode", symbol)){
        for(int i=0; i < NumRealtimeSyncModes; ++i){
            if(symbol == realtimeSyncModeSymbols[i]){
                realtimeSyncMode.select(i);
            }
        }
    } else { // Old format
        bool on = archive.get("realtimeSync", true);
        realtimeSyncMode.select(on ? CompensatoryRealtimeSync : NonRealtimeSync);
    }

    archive.read({ "time_length", "timeLength" }, timeLength);

    bool on = archive.get({ "output_all_link_positions", "allLinkPositionOutputMode" }, isAllLinkPositionOutputMode);
    self->setAllLinkPositionOutputMode(on);
    
    archive.read({ "output_device_states", "deviceStateOutput" }, isDeviceStateOutputEnabled);
    archive.read({ "record_collision_data", "recordCollisionData" }, isCollisionDataRecordingEnabled);
    archive.read({ "use_controller_threads", "controllerThreads" }, useControllerThreadsProperty);
    archive.read({ "controller_options", "controllerOptions" }, controllerOptionString_);
    archive.read({ "block_scene_view_edit_mode", "scene_view_edit_mode_blocking" },
                 isSceneViewEditModeBlockedDuringSimulation);

    archive.addPostProcess([&](){ restoreTimeSyncItemEngines(archive); });
    
    return true;
}


void SimulatorItem::Impl::restoreTimeSyncItemEngines(const Archive& archive)
{
    getOrCreateLogEngine()->clearSubEngines();

    const Listing& idseq = *archive.findListing("time_sync_items");
    if(idseq.isValid()){
        for(int i=0; i < idseq.size(); ++i){
            ValueNode* id = idseq.at(i);
            if(id){
                if(auto item = archive.findItem(id)){
                    logEngine->addSubEnginesFor(item);
                }
            }
        }
    }

    ValueNode* id;
    id = archive.find("collisionSeqItem");
    if(id->isValid()){
        auto collisionSeqItem = dynamic_cast<CollisionSeqItem*>(archive.findItem(id));
        if(collisionSeqItem){
            logEngine->addCollisionSeqEngine(collisionSeqItem);
        }
    }
}


SimulationLogEngine* SimulatorItem::Impl::getOrCreateLogEngine()
{
    if(!logEngine){
        logEngine = new SimulationLogEngine(this);
    }
    return logEngine;
}


SimulationLogEngine::SimulationLogEngine(SimulatorItem::Impl* itemImpl)
    : TimeSyncItemEngine(itemImpl->self),
      itemImpl(itemImpl)
{
    manager = TimeSyncItemEngineManager::instance();
    doKeepPlayback = false;
}


void SimulationLogEngine::clearSubEngines()
{
    subEngines.clear();
    collisionSeqEngine.reset();
}


void SimulationLogEngine::addSubEnginesFor(Item* item)
{
    manager->createEngines(item, subEngines);
}


void SimulationLogEngine::addCollisionSeqEngine(CollisionSeqItem* collisionSeqItem)
{
    if(itemImpl->worldItem){
        collisionSeqEngine = new CollisionSeqEngine(itemImpl->worldItem, collisionSeqItem);
    }
}


void SimulationLogEngine::onPlaybackStarted(double /* time */)
{
    doKeepPlayback = true;
    setupBodyItems(true);
}


bool SimulationLogEngine::isTimeSyncAlwaysMaintained() const
{
    return doKeepPlayback;
}


bool SimulationLogEngine::onTimeChanged(double time)
{
    bool isActive = itemImpl->isActive();

    if(itemImpl->isRecordingEnabled){
        if(!subEngines.empty()){
            for(auto& engine : subEngines){
                isActive |= engine->onTimeChanged(time);
            }
        } else if(itemImpl->worldLogFileItem){
            isActive |= itemImpl->worldLogFileItem->recallStateAtTime(time);
        }
        if(collisionSeqEngine){
            isActive |= collisionSeqEngine->onTimeChanged(time);
        }
    }
    return isActive;
}


double SimulationLogEngine::onPlaybackStopped(double time, bool /* isStoppedManually */)
{
    doKeepPlayback = false;
    setupBodyItems(false);

    double simulationTime = itemImpl->currentTime_;
    return simulationTime < time ? simulationTime : time;
}


void SimulationLogEngine::setupBodyItems(bool doStartPlayback)
{
    bodyItemEntries.clear();

    if(itemImpl->worldItem){
        for(auto& bodyItem : itemImpl->worldItem->descendantItems<BodyItem>()){
            if(!bodyItem->body()->isStaticModel()){
                bodyItem->notifyKinematicStateUpdate(false);
            }
            if(doStartPlayback){
                bodyItemEntries.push_back(bodyItem->startContinuousKinematicUpdate());
            }
        }
    }
}
