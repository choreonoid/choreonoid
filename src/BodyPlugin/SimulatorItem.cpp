/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "SimulatorItem.h"
#include "WorldItem.h"
#include "ControllerItem.h"
#include "SubSimulatorItem.h"
#include "SimulationScriptItem.h"
#include "BodyMotionItem.h"
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
#include <cnoid/AppUtil>
#include <cnoid/TimeBar>
#include <cnoid/MessageView>
#include <cnoid/LazyCaller>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Archive>
#include <cnoid/MultiDeviceStateSeq>
#include <cnoid/ControllerLogItem>
#include <cnoid/Timer>
#include <cnoid/Deque2D>
#include <cnoid/ConnectionSet>
#include <cnoid/FloatingNumberString>
#include <cnoid/SceneGraph>
#include <cnoid/CloneMap>
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

typedef Deque2D<SE3, Eigen::aligned_allocator<SE3> > MultiSE3Deque;

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
    SimulatorItem::Impl* simImpl;
    Body* body_;

    std::thread controlThread;
    std::condition_variable controlCondition;
    std::mutex controlMutex;
    bool isExitingControlLoopRequested;
    bool isControlRequested;
    bool isControlFinished;
    bool isControlToBeContinued;

    std::mutex logMutex;
    ReferencedPtr lastLogData;
    unique_ptr<ReferencedObjectSeq> logBuf;
    int logBufFrameOffset;
    ControllerLogItemPtr logItem;
    shared_ptr<ReferencedObjectSeq> log;
    bool isLogEnabled_;

    ControllerInfo(ControllerItem* controller, SimulationBody::Impl* simBodyImpl);

    virtual std::string controllerName() const override;
    virtual Body* body() override;
    std::ostream& os() const override;
    virtual double timeStep() const override;
    virtual double currentTime() const override;
    virtual std::string optionString() const override;

    virtual bool enableLog() override;
    bool isLogEnabled() const;
    virtual void outputLog(Referenced* frameLog) override;
    void flushLog();

    virtual bool isNoDelayMode() const override;
    virtual bool setNoDelayMode(bool on) override;

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
    bool doKeepPlayback;

    SimulationLogEngine(SimulatorItem::Impl* itemImpl);
    void clearSubEngines();
    void addSubEnginesFor(Item* item);
    void addCollisionSeqEngine(CollisionSeqItem* collisionSeqItem);
    virtual void onPlaybackStarted(double /* time */) override;
    virtual bool onTimeChanged(double time) override;
    virtual void onPlaybackStopped(double time, bool isStoppedManually) override;
    virtual bool isTimeSyncAlwaysMaintained() const override;
    void notifyKinematicStateUpdate();
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

    vector<ControllerInfoPtr> controllerInfos;
    SimulatorItem::Impl* simImpl;

    bool isActive;
    bool isDynamic;
    bool areShapesCloned;

    Deque2D<double> jointPosBuf;
    MultiSE3Deque linkPosBuf;
    vector<Device*> devicesToNotifyRecords;
    ScopedConnectionSet deviceStateConnections;
    vector<bool> deviceStateChangeFlag;
    Deque2D<DeviceStatePtr> deviceStateBuf;

    ItemPtr parentOfRecordItems;
    string recordItemPrefix;
    shared_ptr<BodyMotion> motion;
    shared_ptr<MultiValueSeq> jointPosRecord;
    shared_ptr<MultiSE3Seq> linkPosRecord;
    MultiSE3SeqItemPtr linkPosRecordItem;
    vector<DeviceStatePtr> prevFlushedDeviceStateInDirectMode;
    shared_ptr<MultiDeviceStateSeq> deviceStateRecord;

    Impl(SimulationBody* self, Body* body);
    void findControlSrcItems(Item* item, vector<Item*>& io_items);
    bool initialize(SimulatorItem* simulatorItem, BodyItem* bodyItem);
    bool initialize(SimulatorItem::Impl* simImpl, ControllerItem* controllerItem);
    void extractAssociatedItems(bool doReset);
    void copyStateToBodyItem();
    void cloneShapesOnce();
    void initializeRecording();
    void initializeRecordBuffers();
    void initializeRecordItems();
    void setInitialStateOfBodyMotion(shared_ptr<BodyMotion> bodyMotion);
    void setActive(bool on);
    void bufferRecords();
    void flushRecords();
    void flushRecordsToBodyMotionItems();
    void flushRecordsToBody();
    void flushRecordsToWorldLogFile(int bufferFrame);
    void notifyRecords(double time);
};


class SimulatorItem::Impl : public QThread, public ControllerIO
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
    double worldFrameRate;
    double worldTimeStep_;
    int frameAtLastBufferWriting;
    int numBufferedFrames;
    Timer flushTimer;

    FunctionSet preDynamicsFunctions;
    FunctionSet midDynamicsFunctions;
    FunctionSet postDynamicsFunctions;
    
    vector<SimulationBody::Impl*> simBodyImplsToNotifyRecords;
    ItemList<SubSimulatorItem> subSimulatorItems;

    vector<ControllerInfoPtr> activeControllerInfos;
    vector<ControllerItemPtr> loggingControllers;
    bool doStopSimulationWhenNoActiveControllers;
    bool hasControllers; // Includes non-active controllers

    CollisionDetectorPtr collisionDetector;

    shared_ptr<CollisionSeq> collisionSeq;
    deque<shared_ptr<CollisionLinkPairList>> collisionPairsBuf;

    Selection recordingMode;
    Selection timeRangeMode;
    double timeLength;
    int maxFrame;
    int ringBufferSize;
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
    bool isRealtimeSyncMode;
    bool needToUpdateSimBodyLists;
    bool hasActiveFreeBodies;
    bool recordCollisionData;

    string controllerOptionString_;

    TimeBar* timeBar;
    QMutex recordBufMutex;
    double actualSimulationTime;
    double finishTime;
    MessageView* mv;

    bool doReset;
    bool isWaitingForSimulationToStop;
    bool isForcedToStopSimulation;
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
    bool startSimulation(bool doReset);
    virtual void run() override;
    void onSimulationLoopStarted();
    void updateSimBodyLists();
    bool stepSimulationMain();
    void flushRecords();
    int flushMainRecords();
    void stopSimulation(bool isForced, bool doSync);
    void pauseSimulation();
    void restartSimulation();
    void onSimulationLoopStopped(bool isForced);
    void setExternalForce(BodyItem* bodyItem, Link* link, const Vector3& point, const Vector3& f, double time);
    void doSetExternalForce();
    void setVirtualElasticString(
        BodyItem* bodyItem, Link* link, const Vector3& attachmentPoint, const Vector3& endPoint);
    void setVirtualElasticStringForce();
    void onRealtimeSyncChanged(bool on);
    bool onAllLinkPositionOutputModeChanged(bool on);
    void doPutProperties(PutPropertyFunction& putProperty);
    bool store(Archive& archive);
    bool restore(const Archive& archive);
    void restoreTimeSyncItemEngines(const Archive& archive);
    SimulationLogEngine* getOrCreateLogEngine();

    // Functions defined in the ControllerIO class
    virtual std::string controllerName() const override;
    virtual Body* body() override;
    virtual std::string optionString() const override;
    virtual std::ostream& os() const override;
    virtual double timeStep() const override;
    virtual double currentTime() const override;
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

    ItemTreeView::instance()->customizeContextMenu<SimulatorItem>(
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
            worldItem->traverse<SimulatorItem>(
                [&](SimulatorItem* item){
                    if(item->isActive()){
                        activeSimulatorItem = item;
                        return true;
                    }
                    return false;
                });
        }
    }
    return activeSimulatorItem;
}


namespace {

ControllerInfo::ControllerInfo(ControllerItem* controller, SimulationBody::Impl* simBodyImpl)
    : controller(controller),
      body_(simBodyImpl->body_),
      simImpl(simBodyImpl->simImpl),
      isLogEnabled_(false)
{

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
    return simImpl->currentTime();
}


std::string ControllerInfo::optionString() const
{
    return simImpl->controllerOptionString_;
}


bool ControllerInfo::enableLog()
{
    logBuf.reset(new ReferencedObjectSeq);
    logBuf->setFrameRate(simImpl->worldFrameRate);
    logBufFrameOffset = 0;

    string logName = simImpl->self->name() + "-" + controller->name();
    logItem = controller->findChildItem<ControllerLogItem>(logName);
    if(logItem){
        logItem->resetSeq();
    } else {
        logItem = controller->createLogItem();
        logItem->setTemporal();
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


void ControllerInfo::outputLog(Referenced* logData)
{
    std::lock_guard<std::mutex> lock(logMutex);

    if(!lastLogData){
        lastLogData = logData;
    }

    const int bufFrame = simImpl->currentFrame - logBufFrameOffset;
    const int numFrames = bufFrame + 1;
    const int lastNumFrames = logBuf->numFrames();

    logBuf->setNumFrames(numFrames);

    for(int i = lastNumFrames; i < numFrames - 1; ++i){
        logBuf->at(i) = lastLogData;
    }
    logBuf->back() = logData;

    lastLogData = logData;
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


void SimulationBody::Impl::findControlSrcItems(Item* item, vector<Item*>& io_items)
{
    while(item){
        if(!dynamic_cast<BodyItem*>(item)){
            if(auto controllerItem = dynamic_cast<ControllerItem*>(item)){
                io_items.push_back(controllerItem);
            } else if(item->childItem()){
                findControlSrcItems(item->childItem(), io_items);
            }
        }
        item = item->nextItem();
    }
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

    body_->setCurrentTimeFunction([this](){ return this->simImpl->currentTime(); });
    body_->initializeState();

    isDynamic = !body_->isStaticModel();
    bool doReset = simImpl->doReset && isDynamic;
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
    this->simImpl = simImpl;
    simImpl->hasControllers = true;

    ControllerInfoPtr info = new ControllerInfo(controllerItem, this);

    if(!controllerItem->initialize(info)){
        return false;
    }
    controllerInfos.push_back(info);

    linkPosBuf.resizeColumn(0);

    return true;
}


void SimulationBody::Impl::extractAssociatedItems(bool doReset)
{
    vector<Item*> controlSrcItems;
    findControlSrcItems(bodyItem->childItem(), controlSrcItems);
    auto iter = controlSrcItems.begin();
    while(iter != controlSrcItems.end()){
        Item* srcItem = *iter;
        auto controllerItem = dynamic_cast<ControllerItem*>(srcItem);
        if(controllerItem){
            simImpl->hasControllers = true;
            ControllerInfoPtr info = new ControllerInfo(controllerItem, this);
            if(controllerItem->initialize(info)){
                controllerInfos.push_back(info);
            } else {
                controllerItem = nullptr;
            }
        }
        if(controllerItem){
            ++iter;
        } else {
            iter = controlSrcItems.erase(iter);
        }
    }

    if(controlSrcItems.empty()){
        parentOfRecordItems = bodyItem;

    } else if(controlSrcItems.size() == 1){
        parentOfRecordItems = controlSrcItems.front();

    } else {
        // find the common owner of all the controllers
        int minDepth = std::numeric_limits<int>::max();
        for(size_t i=0; i < controlSrcItems.size(); ++i){
            Item* owner = controlSrcItems[i]->parentItem();
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


void SimulationBody::Impl::copyStateToBodyItem()
{
    BodyState state(*body_);
    state.restorePositions(*bodyItem->body());
}


void SimulationBody::cloneShapesOnce()
{
    if(!impl->areShapesCloned){
        if(!impl->simImpl){
            // throw exception
        }
        impl->body_->cloneShapes(impl->simImpl->cloneMap);
        impl->areShapesCloned = true;
    }
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

    self->bufferRecords(); // put the intial state
}


void SimulationBody::initializeRecordBuffers()
{
    impl->initializeRecordBuffers();
}


void SimulationBody::Impl::initializeRecordBuffers()
{
    jointPosBuf.resizeColumn(body_->numAllJoints());
    int numLinksToRecord = 0;
    if(isDynamic){
        numLinksToRecord = simImpl->isAllLinkPositionOutputMode ? body_->numLinks() : 1;
    }
    linkPosBuf.resizeColumn(numLinksToRecord);

    const DeviceList<>& devices = body_->devices();
    const int numDevices = devices.size();
    deviceStateConnections.disconnect();
    deviceStateChangeFlag.clear();
    deviceStateChangeFlag.resize(numDevices, true); // set all the bits to store the initial states
    devicesToNotifyRecords.clear();
    
    if(devices.empty() || !simImpl->isDeviceStateOutputEnabled){
        deviceStateBuf.clear();
        prevFlushedDeviceStateInDirectMode.clear();
    } else {
        /**
           Temporary code to avoid a bug in storing device states by reserving sufficient buffer size.
           The original bug is in Deque2D's resize operation.
        */
        deviceStateBuf.resize(100, numDevices); 
        
        // This buf always has the first element to keep unchanged states
        deviceStateBuf.resize(1, numDevices); 
        prevFlushedDeviceStateInDirectMode.resize(numDevices);
        for(size_t i=0; i < devices.size(); ++i){
            deviceStateConnections.add(
                devices[i]->sigStateChanged().connect(
                    [this, i](){
                        /** \note This must be thread safe
                            if notifyStateChange is called from several threads.
                        */
                        deviceStateChangeFlag[i] = true;
                    }));
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
    if(!motionItem){
        motionItem = new BodyMotionItem();
        motionItem->setTemporal();
        motionItem->setName(recordItemPrefix);
        doAddMotionItem = true;
    }

    motion = motionItem->motion();
    motion->setFrameRate(simImpl->worldFrameRate);
    motion->setDimension(0, jointPosBuf.colSize(), linkPosBuf.colSize());
    motion->setOffsetTime(0.0);
    jointPosRecord = motion->jointPosSeq();
    linkPosRecordItem = motionItem->linkPosSeqItem();
    linkPosRecord = motion->linkPosSeq();

    const int numDevices = deviceStateBuf.colSize();
    if(numDevices == 0 || !simImpl->isDeviceStateOutputEnabled){
        clearMultiDeviceStateSeq(*motion);
    } else {
        deviceStateRecord = getOrCreateMultiDeviceStateSeq(*motion);
        deviceStateRecord->initialize(body_->devices());
    }

    if(doAddMotionItem){
        parentOfRecordItems->addChildItem(motionItem);
    }
    simImpl->logEngine->addSubEnginesFor(motionItem);
}


void SimulationBody::Impl::setInitialStateOfBodyMotion(shared_ptr<BodyMotion> bodyMotion)
{
    bool updated = false;
    
    auto lseq = bodyMotion->linkPosSeq();
    if(lseq->numParts() > 0 && lseq->numFrames() > 0){
        SE3& p = lseq->at(0, 0);
        Link* rootLink = body_->rootLink();
        rootLink->p() = p.translation();
        rootLink->R() = p.rotation().toRotationMatrix();
        updated = true;
    }
    auto jseq = bodyMotion->jointPosSeq();
    if(jseq->numFrames() > 0){
        auto jframe0 = jseq->frame(0);
        int n = std::min(jframe0.size(), body_->numJoints());
        for(int i=0; i < n; ++i){
            body_->joint(i)->q() = jframe0[i];
        }
        updated = true;
    }
    if(updated){
        body_->calcForwardKinematics();
    }
}


bool SimulationBody::isActive() const
{
    return impl->isActive;
}


void SimulationBody::setActive(bool on)
{
    impl->setActive(on);
}


void SimulationBody::Impl::setActive(bool on)
{
    if(body_){
        if(on){
            if(!isActive){
                simImpl->recordBufMutex.lock();
                self->initializeRecordBuffers();
                self->bufferRecords();
                simImpl->recordBufMutex.unlock();
                isActive = true;
                simImpl->needToUpdateSimBodyLists = true;
            }
        } else {
            if(isActive){
                isActive = false;
                simImpl->needToUpdateSimBodyLists = true;
            }
        }
    }
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
    if(jointPosBuf.colSize() > 0){
        Deque2D<double>::Row q = jointPosBuf.append();
        for(int i=0; i < q.size() ; ++i){
            q[i] = body_->joint(i)->q();
        }
    }
    if(linkPosBuf.colSize() > 0){
        MultiSE3Deque::Row pos = linkPosBuf.append();
        for(int i=0; i < linkPosBuf.colSize(); ++i){
            Link* link = body_->link(i);
            pos[i].set(link->p(), link->R());
        }
    }
    if(deviceStateBuf.colSize() > 0){
        const int prevIndex = std::max(0, deviceStateBuf.rowSize() - 1);
        Deque2D<DeviceStatePtr>::Row current = deviceStateBuf.append();
        Deque2D<DeviceStatePtr>::Row prev = deviceStateBuf[prevIndex];
        const DeviceList<>& devices = body_->devices();
        for(size_t i=0; i < devices.size(); ++i){
            if(deviceStateChangeFlag[i]){
                current[i] = devices[i]->cloneState();
                deviceStateChangeFlag[i] = false;
            } else {
                current[i] = prev[i];
            }
        }
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
        flushRecordsToBody();
    }

    // clear buffers
    linkPosBuf.resizeRow(0);
    jointPosBuf.resizeRow(0);

    // keep the last state so that unchanged states can be shared
    const int numPops = (deviceStateBuf.rowSize() >= 2) ? (deviceStateBuf.rowSize() - 1) : 0;
    deviceStateBuf.pop_front(numPops);
}


void SimulationBody::Impl::flushRecordsToBodyMotionItems()
{
    if(!linkPosRecord){
        initializeRecordItems();
    }

    const int ringBufferSize = simImpl->ringBufferSize;
    const int numBufFrames = linkPosBuf.rowSize();
    const int nextFrame = simImpl->currentFrame + 1;

    if(linkPosBuf.colSize() > 0){
        bool offsetChanged = false;
        for(int i=0; i < numBufFrames; ++i){
            auto buf = linkPosBuf.row(i);
            if(linkPosRecord->numFrames() >= ringBufferSize){
                linkPosRecord->popFrontFrame();
                offsetChanged = true;
            }
            std::copy(buf.begin(), buf.end(), linkPosRecord->appendFrame().begin());
        }
        if(offsetChanged){
            linkPosRecord->setOffsetTimeFrame(nextFrame - linkPosRecord->numFrames());
        }
    }
    if(jointPosBuf.colSize() > 0){
        bool offsetChanged = false;
        for(int i=0; i < numBufFrames; ++i){
            auto buf = jointPosBuf.row(i);
            if(jointPosRecord->numFrames() >= ringBufferSize){
                jointPosRecord->popFrontFrame();
                offsetChanged = true;
            }
            std::copy(buf.begin(), buf.end(), jointPosRecord->appendFrame().begin());
        }
        if(offsetChanged){
            jointPosRecord->setOffsetTimeFrame(nextFrame - jointPosRecord->numFrames());
        }
    }
    if(deviceStateBuf.colSize() > 0){
        bool offsetChanged = false;
        // This loop begins with the second element to skip the first element to keep the unchanged states
        for(int i=1; i < deviceStateBuf.rowSize(); ++i){ 
            auto buf = deviceStateBuf.row(i);
            if(deviceStateRecord->numFrames() >= ringBufferSize){
                deviceStateRecord->popFrontFrame();
                offsetChanged = true;
            }
            std::copy(buf.begin(), buf.end(), deviceStateRecord->appendFrame().begin());
        }
        if(offsetChanged){
            deviceStateRecord->setOffsetTimeFrame(nextFrame - deviceStateRecord->numFrames());
        }
    }
}


void SimulationBody::Impl::flushRecordsToBody()
{
    Body* orgBody = bodyItem->body();
    if(!linkPosBuf.empty()){
        auto last = linkPosBuf.last();
        const int n = last.size();
        for(int i=0; i < n; ++i){
            SE3& pos = last[i];
            Link* link = orgBody->link(i);
            link->p() = pos.translation();
            link->R() = pos.rotation().toRotationMatrix();
        }
    }
    if(!jointPosBuf.empty()){
        auto last = jointPosBuf.last();
        const int n = body_->numJoints();
        for(int i=0; i < n; ++i){
            orgBody->joint(i)->q() = last[i];
        }
    }
    if(!deviceStateBuf.empty()){
        devicesToNotifyRecords.clear();
        const DeviceList<>& devices = orgBody->devices();
        auto ds = deviceStateBuf.last();
        const int ndevices = devices.size();
        for(int i=0; i < ndevices; ++i){
            const DeviceStatePtr& s = ds[i];
            if(s != prevFlushedDeviceStateInDirectMode[i]){
                Device* device = devices[i];
                device->copyStateFrom(*s);
                prevFlushedDeviceStateInDirectMode[i] = s;
                devicesToNotifyRecords.push_back(device);
            }
        }
    }
}


void SimulationBody::Impl::flushRecordsToWorldLogFile(int bufferFrame)
{
    //if(bufferFrame < linkPosBuf.rowSize()){
    
    WorldLogFileItem* log = simImpl->worldLogFileItem;
    log->beginBodyStateOutput();

    if(linkPosBuf.colSize() > 0){
        auto posbuf = linkPosBuf.row(bufferFrame);
        log->outputLinkPositions(posbuf.begin(), posbuf.size());
    }
    if(jointPosBuf.colSize() > 0){
        auto jointbuf = jointPosBuf.row(bufferFrame);
        log->outputJointPositions(jointbuf.begin(), jointbuf.size());
    }
    if(deviceStateBuf.colSize() > 0){
        // Skip the first element because it is used for sharing an unchanged state
        auto states = deviceStateBuf.row(bufferFrame + 1);
        log->beginDeviceStateOutput();
        for(int i=0; i < states.size(); ++i){
            log->outputDeviceState(states[i]);
        }
        log->endDeviceStateOutput();
    }

    log->endBodyStateOutput();

    //}
}


/**
   This function is called when the no recording mode
*/
void SimulationBody::Impl::notifyRecords(double time)
{
    if(isDynamic){
        bodyItem->notifyKinematicStateChange(!simImpl->isAllLinkPositionOutputMode);
    }
    for(Device* device : devicesToNotifyRecords){
        device->notifyStateChange();
    }
    for(Device* device : bodyItem->body()->devices()){
        device->notifyTimeChange(time);
    }
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
      mv(MessageView::instance())
{
    worldItem = nullptr;
    
    temporalResolutionType.setSymbol(RESOLUTION_TIMESTEP, N_("Timestep"));
    temporalResolutionType.setSymbol(RESOLUTION_FRAMERATE, N_("Framerate"));
    temporalResolutionType.setSymbol(RESOLUTION_TIMEBAR, N_("Time bar"));
    timeStepProperty = 0.001;
    frameRateProperty = 1000;

    currentFrame = 0;
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

    timeLength = 180.0; // 3 min.
    useControllerThreadsProperty = true;
    isActiveControlTimeRangeMode = false;
    isAllLinkPositionOutputMode = true;
    isDeviceStateOutputEnabled = true;
    isDoingSimulationLoop = false;
    isRealtimeSyncMode = true;
    recordCollisionData = false;

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

    timeLength = org.timeLength;
    useControllerThreadsProperty = org.useControllerThreadsProperty;
    isActiveControlTimeRangeMode = org.isActiveControlTimeRangeMode;
    isAllLinkPositionOutputMode = org.isAllLinkPositionOutputMode;
    isDeviceStateOutputEnabled = org.isDeviceStateOutputEnabled;
    isRealtimeSyncMode = org.isRealtimeSyncMode;
    recordCollisionData = org.recordCollisionData;
    controllerOptionString_ = org.controllerOptionString_;
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
        return TimeBar::instance()->timeStep();
    }
}


void SimulatorItem::setTimeStep(double step)
{
    if(step > 0.0){
        impl->temporalResolutionType.select(RESOLUTION_TIMESTEP);
        impl->timeStepProperty = step;
    }
}


void SimulatorItem::setRecordingMode(int selection)
{
    impl->recordingMode.select(selection);
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


void SimulatorItem::setRealtimeSyncMode(bool on)
{
    impl->isRealtimeSyncMode = on;
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
    simBodiesWithBody.clear();;
    activeSimBodies.clear();
    needToUpdateSimBodyLists = true;

    preDynamicsFunctions.clear();
    midDynamicsFunctions.clear();
    postDynamicsFunctions.clear();

    subSimulatorItems.clear();

    hasControllers = false;

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
    
    this->doReset = doReset;
    
    stopSimulation(true, true);

    if(!worldItem){
        mv->putln(format(_("{} must be in a WorldItem to do simulation."), self->displayName()),
                  MessageView::Error);
        return false;
    }

    ItemList<Item> targetItems;
    findTargetItems(worldItem, false, targetItems);
    if(targetItems.empty()){
        return false;
    }

    cloneMap.clear();

    currentFrame = 0;
    worldTimeStep_ = self->worldTimeStep();
    worldFrameRate = 1.0 / worldTimeStep_;

    if(recordingMode.is(SimulatorItem::REC_NONE)){
        isRecordingEnabled = false;
        isRingBufferMode = false;
    } else {
        isRecordingEnabled = true;
        isRingBufferMode = recordingMode.is(SimulatorItem::REC_TAIL);
    }

    clearSimulation();
    getOrCreateLogEngine()->clearSubEngines();

    for(size_t i=0; i < targetItems.size(); ++i){

        if(auto bodyItem = dynamic_cast<BodyItem*>(targetItems.get(i))){
            if(doReset){
                bodyItem->restoreInitialState(false);
            }
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
                    if(simBody->initialize(self, bodyItem)){
                        // copy the body state overwritten by the controller
                        simBody->impl->copyStateToBodyItem();
                        
                        allSimBodies.push_back(simBody);
                        simBodiesWithBody.push_back(simBody);
                        simBodyMap[bodyItem] = simBody;
                    }
                }
            }
            bodyItem->notifyKinematicStateChange();
            
        } else if(auto controller = dynamic_cast<ControllerItem*>(targetItems.get(i))){
            // ControllerItem which is not associated with a body
            SimulationBodyPtr simBody = new SimulationBody(nullptr);
            if(simBody->impl->initialize(this, controller)){
                allSimBodies.push_back(simBody);
            }
        } else if(auto script = dynamic_cast<SimulationScriptItem*>(targetItems.get(i))){
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
    
    bool result = self->initializeSimulation(simBodiesWithBody);

    if(result){

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

        for(auto& simBody : allSimBodies){
            auto simBodyImpl = simBody->impl;
            Body* body = simBodyImpl->body_;
            auto& controllerInfos = simBodyImpl->controllerInfos;
            auto iter = controllerInfos.begin();
            while(iter != controllerInfos.end()){
                auto& info = *iter;
                ControllerItem* controller = info->controller;
                bool ready = false;
                controller->setSimulatorItem(self);
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
                    ++iter;
                } else {
                    controller->setSimulatorItem(nullptr);
                    iter = controllerInfos.erase(iter);
                }
            }
        }

        updateSimBodyLists();

        doStopSimulationWhenNoActiveControllers = isActiveControlTimeRangeMode && hasControllers;

        if(doStopSimulationWhenNoActiveControllers && activeControllerInfos.empty()){
            mv->putln(_("The simulation cannot be started because all the controllers are inactive."),
                      MessageView::Error);
            result = false;
        }
    }

    if(result){
        result = self->completeInitializationOfSimulation();
    }

    if(result){

        for(auto& simBody : activeSimBodies){
            if(simBody->body()){
                simBody->impl->initializeRecording();
            }
        }

        numBufferedFrames = 1;
    
        if(isRecordingEnabled && recordCollisionData){
            collisionPairsBuf.clear();
            string collisionSeqName = self->name() + "-collisions";
            auto collisionSeqItem = worldItem->findChildItem<CollisionSeqItem>(collisionSeqName);
            if(!collisionSeqItem){
                collisionSeqItem = new CollisionSeqItem();
                collisionSeqItem->setTemporal();
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
        
        frameAtLastBufferWriting = 0;
        isDoingSimulationLoop = true;
        isWaitingForSimulationToStop = false;
        isForcedToStopSimulation = false;
        stopRequested = false;
        pauseRequested = false;

        ringBufferSize = std::numeric_limits<int>::max();
        
        if(timeRangeMode.is(SimulatorItem::TR_SPECIFIED)){
            maxFrame = timeLength / worldTimeStep_;
        } else if(timeRangeMode.is(SimulatorItem::TR_TIMEBAR)){
            maxFrame = TimeBar::instance()->maxTime() / worldTimeStep_;
        } else if(isRingBufferMode){
            maxFrame = std::numeric_limits<int>::max();
            ringBufferSize = timeLength / worldTimeStep_;
        } else {
            maxFrame = std::numeric_limits<int>::max();
        }

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
        aboutToQuitConnection = cnoid::sigAboutToQuit().connect(
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

        if(isRecordingEnabled){
            logEngine->startOngoingTimeUpdate(0.0);
        }

        flushRecords();
        start();
        flushTimer.start(1000.0 / timeBar->playbackFrameRate());

        mv->notify(format(_("Simulation by {} has started."), self->displayName()));

        sigSimulationStarted();
    }

    if(!result){
        mv->notify(format(_("{0} failed to initialize the simulation."), self->displayName()),
                   MessageView::Error);
        clearSimulation();
    }

    return result;
}


void SimulatorItem::Impl::updateSimBodyLists()
{
    activeSimBodies.clear();
    activeControllerInfos.clear();
    loggingControllers.clear();
    hasActiveFreeBodies = false;
    
    for(size_t i=0; i < allSimBodies.size(); ++i){
        SimulationBody* simBody = allSimBodies[i];
        auto simBodyImpl = simBody->impl;
        auto& controllerInfos = simBodyImpl->controllerInfos;
        if(simBodyImpl->isActive){
            activeSimBodies.push_back(simBody);
            if(controllerInfos.empty()){
                hasActiveFreeBodies = true;
            }
        }
        for(auto& info : controllerInfos){
            activeControllerInfos.push_back(info);
            if(info->isLogEnabled()){
                loggingControllers.push_back(info->controller);
            }
       }
    }

    needToUpdateSimBodyLists = false;
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

    if(isRealtimeSyncMode){
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
                compensatedSimulationTime += dtms;
                ++frame;
            }
        }
    } else {
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
    currentFrame++;

    if(needToUpdateSimBodyLists){
        updateSimBodyLists();
    }
    
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

    shared_ptr<CollisionLinkPairList> collisionPairs;
    if(isRecordingEnabled && recordCollisionData){
        collisionPairs = self->getCollisions();
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

    for(auto& controller : loggingControllers){
        controller->log();
    }

    {
        recordBufMutex.lock();

        ++numBufferedFrames;
        for(size_t i=0; i < activeSimBodies.size(); ++i){
            activeSimBodies[i]->bufferRecords();
        }
        collisionPairsBuf.push_back(collisionPairs);
        frameAtLastBufferWriting = currentFrame;

        recordBufMutex.unlock();
    }

    for(auto& info : activeControllerInfos){
        if(!info->controller->isNoDelayMode()){
            info->controller->output();
        }
    }

    return doContinue;
}


namespace {

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

}


void SimulatorItem::Impl::flushRecords()
{
    int frame = flushMainRecords();

    for(auto& info : loggedControllerInfos){
        info->flushLog();
    }

    if(isRecordingEnabled){
        logEngine->updateOngoingTime(frame / worldFrameRate);
        
    } else {
        const double time = frame / worldFrameRate;
        for(size_t i=0; i < activeSimBodies.size(); ++i){
            activeSimBodies[i]->impl->notifyRecords(time);
        }
        if(self->isSelected()){
            timeBar->setTime(time);
        }
    }
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
    if(isRecordingEnabled && recordCollisionData){
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
    if(isRecordingEnabled){
        logEngine->stopOngoingTimeUpdate();
    }
    pauseRequested = true;
    flushRecords();
}


void SimulatorItem::restartSimulation()
{
    impl->restartSimulation();
}


void SimulatorItem::Impl::restartSimulation()
{
    if(pauseRequested){
        if(isRecordingEnabled){
            logEngine->startOngoingTimeUpdate();
        }
        pauseRequested = false;
        flushTimer.start(1000.0 / timeBar->playbackFrameRate());
    }
}


void SimulatorItem::stopSimulation(bool isForced)
{
    impl->stopSimulation(isForced, false);
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
            controller->setSimulatorItem(nullptr);
        }
    }
    self->finalizeSimulation();

    for(auto& subSimulator : subSimulatorItems){
        subSimulator->finalizeSimulation();
    }

    flushRecords();

    logEngine->stopOngoingTimeUpdate();

    mv->notify(format(_("Simulation by {0} has finished at {1} [s]."), self->displayName(), finishTime));

    if(finishTime > 0.0){
        mv->putln(format(_("Computation time is {0} [s], computation time / simulation time = {1}."),
                         actualSimulationTime, (actualSimulationTime / finishTime)));
    }

    clearSimulation();

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
    return impl->isDoingSimulationLoop && !impl->pauseRequested;
}


int SimulatorItem::currentFrame() const
{
    return impl->currentFrame;
}


double SimulatorItem::currentTime() const
{
    return impl->currentFrame / impl->worldFrameRate;
}


double SimulatorItem::Impl::currentTime() const
{
    return currentFrame / worldFrameRate;
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


double SimulatorItem::Impl::timeStep() const
{
    return worldTimeStep_;
}


std::string SimulatorItem::Impl::controllerName() const
{
    return string();
}


Body* SimulatorItem::Impl::body()
{
    return nullptr;
}


std::string SimulatorItem::Impl::optionString() const
{
    return controllerOptionString_;
}


std::ostream& SimulatorItem::Impl::os() const
{
    return mv->cout();
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


void SimulatorItem::Impl::onRealtimeSyncChanged(bool on)
{
    isRealtimeSyncMode = on;
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

    putProperty(_("Sync with realtime"), isRealtimeSyncMode,
                [&](bool on){ onRealtimeSyncChanged(on); return true; });
    putProperty(_("Time range"), timeRangeMode,
                [&](int index){ return timeRangeMode.select(index); });
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
    putProperty(_("Record collision data"), recordCollisionData,
                changeProperty(recordCollisionData));
    putProperty(_("Controller Threads"), useControllerThreadsProperty,
                changeProperty(useControllerThreadsProperty));
    putProperty(_("Controller options"), controllerOptionString_,
                changeProperty(controllerOptionString_));
}


bool SimulatorItem::store(Archive& archive)
{
    return impl->store(archive);

}


bool SimulatorItem::Impl::store(Archive& archive)
{
    if(temporalResolutionType.is(RESOLUTION_TIMESTEP)){
        archive.write("timeStep", timeStepProperty.string());
    } else if(temporalResolutionType.is(RESOLUTION_FRAMERATE)){
        archive.write("frameRate", frameRateProperty);
    }
    archive.write("realtimeSync", isRealtimeSyncMode);
    archive.write("recording", recordingMode.selectedSymbol(), DOUBLE_QUOTED);
    archive.write("timeRangeMode", timeRangeMode.selectedSymbol(), DOUBLE_QUOTED);
    archive.write("timeLength", timeLength);
    archive.write("active_control_time_range_mode", isActiveControlTimeRangeMode);
    archive.write("allLinkPositionOutputMode", isAllLinkPositionOutputMode);
    archive.write("deviceStateOutput", isDeviceStateOutputEnabled);
    archive.write("controllerThreads", useControllerThreadsProperty);
    archive.write("recordCollisionData", recordCollisionData);
    archive.write("controllerOptions", controllerOptionString_, DOUBLE_QUOTED);
    
    ListingPtr idseq = new Listing();
    idseq->setFlowStyle(true);
    for(auto& engine : getOrCreateLogEngine()->subEngines){
        if(ValueNodePtr id = archive.getItemId(engine->item())){
            idseq->append(id);
        }
    }
    if(!idseq->empty()){
        archive.insert("time_sync_items", idseq);
    }

    if(auto engine = logEngine->collisionSeqEngine){
        if(ValueNodePtr id = archive.getItemId(engine->collisionSeqItem())){
            archive.insert("collisionSeqItem", id);
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

    if(archive.read("timeStep", symbol) || archive.read("timestep", symbol)){
        timeStepProperty = symbol;
        temporalResolutionType.select(RESOLUTION_TIMESTEP);
    } else if(archive.read("frameRate", frameRateProperty) || archive.read("framerate", frameRateProperty)){
        temporalResolutionType.select(RESOLUTION_FRAMERATE);
    } else {
        temporalResolutionType.select(RESOLUTION_TIMEBAR);
    }

    if(archive.read("timeRangeMode", symbol)){
        if(timeRangeMode.select(symbol)){
            self->setTimeRangeMode(timeRangeMode.which());
        } else if(symbol == "Active control period"){ // Old
            self->setTimeRangeMode(ActiveControlTime);
        }
    }
    isActiveControlTimeRangeMode = archive.get("active_control_time_range_mode", isActiveControlTimeRangeMode);

    if(archive.read("recording", symbol)){
        recordingMode.select(symbol);
    }
    // for the compatibility with older version
    else if(archive.read("recording", boolValue) && boolValue){ 
        recordingMode.select(SimulatorItem::REC_FULL);
    } else if(archive.read("recordingMode", symbol)){
        if(symbol == "Direct"){
            recordingMode.select(SimulatorItem::REC_NONE);
            timeRangeMode.select(SimulatorItem::TR_UNLIMITED);
        }
    }
    archive.read("realtimeSync", isRealtimeSyncMode);
    archive.read("timeLength", timeLength);
    self->setAllLinkPositionOutputMode(archive.get("allLinkPositionOutputMode", isAllLinkPositionOutputMode));
    archive.read("deviceStateOutput", isDeviceStateOutputEnabled);
    archive.read("recordCollisionData", recordCollisionData);
    archive.read("controllerThreads", useControllerThreadsProperty);
    archive.read("controllerOptions", controllerOptionString_);

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


namespace {

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
    notifyKinematicStateUpdate();
}


bool SimulationLogEngine::isTimeSyncAlwaysMaintained() const
{
    return doKeepPlayback;
}


bool SimulationLogEngine::onTimeChanged(double time)
{
    bool processed = false;
    auto& si = itemImpl;
    if(!subEngines.empty()){
        for(auto& engine : subEngines){
            processed |= engine->onTimeChanged(time);
        }
    } else if(si->worldLogFileItem){
        processed |= si->worldLogFileItem->recallStateAtTime(time);
    }
    if(collisionSeqEngine){
        processed |= collisionSeqEngine->onTimeChanged(time);
    }
    return processed;
}


void SimulationLogEngine::onPlaybackStopped(double /* time */, bool /* isStoppedManually */)
{
    doKeepPlayback = false;
    notifyKinematicStateUpdate();
}


void SimulationLogEngine::notifyKinematicStateUpdate()
{
    auto& si = itemImpl;
    if(si->worldItem){
        for(auto& bodyItem : si->worldItem->descendantItems<BodyItem>()){
            if(!bodyItem->body()->isStaticModel()){
                bodyItem->notifyKinematicStateUpdate(false);
            }
        }
    }
}

}
