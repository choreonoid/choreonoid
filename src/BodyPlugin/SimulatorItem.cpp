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
#include "BodyMotionEngine.h"
#include "WorldLogFileItem.h"
#include "CollisionSeq.h"
#include "CollisionSeqItem.h"
#include "CollisionSeqEngine.h"
#include <cnoid/ControllerIO>
#include <cnoid/BodyState>
#include <cnoid/AppUtil>
#include <cnoid/ExtensionManager>
#include <cnoid/TimeBar>
#include <cnoid/ItemTreeView>
#include <cnoid/MessageView>
#include <cnoid/LazyCaller>
#include <cnoid/Archive>
#include <cnoid/MultiDeviceStateSeq>
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

#ifdef ENABLE_SIMULATION_PROFILING
#include <cnoid/ViewManager>
#include <cnoid/SceneView>
#include <cnoid/SceneWidget>
#endif

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
    SimulatorItemImpl* simImpl;
    int idCounter;
    bool needToUpdate;
    vector<FunctionInfo> functionsToAdd;
    set<int> registerdIds;
    vector<int> idsToRemove;
        
    FunctionSet(SimulatorItemImpl* simImpl) : simImpl(simImpl) {
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

}

namespace cnoid {

class SimulationBodyImpl : public ControllerIO
{
public:
    SimulationBody* self;
    BodyPtr body_;
    BodyItemPtr bodyItem;
    vector<ControllerItemPtr> controllers;
    double frameRate;
    SimulatorItemImpl* simImpl;

    bool isActive;
    bool isDynamic;
    bool areShapesCloned;

    Deque2D<double> jointPosBuf;
    MultiSE3Deque linkPosBuf;
    vector<Device*> devicesToNotifyResults;
    ScopedConnectionSet deviceStateConnections;
    vector<bool> deviceStateChangeFlag;
    Deque2D<DeviceStatePtr> deviceStateBuf;

    ItemPtr parentOfResultItems;
    string resultItemPrefix;
    shared_ptr<BodyMotion> motion;
    shared_ptr<MultiValueSeq> jointPosResults;
    shared_ptr<MultiSE3Seq> linkPosResults;
    MultiSE3SeqItemPtr linkPosResultItem;
    vector<DeviceStatePtr> prevFlushedDeviceStateInDirectMode;
    shared_ptr<MultiDeviceStateSeq> deviceStateResults;

    SimulationBodyImpl(SimulationBody* self, Body* body);
    void findControlSrcItems(Item* item, vector<Item*>& io_items, bool doPickCheckedItems = false);
    bool initialize(SimulatorItem* simulatorItem, BodyItem* bodyItem);
    bool initialize(SimulatorItemImpl* simImpl, ControllerItem* controllerItem);
    void extractAssociatedItems(bool doReset);
    void copyStateToBodyItem();
    void cloneShapesOnce();
    void initializeResultData();
    void initializeResultBuffers();
    void initializeResultItems();
    void setInitialStateOfBodyMotion(shared_ptr<BodyMotion> bodyMotion);
    void setActive(bool on);
    void bufferResults();
    void flushResults();
    void flushResultsToBodyMotionItems();
    void flushResultsToBody();
    void flushResultsToWorldLogFile(int bufferFrame);
    void notifyResults(double time);

    // Following functions are defined in the ControllerIO class
    virtual Body* body() override;
    std::ostream& os() const;
    virtual double timeStep() const override;
    virtual double currentTime() const override;
    virtual std::string optionString() const override;
};


class SimulatorItemImpl : public QThread, public ControllerIO
{
public:
    SimulatorItemImpl(SimulatorItem* self);
    SimulatorItemImpl(SimulatorItem* self, const SimulatorItemImpl& org);
    ~SimulatorItemImpl();
            
    SimulatorItem* self;
    WorldItem* worldItem;

    vector<SimulationBodyPtr> allSimBodies;
    vector<SimulationBody*> simBodiesWithBody;
    vector<SimulationBody*> activeSimBodies;

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
    
    vector<SimulationBodyImpl*> simBodyImplsToNotifyResults;
    ItemList<SubSimulatorItem> subSimulatorItems;

    vector<ControllerItem*> activeControllers;
    std::thread controlThread;
    std::condition_variable controlCondition;
    std::mutex controlMutex;
    bool isExitingControlLoopRequested;
    bool isControlRequested;
    bool isControlFinished;
    bool isControlToBeContinued;
    bool doCheckContinue;

    CollisionDetectorPtr collisionDetector;

    shared_ptr<CollisionSeq> collisionSeq;
    deque<shared_ptr<CollisionLinkPairList>> collisionPairsBuf;

    Selection recordingMode;
    Selection timeRangeMode;
    double specifiedTimeLength;
    int maxFrame;
    int ringBufferSize;
    bool isRecordingEnabled;
    bool isRingBufferMode;
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
    int fillLevelId;
    QMutex resultBufMutex;
    double actualSimulationTime;
    double finishTime;
    MessageView* mv;

    bool doReset;
    bool isWaitingForSimulationToStop;
    Signal<void()> sigSimulationStarted;
    Signal<void()> sigSimulationPaused;
    Signal<void()> sigSimulationResumed;
    Signal<void()> sigSimulationFinished;

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

    vector<BodyMotionEnginePtr> bodyMotionEngines;
    CollisionSeqEnginePtr collisionSeqEngine;

    Connection aboutToQuitConnection;

    CloneMap cloneMap;
        
    ItemTreeView* itemTreeView;

#ifdef ENABLE_SIMULATION_PROFILING
    double controllerTime;
    QElapsedTimer timer;
    Deque2D<double> simProfilingBuf;
    MultiValueSeq simProfilingSeq;
    SceneWidget* sw;
#endif

    void findTargetItems(Item* item, bool isUnderBodyItem, ItemList<Item>& out_targetItems);
    void clearSimulation();
    bool startSimulation(bool doReset);
    virtual void run();
    void onSimulationLoopStarted();
    void updateSimBodyLists();
    bool stepSimulationMain();
    void concurrentControlLoop();
    void flushResults();
    void stopSimulation(bool doSync);
    void pauseSimulation();
    void restartSimulation();
    void onSimulationLoopStopped();
    void setExternalForce(BodyItem* bodyItem, Link* link, const Vector3& point, const Vector3& f, double time);
    void doSetExternalForce();
    void setVirtualElasticString(
        BodyItem* bodyItem, Link* link, const Vector3& attachmentPoint, const Vector3& endPoint);
    void setVirtualElasticStringForce();
    void onRealtimeSyncChanged(bool on);
    bool onAllLinkPositionOutputModeChanged(bool on);
    void setSpecifiedRecordingTimeLength(double length);
    void doPutProperties(PutPropertyFunction& putProperty);
    bool store(Archive& archive);
    bool restore(const Archive& archive);
    void restoreBodyMotionEngines(const Archive& archive);
    void addBodyMotionEngine(BodyMotionItem* motionItem);
    bool setPlaybackTime(double time);
    void addCollisionSeqEngine(CollisionSeqItem* collisionSeqItem);

    // Functions defined in the ControllerIO class
    virtual Body* body() override;
    virtual std::string optionString() const override;
    virtual std::ostream& os() const override;
    virtual double timeStep() const override;
    virtual double currentTime() const override;
};


class SimulatedMotionEngineManager
{
public:
    ItemList<SimulatorItem> simulatorItems;
    ScopedConnection selectionOrTreeChangedConnection;
    ScopedConnection timeChangeConnection;

    SimulatedMotionEngineManager(){
        selectionOrTreeChangedConnection.reset(
            ItemTreeView::instance()->sigSelectionOrTreeChanged().connect(
                [&](const ItemList<SimulatorItem>& selected){
                    onItemSelectionOrTreeChanged(selected);
                }));
    }

    void onItemSelectionOrTreeChanged(const ItemList<SimulatorItem>& selected){

        bool changed = false;
        
        if(selected.empty()){
            vector<SimulatorItemPtr>::iterator p = simulatorItems.begin();
            while(p != simulatorItems.end()){
                if((*p)->isRunning() && (*p)->findRootItem()){
                    ++p;
                } else {
                    p = simulatorItems.erase(p);
                    changed = true;
                }
            }
        } else {
            if(simulatorItems != selected){
                simulatorItems = selected;
                changed = true;
            }
        }
        if(changed){
            if(simulatorItems.empty()){
                timeChangeConnection.disconnect();
            } else {
                TimeBar* timeBar = TimeBar::instance();
                timeChangeConnection.reset(
                    timeBar->sigTimeChanged().connect(
                        [&](double time){ return setTime(time); }));
                setTime(timeBar->time());
            }
        }
    }

    bool setTime(double time);
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
    ext->manage(new SimulatedMotionEngineManager());
}


SimulatorItem* SimulatorItem::findActiveSimulatorItemFor(Item* item)
{
    SimulatorItem* activeSimulatorItem = nullptr;
    if(item){
        WorldItem* worldItem = item->findOwnerItem<WorldItem>();
        if(worldItem){
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


SimulationBody::SimulationBody(Body* body)
{
    impl = new SimulationBodyImpl(this, body);
}


SimulationBodyImpl::SimulationBodyImpl(SimulationBody* self, Body* body)
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
    return impl->controllers.size();
}


ControllerItem* SimulationBody::controller(int index) const
{
    if(index < static_cast<int>(impl->controllers.size())){
        return impl->controllers[index];
    }
    return 0;
}


void SimulationBodyImpl::findControlSrcItems(Item* item, vector<Item*>& io_items, bool doPickCheckedItems)
{
    while(item){
        Item* srcItem = nullptr;
        if(dynamic_cast<ControllerItem*>(item)){
            srcItem = item;
        }
        if(srcItem){
            bool isChecked = ItemTreeView::instance()->isItemChecked(srcItem);
            if(!doPickCheckedItems || isChecked){
                if(!doPickCheckedItems && isChecked){
                    io_items.clear();
                    doPickCheckedItems = true;
                }
                io_items.push_back(srcItem);
            }
        } else if(item->childItem()){
            findControlSrcItems(item->childItem(), io_items, doPickCheckedItems);
        }
        item = item->nextItem();
    }
}


bool SimulationBody::initialize(SimulatorItem* simulatorItem, BodyItem* bodyItem)
{
    return impl->initialize(simulatorItem, bodyItem);
}


bool SimulationBodyImpl::initialize(SimulatorItem* simulatorItem, BodyItem* bodyItem)
{
    simImpl = simulatorItem->impl;
    this->bodyItem = bodyItem;
    frameRate = simImpl->worldFrameRate;
    deviceStateConnections.disconnect();
    controllers.clear();
    resultItemPrefix = simImpl->self->name() + "-" + bodyItem->name();

    body_->setCurrentTimeFunction([this](){ return this->simImpl->currentTime(); });
    body_->initializeState();

    isDynamic = !body_->isStaticModel();
    bool doReset = simImpl->doReset && isDynamic;
    extractAssociatedItems(doReset);
    
    if(!isDynamic && body_->numDevices() == 0){
        return true;
    }

    isActive = true;
    
    initializeResultData();

    self->bufferResults(); // put the intial state
    
    return true;
}


// For a controller which is not associated with a body
bool SimulationBodyImpl::initialize(SimulatorItemImpl* simImpl, ControllerItem* controllerItem)
{
    this->simImpl = simImpl;
    frameRate = simImpl->worldFrameRate;

    if(!controllerItem->initialize(this)){
        return false;
    }
    controllers.push_back(controllerItem);

    linkPosBuf.resizeColumn(0);

    return true;
}


void SimulationBodyImpl::extractAssociatedItems(bool doReset)
{
    vector<Item*> controlSrcItems;
    findControlSrcItems(bodyItem->childItem(), controlSrcItems);
    vector<Item*>::iterator iter = controlSrcItems.begin();
    while(iter != controlSrcItems.end()){
        Item* srcItem = *iter;
        ControllerItem* controllerItem = dynamic_cast<ControllerItem*>(srcItem);
        if(controllerItem){
            if(controllerItem->initialize(this)){
                controllers.push_back(controllerItem);
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
        parentOfResultItems = bodyItem;

    } else if(controlSrcItems.size() == 1){
        parentOfResultItems = controlSrcItems.front();

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
                parentOfResultItems = owner;
                minDepth = depth;
            }
        }
    }
}


void SimulationBodyImpl::copyStateToBodyItem()
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


const std::string& SimulationBody::resultItemPrefix() const
{
    return impl->resultItemPrefix;
}


void SimulationBodyImpl::initializeResultData()
{
    self->initializeResultBuffers();
    
    if(!simImpl->isRecordingEnabled){
        return;
    }

    self->initializeResultItems();
}


void SimulationBody::initializeResultBuffers()
{
    impl->initializeResultBuffers();
}


void SimulationBodyImpl::initializeResultBuffers()
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
    devicesToNotifyResults.clear();
    
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


void SimulationBody::initializeResultItems()
{
    impl->initializeResultItems();
}


void SimulationBodyImpl::initializeResultItems()
{
    if(!parentOfResultItems){
        return;
    }
    
    BodyMotionItem* motionItem = parentOfResultItems->findChildItem<BodyMotionItem>(resultItemPrefix);
    if(!motionItem){
        motionItem = new BodyMotionItem();
        motionItem->setTemporal();
        motionItem->setName(resultItemPrefix);
        parentOfResultItems->addChildItem(motionItem);
    }

    motion = motionItem->motion();
    motion->setFrameRate(frameRate);
    motion->setDimension(0, jointPosBuf.colSize(), linkPosBuf.colSize());
    motion->setOffsetTime(0.0);
    simImpl->addBodyMotionEngine(motionItem);
    jointPosResults = motion->jointPosSeq();
    linkPosResultItem = motionItem->linkPosSeqItem();
    linkPosResults = motion->linkPosSeq();

    const int numDevices = deviceStateBuf.colSize();
    if(numDevices == 0 || !simImpl->isDeviceStateOutputEnabled){
        clearMultiDeviceStateSeq(*motion);
    } else {
        deviceStateResults = getOrCreateMultiDeviceStateSeq(*motion);
        deviceStateResults->initialize(body_->devices());
    }
}


void SimulationBodyImpl::setInitialStateOfBodyMotion(shared_ptr<BodyMotion> bodyMotion)
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
        MultiValueSeq::Frame jframe0 = jseq->frame(0);
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


void SimulationBodyImpl::setActive(bool on)
{
    if(body_){
        if(on){
            if(!isActive){
                simImpl->resultBufMutex.lock();
                self->initializeResultBuffers();
                self->bufferResults();
                simImpl->resultBufMutex.unlock();
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


void SimulationBody::bufferResults()
{
    impl->bufferResults();
}


void SimulationBodyImpl::bufferResults()
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


void SimulationBody::flushResults()
{
    impl->flushResults();
}


void SimulationBodyImpl::flushResults()
{
    if(simImpl->isRecordingEnabled){
        flushResultsToBodyMotionItems();
    } else {
        flushResultsToBody();
    }

    // clear buffers
    linkPosBuf.resizeRow(0);
    jointPosBuf.resizeRow(0);

    // keep the last state so that unchanged states can be shared
    const int numPops = (deviceStateBuf.rowSize() >= 2) ? (deviceStateBuf.rowSize() - 1) : 0;
    deviceStateBuf.pop_front(numPops);
}


void SimulationBodyImpl::flushResultsToBodyMotionItems()
{
    if(!linkPosResults){
        initializeResultItems();
    }

    const int ringBufferSize = simImpl->ringBufferSize;
    const int numBufFrames = linkPosBuf.rowSize();
    const int nextFrame = simImpl->currentFrame + 1;

    if(linkPosBuf.colSize() > 0){
        bool offsetChanged = false;
        for(int i=0; i < numBufFrames; ++i){
            MultiSE3Deque::Row buf = linkPosBuf.row(i);
            if(linkPosResults->numFrames() >= ringBufferSize){
                linkPosResults->popFrontFrame();
                offsetChanged = true;
            }
            std::copy(buf.begin(), buf.end(), linkPosResults->appendFrame().begin());
        }
        if(offsetChanged){
            linkPosResults->setOffsetTimeFrame(nextFrame - linkPosResults->numFrames());
        }
    }
    if(jointPosBuf.colSize() > 0){
        bool offsetChanged = false;
        for(int i=0; i < numBufFrames; ++i){
            Deque2D<double>::Row buf = jointPosBuf.row(i);
            if(jointPosResults->numFrames() >= ringBufferSize){
                jointPosResults->popFrontFrame();
                offsetChanged = true;
            }
            std::copy(buf.begin(), buf.end(), jointPosResults->appendFrame().begin());
        }
        if(offsetChanged){
            jointPosResults->setOffsetTimeFrame(nextFrame - jointPosResults->numFrames());
        }
    }
    if(deviceStateBuf.colSize() > 0){
        bool offsetChanged = false;
        // This loop begins with the second element to skip the first element to keep the unchanged states
        for(int i=1; i < deviceStateBuf.rowSize(); ++i){ 
            Deque2D<DeviceStatePtr>::Row buf = deviceStateBuf.row(i);
            if(deviceStateResults->numFrames() >= ringBufferSize){
                deviceStateResults->popFrontFrame();
                offsetChanged = true;
            }
            std::copy(buf.begin(), buf.end(), deviceStateResults->appendFrame().begin());
        }
        if(offsetChanged){
            deviceStateResults->setOffsetTimeFrame(nextFrame - deviceStateResults->numFrames());
        }
    }
}


void SimulationBodyImpl::flushResultsToBody()
{
    Body* orgBody = bodyItem->body();
    if(!linkPosBuf.empty()){
        MultiSE3Deque::Row last = linkPosBuf.last();
        const int n = last.size();
        for(int i=0; i < n; ++i){
            SE3& pos = last[i];
            Link* link = orgBody->link(i);
            link->p() = pos.translation();
            link->R() = pos.rotation().toRotationMatrix();
        }
    }
    if(!jointPosBuf.empty()){
        Deque2D<double>::Row last = jointPosBuf.last();
        const int n = body_->numJoints();
        for(int i=0; i < n; ++i){
            orgBody->joint(i)->q() = last[i];
        }
    }
    if(!deviceStateBuf.empty()){
        devicesToNotifyResults.clear();
        const DeviceList<>& devices = orgBody->devices();
        Deque2D<DeviceStatePtr>::Row ds = deviceStateBuf.last();
        const int ndevices = devices.size();
        for(int i=0; i < ndevices; ++i){
            const DeviceStatePtr& s = ds[i];
            if(s != prevFlushedDeviceStateInDirectMode[i]){
                Device* device = devices[i];
                device->copyStateFrom(*s);
                prevFlushedDeviceStateInDirectMode[i] = s;
                devicesToNotifyResults.push_back(device);
            }
        }
    }
}


void SimulationBodyImpl::flushResultsToWorldLogFile(int bufferFrame)
{
    //if(bufferFrame < linkPosBuf.rowSize()){
    
    WorldLogFileItem* log = simImpl->worldLogFileItem;
    log->beginBodyStateOutput();

    if(linkPosBuf.colSize() > 0){
        MultiSE3Deque::Row posbuf = linkPosBuf.row(bufferFrame);
        log->outputLinkPositions(posbuf.begin(), posbuf.size());
    }
    if(jointPosBuf.colSize() > 0){
        Deque2D<double>::Row jointbuf = jointPosBuf.row(bufferFrame);
        log->outputJointPositions(jointbuf.begin(), jointbuf.size());
    }
    if(deviceStateBuf.colSize() > 0){
        // Skip the first element because it is used for sharing an unchanged state
        Deque2D<DeviceStatePtr>::Row states = deviceStateBuf.row(bufferFrame + 1);
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
void SimulationBodyImpl::notifyResults(double time)
{
    if(isDynamic){
        bodyItem->notifyKinematicStateChange(!simImpl->isAllLinkPositionOutputMode);
    }
    for(Device* device : devicesToNotifyResults){
        device->notifyStateChange();
    }
    for(Device* device : bodyItem->body()->devices()){
        device->notifyTimeChange(time);
    }
}


Body* SimulationBodyImpl::body()
{
    return body_;
}


std::string SimulationBodyImpl::optionString() const
{
    return simImpl->controllerOptionString_;
}


std::ostream& SimulationBodyImpl::os() const
{
    return simImpl->mv->cout();
}


double SimulationBodyImpl::timeStep() const
{
    return simImpl->worldTimeStep_;
}


double SimulationBodyImpl::currentTime() const
{
    return simImpl->currentTime();
}


SimulatorItem::SimulatorItem()
{
    impl = new SimulatorItemImpl(this);
}


SimulatorItemImpl::SimulatorItemImpl(SimulatorItem* self)
    : self(self),
      temporalResolutionType(N_TEMPORARL_RESOLUTION_TYPES, CNOID_GETTEXT_DOMAIN_NAME),
      preDynamicsFunctions(this),
      midDynamicsFunctions(this),
      postDynamicsFunctions(this),
      recordingMode(SimulatorItem::N_RECORDING_MODES, CNOID_GETTEXT_DOMAIN_NAME),
      timeRangeMode(SimulatorItem::N_TIME_RANGE_MODES, CNOID_GETTEXT_DOMAIN_NAME),
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
    flushTimer.sigTimeout().connect([&](){ flushResults(); });

    recordingMode.setSymbol(SimulatorItem::REC_FULL, N_("full"));
    recordingMode.setSymbol(SimulatorItem::REC_TAIL, N_("tail"));
    recordingMode.setSymbol(SimulatorItem::REC_NONE, N_("off"));
    recordingMode.select(SimulatorItem::REC_FULL);

    timeRangeMode.setSymbol(SimulatorItem::TR_UNLIMITED, N_("Unlimited"));
    timeRangeMode.setSymbol(SimulatorItem::TR_ACTIVE_CONTROL, N_("Active control period"));
    timeRangeMode.setSymbol(SimulatorItem::TR_SPECIFIED, N_("Specified time"));
    timeRangeMode.setSymbol(SimulatorItem::TR_TIMEBAR, N_("Time bar range"));
    timeRangeMode.select(SimulatorItem::TR_UNLIMITED);

    specifiedTimeLength = 180.0; // 3 min.
    useControllerThreadsProperty = true;
    isAllLinkPositionOutputMode = true;
    isDeviceStateOutputEnabled = true;
    isDoingSimulationLoop = false;
    isRealtimeSyncMode = true;
    recordCollisionData = false;

    timeBar = TimeBar::instance();
    itemTreeView = ItemTreeView::instance();
}


SimulatorItem::SimulatorItem(const SimulatorItem& org)
    : Item(org)
{
    impl = new SimulatorItemImpl(this, *org.impl);
}


SimulatorItemImpl::SimulatorItemImpl(SimulatorItem* self, const SimulatorItemImpl& org)
    : SimulatorItemImpl(self)
{
    temporalResolutionType = org.temporalResolutionType;
    timeStepProperty = org.timeStepProperty;
    frameRateProperty = org.frameRateProperty;

    recordingMode = org.recordingMode;
    timeRangeMode = org.timeRangeMode;

    specifiedTimeLength = org.specifiedTimeLength;
    useControllerThreadsProperty = org.useControllerThreadsProperty;
    isAllLinkPositionOutputMode = org.isAllLinkPositionOutputMode;
    isDeviceStateOutputEnabled = org.isDeviceStateOutputEnabled;
    isRealtimeSyncMode = org.isRealtimeSyncMode;
    recordCollisionData = org.recordCollisionData;
    controllerOptionString_ = org.controllerOptionString_;
}
    

SimulatorItem::~SimulatorItem()
{
    impl->stopSimulation(true);
    delete impl;
}


SimulatorItemImpl::~SimulatorItemImpl()
{
    aboutToQuitConnection.disconnect();
}


void SimulatorItem::onPositionChanged()
{
    impl->worldItem = findOwnerItem<WorldItem>();
}


void SimulatorItem::onDisconnectedFromRoot()
{
    impl->stopSimulation(true);
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


Selection SimulatorItem::recordingMode() const
{
    return impl->recordingMode;
}


bool SimulatorItem::isRecordingEnabled() const
{
    return impl->isRecordingEnabled;
}


void SimulatorItem::setTimeRangeMode(int selection)
{
    impl->timeRangeMode.select(selection);
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

void SimulatorItem::setSpecifiedRecordingTimeLength(double length)
{
    impl->setSpecifiedRecordingTimeLength(length);
}

void SimulatorItemImpl::setSpecifiedRecordingTimeLength(double length)
{
    specifiedTimeLength = length;
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


void SimulatorItem::setSelfCollisionEnabled(bool on)
{

}


bool SimulatorItem::isSelfCollisionEnabled() const
{
    return false;
}


const std::string& SimulatorItem::controllerOptionString() const
{
    return impl->controllerOptionString_;
}


/*
  Extract body items, controller items which are not associated with (not under) a body item,
  and simulation script items which are not under another simulator item
*/
void SimulatorItemImpl::findTargetItems
(Item* item, bool isUnderBodyItem, ItemList<Item>& out_targetItems)
{
    if(dynamic_cast<BodyItem*>(item)){
        out_targetItems.push_back(item);
        isUnderBodyItem = true;

    } else if(!isUnderBodyItem && dynamic_cast<ControllerItem*>(item)){
        out_targetItems.push_back(item);

    } else if(SimulationScriptItem* scriptItem = dynamic_cast<SimulationScriptItem*>(item)){
        if(itemTreeView->isItemChecked(scriptItem)){
            if(scriptItem->executionTiming() == SimulationScriptItem::BEFORE_INITIALIZATION){
                scriptItem->executeAsSimulationScript();
            } else {
                out_targetItems.push_back(item);
            }
        }
    }
    SimulatorItem* simulatorItem = dynamic_cast<SimulatorItem*>(item);
    if(!simulatorItem || (simulatorItem == self)){
        for(Item* childItem = item->childItem(); childItem; childItem = childItem->nextItem()){
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
        vector<FunctionInfo>::iterator p = functions.end();
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
    
    
void SimulatorItemImpl::clearSimulation()
{
    allSimBodies.clear();
    simBodiesWithBody.clear();;
    activeSimBodies.clear();
    needToUpdateSimBodyLists = true;

    preDynamicsFunctions.clear();
    midDynamicsFunctions.clear();
    postDynamicsFunctions.clear();

    subSimulatorItems.clear();

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


bool SimulatorItemImpl::startSimulation(bool doReset)
{
    this->doReset = doReset;
    
    stopSimulation(true);

    if(!worldItem){
        mv->putln(format(_("{} must be in a WorldItem to do simulation."),
                         self->name()),
                  MessageView::ERROR);
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
    bodyMotionEngines.clear();

    for(size_t i=0; i < targetItems.size(); ++i){

        if(BodyItem* bodyItem = dynamic_cast<BodyItem*>(targetItems.get(i))){
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
                          MessageView::WARNING);
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
            
        } else if(ControllerItem* controller = dynamic_cast<ControllerItem*>(targetItems.get(i))){
            // ControllerItem which is not associated with a body
            SimulationBodyPtr simBody = new SimulationBody(nullptr);
            if(simBody->impl->initialize(this, controller)){
                allSimBodies.push_back(simBody);
            }
        } else if(SimulationScriptItem* script = dynamic_cast<SimulationScriptItem*>(targetItems.get(i))){
            SimulationBodyPtr simBody = new SimulationBody(nullptr);
            ControllerItemPtr scriptControllerItem = new ScriptControllerItem(script);
            if(simBody->impl->initialize(this, scriptControllerItem)){
                allSimBodies.push_back(simBody);
            }
        }
    }

    numBufferedFrames = 1;
    
    if(isRecordingEnabled && recordCollisionData){
        collisionPairsBuf.clear();
        string collisionSeqName = self->name() + "-collisions";
        CollisionSeqItem* collisionSeqItem = worldItem->findChildItem<CollisionSeqItem>(collisionSeqName);
        if(!collisionSeqItem){
            collisionSeqItem = new CollisionSeqItem();
            collisionSeqItem->setTemporal();
            collisionSeqItem->setName(collisionSeqName);
            worldItem->addChildItem(collisionSeqItem);

            addCollisionSeqEngine(collisionSeqItem);
        }
        collisionSeq = collisionSeqItem->collisionSeq();
        collisionSeq->setFrameRate(worldFrameRate);
        collisionSeq->setNumParts(1);
        collisionSeq->setNumFrames(1);
        CollisionSeq::Frame frame0 = collisionSeq->frame(0);
        frame0[0]  = std::make_shared<CollisionLinkPairList>();
    }

    extForceFunctionId = stdx::nullopt;
    virtualElasticStringFunctionId = stdx::nullopt;

    cloneMap.replacePendingObjects();
    
    bool result = self->initializeSimulation(simBodiesWithBody);

    if(result){

        frameAtLastBufferWriting = 0;
        isDoingSimulationLoop = true;
        isWaitingForSimulationToStop = false;
        stopRequested = false;
        pauseRequested = false;

        ringBufferSize = std::numeric_limits<int>::max();
        
        if(timeRangeMode.is(SimulatorItem::TR_SPECIFIED)){
            maxFrame = specifiedTimeLength / worldTimeStep_;
        } else if(timeRangeMode.is(SimulatorItem::TR_TIMEBAR)){
            maxFrame = TimeBar::instance()->maxTime() / worldTimeStep_;
        } else if(isRingBufferMode){
            maxFrame = std::numeric_limits<int>::max();
            ringBufferSize = specifiedTimeLength / worldTimeStep_;
        } else {
            maxFrame = std::numeric_limits<int>::max();
        }

        subSimulatorItems.extractAssociatedItems(self);
        ItemList<SubSimulatorItem>::iterator p = subSimulatorItems.begin();
        while(p != subSimulatorItems.end()){
            SubSimulatorItem* item = *p;
            bool initialized = false;
            if(item->isEnabled()){
                mv->putln(format(_("SubSimulatorItem \"{}\" has been detected."), item->name()));
                if(item->initializeSimulation(self)){
                    initialized = true;
                } else {
                    mv->putln(format(_("The initialization of \"{}\" failed."),
                                     item->name()),
                              MessageView::WARNING);
                }
            } else {
                mv->putln(format(_("SubSimulatorItem \"{}\" is disabled."), item->name()));
            }
            if(initialized){
                ++p;
            } else {
                p = subSimulatorItems.erase(p);
            }
        }

        for(size_t i=0; i < allSimBodies.size(); ++i){
            SimulationBodyImpl* simBodyImpl = allSimBodies[i]->impl;
            Body* body = simBodyImpl->body_;
            vector<ControllerItemPtr>& controllers = simBodyImpl->controllers;
            vector<ControllerItemPtr>::iterator iter = controllers.begin();
            while(iter != controllers.end()){
                ControllerItem* controller = *iter;
                bool ready = false;
                controller->setSimulatorItem(self);
                if(body){
                    ready = controller->start();
                    if(!ready){
                        mv->putln(format(_("{0} for {1} failed to start."),
                                         controller->name(), simBodyImpl->bodyItem->name()),
                                  MessageView::WARNING);
                    }
                } else {
                    ready = controller->start();
                    if(!ready){
                        mv->putln(format(_("{} failed to start."),
                                         controller->name()),
                                  MessageView::WARNING);
                    }
                }
                if(ready){
                    ++iter;
                } else {
                    controller->setSimulatorItem(nullptr);
                    iter = controllers.erase(iter);
                }
            }
        }

        updateSimBodyLists();

        doCheckContinue = timeRangeMode.is(SimulatorItem::TR_ACTIVE_CONTROL) && !activeControllers.empty();
            
        useControllerThreads = useControllerThreadsProperty;
        if(useControllerThreads){
            isExitingControlLoopRequested = false;
            isControlRequested = false;
            isControlFinished = false;
            controlThread = std::thread([&](){ concurrentControlLoop(); });
        }

        aboutToQuitConnection.disconnect();
        aboutToQuitConnection = cnoid::sigAboutToQuit().connect(
            [&](){ stopSimulation(true); });

        worldLogFileItem = nullptr;
        ItemList<WorldLogFileItem> worldLogFileItems;
        worldLogFileItems.extractChildItems(self); // Check child items first
        if(worldLogFileItems.empty()){
            worldLogFileItems.extractChildItems(worldItem); // Check items in the world secondly
        }
        worldLogFileItem = worldLogFileItems.toSingle(true);
        if(worldLogFileItem){
            if(worldLogFileItem->logFile().empty()){
                worldLogFileItem = nullptr;
            } else {
                mv->putln(format(_("WorldLogFileItem \"{0}\" has been detected. "
                                   "A simulation result is recoreded to \"{1}\"."),
                                 worldLogFileItem->name(), worldLogFileItem->logFile()));

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
            fillLevelId = timeBar->startFillLevelUpdate();
        }
        if(!timeBar->isDoingPlayback()){
            timeBar->setTime(0.0);
            timeBar->startPlayback();
        }

#ifdef ENABLE_SIMULATION_PROFILING
        vector<string> profilingNames;
        self->getProfilingNames(profilingNames);
        profilingNames.push_back("Controller calculation time");
        for(size_t i=0; i < activeControllers.size(); ++i){
            vector<string> profilingNames0;
            activeControllers[i]->getProfilingNames(profilingNames0);
            std::copy(profilingNames0.begin(),profilingNames0.end(),std::back_inserter(profilingNames));
        }
        profilingNames.push_back("Total computation time");
        int n = profilingNames.size();
        SceneView* view = ViewManager::findView<SceneView>("Simulation Scene");
        if(!view)
            view = SceneView::instance();
        sw = view->sceneWidget();
        sw->profilingNames.resize(n);
        copy(profilingNames.begin(), profilingNames.end(), sw->profilingNames.begin());
        simProfilingBuf.resizeColumn(n);

        //simProfilingSeq = std::make_shared<MultiValueSeq>();
        simProfilingSeq.setFrameRate(worldFrameRate);
        simProfilingSeq.setNumParts(n);
        simProfilingSeq.setNumFrames(0);
#endif

        flushResults();
        start();
        flushTimer.start(1000.0 / timeBar->playbackFrameRate());

        mv->notify(format(_("Simulation by {} has started."), self->name()));

        sigSimulationStarted();
    }

    return result;
}


CloneMap& SimulatorItem::cloneMap()
{
    return impl->cloneMap;
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
    BodyItemToSimBodyMap::iterator p = impl->simBodyMap.find(bodyItem);
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
    return 0;
}


// Simulation loop
void SimulatorItemImpl::run()
{
    self->initializeSimulationThread();

    double elapsedTime = 0.0;
    QElapsedTimer timer;
    timer.start();

#ifdef ENABLE_SIMULATION_PROFILING
    QElapsedTimer oneStepTimer;
#endif

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
#ifdef ENABLE_SIMULATION_PROFILING
                oneStepTimer.start();
#endif
                if(!stepSimulationMain() || stopRequested || frame >= maxFrame){
                    break;
                }
#ifdef ENABLE_SIMULATION_PROFILING
                double oneStepTime = oneStepTimer.nsecsElapsed();
                vector<double> profilingTimes;
                self->getProfilingTimes(profilingTimes);
                Deque2D<double>::Row buf = simProfilingBuf.append();
                int i=0;
                for(; i<profilingTimes.size(); i++){
                    buf[i] = profilingTimes[i] * 1.0e9;
                }
                buf[i++] = controllerTime;
                for(size_t k=0; k < activeControllers.size(); k++){
                    profilingTimes.clear();
                    activeControllers[k]->getProfilingTimes(profilingTimes);
                    for(int j=0; j<profilingTimes.size(); j++){
                        buf[i++] = profilingTimes[j] * 1.0e9;
                    }
                }
                buf[i] = oneStepTime;
#endif
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
#ifdef ENABLE_SIMULATION_PROFILING
                oneStepTimer.start();
#endif
                if(!stepSimulationMain() || stopRequested || frame++ >= maxFrame){
                    break;
                }
#ifdef ENABLE_SIMULATION_PROFILING
                double oneStepTime = oneStepTimer.nsecsElapsed();
                vector<double> profilingTimes;
                self->getProfilingTimes(profilingTimes);
                Deque2D<double>::Row buf = simProfilingBuf.append();
                int i=0;
                for(; i<profilingTimes.size(); i++){
                    buf[i] = profilingTimes[i] * 1.0e9;
                }
                buf[i++] = controllerTime;
                for(size_t k=0; k < activeControllers.size(); k++){
                    profilingTimes.clear();
                    activeControllers[k]->getProfilingTimes(profilingTimes);
                    for(int j=0; j<profilingTimes.size(); j++){
                        buf[i++] = profilingTimes[j] * 1.0e9;
                    }
                }
                buf[i] = oneStepTime;
#endif
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
        {
            std::lock_guard<std::mutex> lock(controlMutex);
            isExitingControlLoopRequested = true;
        }
        controlCondition.notify_all();
        controlThread.join();
    }

    if(!isWaitingForSimulationToStop){
        callLater([&](){ onSimulationLoopStopped(); });
    }

    self->finalizeSimulationThread();
}


void SimulatorItemImpl::updateSimBodyLists()
{
    activeSimBodies.clear();
    activeControllers.clear();
    hasActiveFreeBodies = false;
    
    for(size_t i=0; i < allSimBodies.size(); ++i){
        SimulationBody* simBody = allSimBodies[i];
        SimulationBodyImpl* simBodyImpl = simBody->impl;
        vector<ControllerItemPtr>& controllers = simBodyImpl->controllers;
        if(simBodyImpl->isActive){
            activeSimBodies.push_back(simBody);
            if(controllers.empty()){
                hasActiveFreeBodies = true;
            }
        }
        for(size_t j=0; j < controllers.size(); ++j){
            activeControllers.push_back(controllers[j]);
       }
    }

    needToUpdateSimBodyLists = false;
}


bool SimulatorItemImpl::stepSimulationMain()
{
    currentFrame++;

    if(needToUpdateSimBodyLists){
        updateSimBodyLists();
    }
    
    bool doContinue = !doCheckContinue;

    preDynamicsFunctions.call();

    if(useControllerThreads){
#ifdef ENABLE_SIMULATION_PROFILING
        controllerTime = 0.0;
#endif
        if(activeControllers.empty()){
            isControlFinished = true;
        } else {
#ifdef ENABLE_SIMULATION_PROFILING
            timer.start();
#endif
            for(size_t i=0; i < activeControllers.size(); ++i){
                activeControllers[i]->input();
            }
#ifdef ENABLE_SIMULATION_PROFILING
            controllerTime += timer.nsecsElapsed();
#endif
            {
                std::lock_guard<std::mutex> lock(controlMutex);                
                isControlRequested = true;
            }
            controlCondition.notify_all();
        }
    } else {
#ifdef ENABLE_SIMULATION_PROFILING
        controllerTime = 0.0;
        timer.start();
#endif
        for(size_t i=0; i < activeControllers.size(); ++i){
            ControllerItem* controller = activeControllers[i];
            controller->input();
            doContinue |= controller->control();
            if(controller->isImmediateMode()){
                controller->output();
            }
        }
#ifdef ENABLE_SIMULATION_PROFILING
        controllerTime += timer.nsecsElapsed();
#endif
    }

    midDynamicsFunctions.call();

    self->stepSimulation(activeSimBodies);

    shared_ptr<CollisionLinkPairList> collisionPairs;
    if(isRecordingEnabled && recordCollisionData){
        collisionPairs = self->getCollisions();
    }

    if(useControllerThreads){
        {
            std::unique_lock<std::mutex> lock(controlMutex);
            while(!isControlFinished){
                controlCondition.wait(lock);
            }
        }
        isControlFinished = false;
        doContinue |= isControlToBeContinued;
    }

    postDynamicsFunctions.call();

    {
        resultBufMutex.lock();

        ++numBufferedFrames;
        for(size_t i=0; i < activeSimBodies.size(); ++i){
            activeSimBodies[i]->bufferResults();
        }
        collisionPairsBuf.push_back(collisionPairs);
        frameAtLastBufferWriting = currentFrame;

        resultBufMutex.unlock();
    }

    if(useControllerThreads){
#ifdef ENABLE_SIMULATION_PROFILING
        timer.start();
#endif
        for(size_t i=0; i < activeControllers.size(); ++i){
            activeControllers[i]->output();
        }
#ifdef ENABLE_SIMULATION_PROFILING
        controllerTime += timer.nsecsElapsed();
#endif
    } else {
#ifdef ENABLE_SIMULATION_PROFILING
        timer.start();
#endif
        for(size_t i=0; i < activeControllers.size(); ++i){
            ControllerItem* controller = activeControllers[i];
            if(!controller->isImmediateMode()){
                controller->output(); 
            }
        }
#ifdef ENABLE_SIMULATION_PROFILING
        controllerTime += timer.nsecsElapsed();
#endif
    }

    return doContinue;
}


void SimulatorItemImpl::concurrentControlLoop()
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

        bool doContinue = false;
#ifdef ENABLE_SIMULATION_PROFILING
        timer.start();
#endif
        for(size_t i=0; i < activeControllers.size(); ++i){
            doContinue |= activeControllers[i]->control();
        }
#ifdef ENABLE_SIMULATION_PROFILING
        controllerTime += timer.nsecsElapsed();
#endif
        
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


void SimulatorItemImpl::flushResults()
{
    resultBufMutex.lock();

    if(worldLogFileItem){
        if(numBufferedFrames > 0){
            int firstFrame = frameAtLastBufferWriting - (numBufferedFrames - 1);
            for(int bufFrame = 0; bufFrame < numBufferedFrames; ++bufFrame){
                double time = (firstFrame + bufFrame) * worldTimeStep_;
                while(time >= nextLogTime){
                    worldLogFileItem->beginFrameOutput(time);
                    for(size_t i=0; i < activeSimBodies.size(); ++i){
                        activeSimBodies[i]->impl->flushResultsToWorldLogFile(bufFrame);
                    }
                    worldLogFileItem->endFrameOutput();
                    nextLogTime = ++nextLogFrame * logTimeStep;
                }
            }
        }
    }
    
    for(size_t i=0; i < activeSimBodies.size(); ++i){
        activeSimBodies[i]->flushResults();
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

#ifdef ENABLE_SIMULATION_PROFILING
    offsetChanged = false;
    for(int i=0 ; i < simProfilingBuf.rowSize(); i++){
        Deque2D<double>::Row buf = simProfilingBuf.row(i);
        if(simProfilingSeq.numFrames() >= ringBufferSize){
            simProfilingSeq.popFrontFrame();
            offsetChanged = true;
        }
        std::copy(buf.begin(), buf.end(), simProfilingSeq.appendFrame().begin());
    }
    if(offsetChanged){
        simProfilingSeq.setOffsetTimeFrame(currentFrame + 1 - simProfilingSeq.numFrames());
    }
    simProfilingBuf.resizeRow(0);
#endif

    int frame = frameAtLastBufferWriting;
    
    numBufferedFrames = 0;
    
    resultBufMutex.unlock();

    if(isRecordingEnabled){
        double fillLevel = frame / worldFrameRate;
        timeBar->updateFillLevel(fillLevelId, fillLevel);
    } else {
        const double time = frame / worldFrameRate;
        for(size_t i=0; i < activeSimBodies.size(); ++i){
            activeSimBodies[i]->impl->notifyResults(time);
        }
        timeBar->setTime(time);
    }
}


void SimulatorItem::pauseSimulation()
{
    impl->pauseSimulation();
}


void SimulatorItemImpl::pauseSimulation()
{
    pauseRequested = true;
}


void SimulatorItem::restartSimulation()
{
    impl->restartSimulation();
}


void SimulatorItemImpl::restartSimulation()
{
    pauseRequested = false;
}


void SimulatorItem::stopSimulation()
{
    impl->stopSimulation(false);
}


void SimulatorItemImpl::stopSimulation(bool doSync)
{
    if(isDoingSimulationLoop){
        if(doSync){
            isWaitingForSimulationToStop = true;
        }
        stopRequested = true;
        
        if(doSync){
            wait();
            isWaitingForSimulationToStop = false;
            onSimulationLoopStopped();
        }
    }
    aboutToQuitConnection.disconnect();
}


void SimulatorItem::finalizeSimulation()
{

}


void SimulatorItemImpl::onSimulationLoopStopped()
{
    flushTimer.stop();
    
    for(size_t i=0; i < allSimBodies.size(); ++i){
        vector<ControllerItemPtr>& controllers = allSimBodies[i]->impl->controllers;
        for(size_t j=0; j < controllers.size(); ++j){
            ControllerItem* controller = controllers[j];
            controller->stop();
            controller->setSimulatorItem(nullptr);
        }
    }
    self->finalizeSimulation();

    for(size_t i=0; i < subSimulatorItems.size(); ++i){
        subSimulatorItems[i]->finalizeSimulation();
    }

    flushResults();

    if(isRecordingEnabled){
        timeBar->stopFillLevelUpdate(fillLevelId);
    }

    mv->notify(format(_("Simulation by {0} has finished at {1} [s]."), self->name(), finishTime));

    if(finishTime > 0.0){
        mv->putln(format(_("Computation time is {0} [s], computation time / simulation time = {1}."),
                         actualSimulationTime, (actualSimulationTime / finishTime)));
    }

    clearSimulation();

    sigSimulationFinished();
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


double SimulatorItemImpl::currentTime() const
{
    return currentFrame / worldFrameRate;
}


int SimulatorItem::simulationFrame() const
{
    QMutexLocker locker(&impl->resultBufMutex);
    return impl->frameAtLastBufferWriting;
}


double SimulatorItem::simulationTime() const
{
    QMutexLocker locker(&impl->resultBufMutex);
    return impl->frameAtLastBufferWriting / impl->worldFrameRate;
}


double SimulatorItemImpl::timeStep() const
{
    return worldTimeStep_;
}


Body* SimulatorItemImpl::body()
{
    return 0;
}


std::string SimulatorItemImpl::optionString() const
{
    return controllerOptionString_;
}


std::ostream& SimulatorItemImpl::os() const
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


SignalProxy<void()> SimulatorItem::sigSimulationFinished()
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


void SimulatorItemImpl::setExternalForce(BodyItem* bodyItem, Link* link, const Vector3& point, const Vector3& f, double time)
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


void SimulatorItemImpl::doSetExternalForce()
{
    std::lock_guard<std::mutex> lock(extForceMutex);
    extForceInfo.link->addExternalForce(extForceInfo.f, extForceInfo.point);
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


void SimulatorItemImpl::setVirtualElasticString
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


void SimulatorItemImpl::setVirtualElasticStringForce()
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


void SimulatorItem::setForcedPosition(BodyItem* bodyItem, const Position& T)
{

}


bool SimulatorItem::isForcedPositionActiveFor(BodyItem* bodyItem) const
{
    return false;
}


void SimulatorItem::clearForcedPositions()
{

}


void SimulatorItemImpl::onRealtimeSyncChanged(bool on)
{
    isRealtimeSyncMode = on;
}


/**
   This function may be overridden.
*/
bool SimulatorItemImpl::onAllLinkPositionOutputModeChanged(bool on)
{
    self->setAllLinkPositionOutputMode(on);
    return (isAllLinkPositionOutputMode == on);
}


void SimulatorItem::doPutProperties(PutPropertyFunction& putProperty)
{
    impl->doPutProperties(putProperty);
}


void SimulatorItemImpl::doPutProperties(PutPropertyFunction& putProperty)
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
    putProperty(_("Time length"), specifiedTimeLength,
                [&](double length){ setSpecifiedRecordingTimeLength(length); return true; });
    putProperty(_("Recording"), recordingMode,
                [&](int index){ return recordingMode.select(index); });
    putProperty(_("All link positions"), isAllLinkPositionOutputMode,
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


bool SimulatorItemImpl::store(Archive& archive)
{
    if(temporalResolutionType.is(RESOLUTION_TIMESTEP)){
        archive.write("timeStep", timeStepProperty.string());
    } else if(temporalResolutionType.is(RESOLUTION_FRAMERATE)){
        archive.write("frameRate", frameRateProperty);
    }
    archive.write("realtimeSync", isRealtimeSyncMode);
    archive.write("recording", recordingMode.selectedSymbol(), DOUBLE_QUOTED);
    archive.write("timeRangeMode", timeRangeMode.selectedSymbol(), DOUBLE_QUOTED);
    archive.write("timeLength", specifiedTimeLength);
    archive.write("allLinkPositionOutputMode", isAllLinkPositionOutputMode);
    archive.write("deviceStateOutput", isDeviceStateOutputEnabled);
    archive.write("controllerThreads", useControllerThreadsProperty);
    archive.write("recordCollisionData", recordCollisionData);
    archive.write("controllerOptions", controllerOptionString_, DOUBLE_QUOTED);

    ListingPtr idseq = new Listing();
    idseq->setFlowStyle(true);
    for(size_t i=0; i < bodyMotionEngines.size(); ++i){
        BodyMotionEnginePtr engine = bodyMotionEngines[i];
        ValueNodePtr id = archive.getItemId(engine->motionItem());
        if(id){
            idseq->append(id);
        }
    }
    if(!idseq->empty()){
        archive.insert("motionItems", idseq);
    }

    if(collisionSeqEngine)
    {
        ValueNodePtr id = archive.getItemId(collisionSeqEngine->collisionSeqItem());
        if(id){
            archive.insert("collisionSeqItem", id);
        }
    }
    return true;
}


bool SimulatorItem::restore(const Archive& archive)
{
    return impl->restore(archive);
}


bool SimulatorItemImpl::restore(const Archive& archive)
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
    
    if(archive.read("onlyActiveControlPeriod", boolValue) && boolValue){
        timeRangeMode.select(SimulatorItem::TR_ACTIVE_CONTROL);
    } else if(archive.read("timeRangeMode", symbol)){
        if(!timeRangeMode.select(symbol)){
            if(symbol == "Specified period"){
                timeRangeMode.select(SimulatorItem::TR_SPECIFIED);
            } else if(symbol == "TimeBar range"){
                timeRangeMode.select(SimulatorItem::TR_TIMEBAR);
            }
        }
    }

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
    archive.read("timeLength", specifiedTimeLength);
    self->setAllLinkPositionOutputMode(archive.get("allLinkPositionOutputMode", isAllLinkPositionOutputMode));
    archive.read("deviceStateOutput", isDeviceStateOutputEnabled);
    archive.read("recordCollisionData", recordCollisionData);
    archive.read("controllerThreads", useControllerThreadsProperty);
    archive.read("controllerOptions", controllerOptionString_);

    archive.addPostProcess([&](){ restoreBodyMotionEngines(archive); });
    
    return true;
}


void SimulatorItemImpl::restoreBodyMotionEngines(const Archive& archive)
{
    bodyMotionEngines.clear();

    const Listing& idseq = *archive.findListing("motionItems");
    if(idseq.isValid()){
        for(int i=0; i < idseq.size(); ++i){
            ValueNode* id = idseq.at(i);
            if(id){
                BodyMotionItem* motionItem = dynamic_cast<BodyMotionItem*>(archive.findItem(id));
                if(motionItem){
                    addBodyMotionEngine(motionItem);
                }
            }
        }
    }

    collisionSeqEngine = nullptr;
    ValueNode* id;
    id = archive.find("collisionSeqItem");
    if(id->isValid()){
        CollisionSeqItem* collisionSeqItem = dynamic_cast<CollisionSeqItem*>(archive.findItem(id));
        if(collisionSeqItem){
            addCollisionSeqEngine(collisionSeqItem);
        }
    }
}


void SimulatorItemImpl::addBodyMotionEngine(BodyMotionItem* motionItem)
{
    BodyItem* bodyItem = motionItem->findOwnerItem<BodyItem>();
    if(bodyItem){
        bodyMotionEngines.push_back(new BodyMotionEngine(bodyItem, motionItem));
    }
}


void SimulatorItemImpl::addCollisionSeqEngine(CollisionSeqItem* collisionSeqItem)
{
    if(worldItem){
        collisionSeqEngine = new CollisionSeqEngine(worldItem, collisionSeqItem);
    }
}


bool SimulatorItemImpl::setPlaybackTime(double time)
{
    bool processed = false;
    if(!bodyMotionEngines.empty()){
        for(size_t i=0; i < bodyMotionEngines.size(); ++i){
            processed |= bodyMotionEngines[i]->onTimeChanged(time);
        }
    } else if(worldLogFileItem){
        processed |= worldLogFileItem->recallStateAtTime(time);
    }
    if(collisionSeqEngine){
        processed |= collisionSeqEngine->onTimeChanged(time);
    }

#ifdef ENABLE_SIMULATION_PROFILING
    const int numFrames = simProfilingSeq.numFrames();
    if(numFrames > 0){
        const int frame = simProfilingSeq.frameOfTime(time);
        const int clampedFrame = simProfilingSeq.clampFrameIndex(frame);
        const MultiValueSeq::Frame profilingTimes = simProfilingSeq.frame(clampedFrame);
        sw->profilingTimes.clear();
        for(int i=0; i<profilingTimes.size(); i++)
            sw->profilingTimes.push_back(profilingTimes[i]);
        sw->worldTimeStep = worldTimeStep_ * 1.0e9;
    }
#endif

    return processed;
}


bool SimulatedMotionEngineManager::setTime(double time)
{
    bool isActive = false;
    for(size_t i=0; i < simulatorItems.size(); ++i){
        isActive |= simulatorItems[i]->impl->setPlaybackTime(time);
    }
    return isActive;
}

#ifdef ENABLE_SIMULATION_PROFILING
void SimulatorItem::getProfilingNames(vector<string>& profilingNames)
{

}


void SimulatorItem::getProfilingTimes(vector<double>& profilingToimes)
{

}
#endif
