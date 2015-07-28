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
#include "CollisionSeq.h"
#include "CollisionSeqItem.h"
#include "CollisionSeqEngine.h"
#include <cnoid/AppUtil>
#include <cnoid/ExtensionManager>
#include <cnoid/TimeBar>
#include <cnoid/ItemTreeView>
#include <cnoid/MessageView>
#include <cnoid/LazyCaller>
#include <cnoid/Archive>
#include <cnoid/MultiDeviceStateSeq>
#include <cnoid/Deque2D>
#include <cnoid/ConnectionSet>
#include <cnoid/Sleep>
#include <cnoid/Timer>
#include <QThread>
#include <QMutex>
#include <boost/thread.hpp>
#include <boost/dynamic_bitset.hpp>
#include <boost/bind.hpp>

#if QT_VERSION >= 0x040700
#include <QElapsedTimer>
#else
#include <QTime>
typedef QTime QElapsedTimer;
#endif
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using boost::format;

namespace {

typedef Deque2D<SE3, Eigen::aligned_allocator<SE3> > MultiSE3Deque;

class ControllerTarget : public ControllerItem::Target
{
    SimulatorItemImpl* simImpl;
    BodyPtr body_;
public:
    ControllerTarget() : simImpl(0) { }
    void setSimImpl(SimulatorItemImpl* simImpl);
    void setSimBodyImpl(SimulationBodyImpl* simBodyImpl);
    virtual Body* body();
    virtual double worldTimeStep() const;
    virtual double currentTime() const;
};

}

namespace cnoid {

class SimulationBodyImpl
{
public:
    BodyPtr body;
    BodyItemPtr bodyItem;
    vector<ControllerItemPtr> controllers;
    ControllerTarget controllerTarget;
    double frameRate;
    SimulatorItemImpl* simImpl;

    bool doStoreResult;
    bool areShapesCloned;

    Deque2D<double> jointPosBuf;
    MultiSE3Deque linkPosBuf;

    BodyMotionPtr motion;
    MultiValueSeqPtr jointPosResult;
    MultiSE3SeqPtr linkPosResult;
    MultiSE3SeqItemPtr linkPosResultItem;

    vector<Device*> devicesToNotifyResult;
    ConnectionSet deviceStateConnections;
    boost::dynamic_bitset<> deviceStateChangeFlag;
    Deque2D<DeviceStatePtr> deviceStateBuf;
    vector<DeviceStatePtr> prevFlushedDeviceStateInDirectMode;
    MultiDeviceStateSeqPtr deviceStateResult;

    SimulationBodyImpl(const BodyPtr& body);
    void findControlSrcItems(Item* item, vector<Item*>& io_items, bool doPickCheckedItems = false);
    bool initialize(SimulatorItemImpl* simImpl, BodyItem* bodyItem);
    bool initialize(SimulatorItemImpl* simImpl, ControllerItem* controllerItem);
    void cloneShapesOnce();
    void setupResultMotion(
        SimulatorItemImpl* simImpl, Item* ownerItem, const string& simulatedMotionName);
    void setupDeviceStateRecording();
    void setInitialStateOfBodyMotion(const BodyMotionPtr& bodyMotion);
    void onDeviceStateChanged(int deviceIndex);
    void storeResult();
    void flushResult();
    void notifyResult();
};


class SimulatorItemImpl : QThread
{
public:
    SimulatorItemImpl(SimulatorItem* self);
    SimulatorItemImpl(SimulatorItem* self, const SimulatorItemImpl& org);
    ~SimulatorItemImpl();
            
    SimulatorItem* self;

    vector<SimulationBodyPtr> allSimBodies;
    vector<SimulationBody*> simBodiesWithBody;
    vector<SimulationBody*> activeSimBodies;

    ControllerTarget controllerTarget;
    vector<ControllerItem*> activeControllers;
    boost::thread controlThread;
    boost::condition_variable controlCondition;
    boost::mutex controlMutex;
    bool isExitingControlLoopRequested;
    bool isControlRequested;
    bool isControlFinished;
    bool isControlToBeContinued;
        
    vector<SimulationBodyImpl*> simBodyImplsToNotifyResult;
    ItemList<SubSimulatorItem> subSimulatorItems;
    vector< boost::function<void()> > preDynamicsFunctions;
    vector< boost::function<void()> > midDynamicsFunctions;
    vector< boost::function<void()> > postDynamicsFunctions;
        
    CollisionDetectorPtr collisionDetector;
        
    int currentFrame;
    double worldFrameRate;
    double worldTimeStep;
    int frameAtLastBufferWriting;
    Timer flushTimer;

    Selection recordingMode;
    bool isRecordingEnabled;
    bool isRingBufferMode;
    bool isActiveControlPeriodOnlyMode;
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

    Selection timeRangeMode;
    double specifiedTimeLength;
    int maxFrame;
    int ringBufferSize;

    TimeBar* timeBar;
    int fillLevelId;
    QMutex mutex;
    double actualSimulationTime;
    double finishTime;
    MessageView* mv;
    ostream& os;

    bool doReset;
    bool isWaitingForSimulationToStop;
    Signal<void()> sigSimulationFinished_;

    vector<BodyMotionEnginePtr> bodyMotionEngines;
    CollisionSeqEnginePtr collisionSeqEngine;

    Connection aboutToQuitConnection;

    SgCloneMap sgCloneMap;
        
    ItemTreeView* itemTreeView;

    CollisionSeqPtr collisionSeq;
    deque<CollisionLinkPairListPtr> collisionPairsBuf;

    double currentTime() const;
    ControllerItem* createBodyMotionController(BodyItem* bodyItem, BodyMotionItem* bodyMotionItem);
    void findTargetItems(Item* item, bool isUnderBodyItem, ItemList<Item>& out_targetItems);
    bool startSimulation(bool doReset);
    virtual void run();
    void onSimulationLoopStarted();
    void updateSimBodyLists();
    bool stepSimulationMain();
    void concurrentControlLoop();
    void flushResult();
    void stopSimulation(bool doSync);
    void pauseSimulation();
    void restartSimulation();
    void onSimulationLoopStopped();
    bool onRealtimeSyncChanged(bool on);
    bool onAllLinkPositionOutputModeChanged(bool on);
    bool setSpecifiedRecordingTimeLength(double length);
    bool store(Archive& archive);
    bool restore(const Archive& archive);
    void restoreBodyMotionEngines(const Archive& archive);
    void addBodyMotionEngine(BodyMotionItem* motionItem);
    bool setPlaybackTime(double time);
    void addCollisionSeqEngine(CollisionSeqItem* collisionSeqItem);
};


class SimulatedMotionEngineManager
{
public:
    ItemList<SimulatorItem> simulatorItems;
    double currentTime;

    ScopedConnection selectionOrTreeChangedConnection;
    ScopedConnection timeChangeConnection;

    SimulatedMotionEngineManager(){

        selectionOrTreeChangedConnection.reset(
            ItemTreeView::instance()->sigSelectionOrTreeChanged().connect(
                boost::bind(&SimulatedMotionEngineManager::onItemSelectionOrTreeChanged, this, _1)));

        TimeBar* timeBar = TimeBar::instance();
        currentTime = timeBar->time();
        timeChangeConnection.reset(
            timeBar->sigTimeChanged().connect(
                boost::bind(&SimulatedMotionEngineManager::setTime, this, _1)));
    }

    void onItemSelectionOrTreeChanged(const ItemList<SimulatorItem>& selected){

        if(selected.empty()){
            vector<SimulatorItemPtr>::iterator p = simulatorItems.begin();
            while(p != simulatorItems.end()){
                if((*p)->isRunning() && (*p)->findRootItem()){
                    ++p;
                } else {
                    p = simulatorItems.erase(p);
                }
            }
        } else {
            simulatorItems = selected;
            setTime(currentTime);
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
    bool doExecAfterInit;
    double time;
    double timeStep_;
    double delay;
    SimulationScriptItemPtr scriptItem;
    LazyCaller executeLater;
public:
    ScriptControllerItem(SimulationScriptItem* scriptItem){
        this->scriptItem = scriptItem;
        doExecAfterInit = false;
    }
    virtual bool start(Target* target) {
        timeStep_ = target->worldTimeStep();
        if(scriptItem->execTiming() == SimulationScriptItem::DURING_INITIALIZATION){
            scriptItem->executeAsSimulationScript();
        } else if(scriptItem->execTiming() == SimulationScriptItem::AFTER_INITIALIZATION){
            doExecAfterInit = true;
            time = 0.0;
            delay = scriptItem->execDelay();
            executeLater.setFunction(boost::bind(&ScriptControllerItem::execute, this));
        }
        return true;
    }
    virtual double timeStep() const{
        return timeStep_;
    }
    virtual void input() {
    }
    virtual bool control() {
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
    virtual void output() {
    }
    virtual void stop() {
        if(scriptItem->execTiming() == SimulationScriptItem::DURING_FINALIZATION){
            scriptItem->executeAsSimulationScript();
        }
    }
};



}


void SimulatorItem::initializeClass(ExtensionManager* ext)
{
    ext->manage(new SimulatedMotionEngineManager());
}


void ControllerTarget::setSimImpl(SimulatorItemImpl* simImpl)
{
    this->simImpl = simImpl;
}


void ControllerTarget::setSimBodyImpl(SimulationBodyImpl* simBodyImpl)
{
    simImpl = simBodyImpl->simImpl;
    body_ = simBodyImpl->body;
}


Body* ControllerTarget::body()
{
    return body_;
}


double ControllerTarget::worldTimeStep() const
{
    return simImpl->worldTimeStep;
}


double ControllerTarget::currentTime() const
{
    return simImpl->currentTime();
}


SimulationBody::SimulationBody(BodyPtr body)
{
    impl = new SimulationBodyImpl(body);
}


SimulationBodyImpl::SimulationBodyImpl(const BodyPtr& body)
    : body(body)
{
    simImpl = 0;
    areShapesCloned = false;
    doStoreResult = false;
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
    return impl->body;
}


int SimulationBody::numControllers() const
{
    return impl->controllers.size();
}


ControllerItem* SimulationBody::controller(int index) const
{
    if(index < impl->controllers.size()){
        return impl->controllers[index];
    }
    return 0;
}


void SimulationBodyImpl::findControlSrcItems(Item* item, vector<Item*>& io_items, bool doPickCheckedItems)
{
    while(item){
        Item* srcItem = 0;
        if(dynamic_cast<ControllerItem*>(item)){
            srcItem = item;
        } else if(dynamic_cast<BodyMotionItem*>(item)){
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
        }
        if(item->childItem()){
            findControlSrcItems(item->childItem(), io_items, doPickCheckedItems);
        }
        item = item->nextItem();
    }
}


bool SimulationBodyImpl::initialize(SimulatorItemImpl* simImpl, BodyItem* bodyItem)
{
    this->simImpl = simImpl;
    this->bodyItem = bodyItem;
    frameRate = simImpl->worldFrameRate;
    controllerTarget.setSimBodyImpl(this);
    deviceStateConnections.disconnect();
    controllers.clear();

    if(body->isStaticModel()){
        return true;
    }
    
    doStoreResult = true;
    string simulatedMotionName = simImpl->self->name() + "-" + bodyItem->name();

    vector<Item*> controlSrcItems;
    findControlSrcItems(bodyItem->childItem(), controlSrcItems);
    vector<Item*>::iterator iter = controlSrcItems.begin();
    while(iter != controlSrcItems.end()){
        Item* srcItem = *iter;
        ControllerItem* controllerItem = 0;
        if(controllerItem = dynamic_cast<ControllerItem*>(srcItem)){
            controllers.push_back(controllerItem);
        } else {
            BodyMotionItem* motionItem = dynamic_cast<BodyMotionItem*>(srcItem);
            if(motionItem && motionItem->name() != simulatedMotionName){
                controllerItem = simImpl->createBodyMotionController(bodyItem, motionItem);
                if(controllerItem){
                    if(simImpl->doReset){
                        setInitialStateOfBodyMotion(motionItem->motion());
                    }
                    controllers.push_back(controllerItem);
                }
            }
        }
        if(controllerItem){
            ++iter;
        } else {
            iter = controlSrcItems.erase(iter);
        }
    }
    
    if(controlSrcItems.empty()){
        setupResultMotion(simImpl, bodyItem, simulatedMotionName);

    } else if(controlSrcItems.size() == 1){
        setupResultMotion(simImpl, controlSrcItems.front(), simulatedMotionName);

    } else {
        // find the top owenr of all the controllers
        Item* topOwner = 0;
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
                topOwner = owner;
                minDepth = depth;
            }
        }
        setupResultMotion(simImpl, topOwner, simulatedMotionName);
    }
    
    return true;
}


void SimulationBody::cloneShapesOnce()
{
    if(!impl->areShapesCloned){
        if(!impl->simImpl){
            // throw exception
        }
        impl->body->cloneShapes(impl->simImpl->sgCloneMap);
        impl->areShapesCloned = true;
    }
}


void SimulationBodyImpl::setupResultMotion
(SimulatorItemImpl* simImpl, Item* ownerItem, const string& simulatedMotionName)
{
    if(!doStoreResult){
        return;
    }
    
    const int numAllJoints = body->numAllJoints();
    jointPosBuf.resizeColumn(numAllJoints);

    const int numLinksToRecord = simImpl->isAllLinkPositionOutputMode ? body->numLinks() : 1;
    linkPosBuf.resizeColumn(numLinksToRecord);

    if(!simImpl->isRecordingEnabled){
        return;
    }

    BodyMotionItem* motionItem = ownerItem->findItem<BodyMotionItem>(simulatedMotionName);
    if(!motionItem){
        motionItem = new BodyMotionItem();
        motionItem->setTemporal();
        motionItem->setName(simulatedMotionName);
        ownerItem->addChildItem(motionItem);
    }

    simImpl->addBodyMotionEngine(motionItem);
    motion = motionItem->motion();
    motion->setFrameRate(frameRate);
    motion->setDimension(1, numAllJoints, numLinksToRecord);
    
    jointPosResult = motion->jointPosSeq();
    if(numAllJoints > 0){
        MultiValueSeq::Frame jframe0 = jointPosResult->frame(0);
        for(int i=0; i < numAllJoints; ++i){
            jframe0[i] = body->joint(i)->q();
        }
    }

    linkPosResultItem = motionItem->linkPosSeqItem();
    linkPosResult = motion->linkPosSeq();
    MultiSE3Seq::Frame lframe0 = linkPosResult->frame(0);
    for(int i=0; i < numLinksToRecord; ++i){
        Link* link = body->link(i);
        lframe0[i].set(link->p(), link->R());
    }
}


void SimulationBodyImpl::setupDeviceStateRecording()
{
    const DeviceList<>& devices = body->devices();
    deviceStateChangeFlag.reset();
    deviceStateChangeFlag.resize(devices.size());
    devicesToNotifyResult.clear();
    
    if(devices.empty() || !simImpl->isDeviceStateOutputEnabled){
        deviceStateBuf.clear();
        if(motion){
            clearMultiDeviceStateSeq(*motion);
        }
        prevFlushedDeviceStateInDirectMode.clear();
    } else {
        deviceStateBuf.resize(1, devices.size());
        prevFlushedDeviceStateInDirectMode.resize(devices.size());
        Deque2D<DeviceStatePtr>::Row dstate = deviceStateBuf.last();

        for(size_t i=0; i < devices.size(); ++i){
            Device* device = devices[i];
            DeviceState* s = device->cloneState();
            dstate[i] = s;
            prevFlushedDeviceStateInDirectMode[i] = s;
            deviceStateConnections.add(
                device->sigStateChanged().connect(
                    boost::bind(&SimulationBodyImpl::onDeviceStateChanged, this, i)));
        }
        if(simImpl->isRecordingEnabled){
            deviceStateResult = getOrCreateMultiDeviceStateSeq(*motion);
            deviceStateResult->setNumParts(devices.size());
            MultiDeviceStateSeq::Row result0 = deviceStateResult->frame(0);
            for(size_t i=0; i < dstate.size(); ++i){
                result0[i] = dstate[i];
            }
        }
    }
}


void SimulationBodyImpl::setInitialStateOfBodyMotion(const BodyMotionPtr& bodyMotion)
{
    bool updated = false;
    
    MultiSE3SeqPtr lseq = bodyMotion->linkPosSeq();
    if(lseq->numParts() > 0 && lseq->numFrames() > 0){
        SE3& p = lseq->at(0, 0);
        Link* rootLink = body->rootLink();
        rootLink->p() = p.translation();
        rootLink->R() = p.rotation().toRotationMatrix();
        updated = true;
    }
    MultiValueSeqPtr jseq = bodyMotion->jointPosSeq();
    if(jseq->numFrames() > 0){
        MultiValueSeq::Frame jframe0 = jseq->frame(0);
        int n = std::min(jframe0.size(), body->numJoints());
        for(int i=0; i < n; ++i){
            body->joint(i)->q() = jframe0[i];
        }
        updated = true;
    }
    if(updated){
        body->calcForwardKinematics();
    }
}


// For a controller which is not associated with a body
bool SimulationBodyImpl::initialize(SimulatorItemImpl* simImpl, ControllerItem* controllerItem)
{
    this->simImpl = simImpl;
    this->controllers.push_back(controllerItem);
    frameRate = simImpl->worldFrameRate;
    linkPosBuf.resizeColumn(0);
    return true;
}


void SimulationBody::notifyUnrecordedDeviceStateChange(Device* device)
{
    bool flag = impl->deviceStateChangeFlag[device->index()];
    device->notifyStateChange();
    impl->deviceStateChangeFlag[device->index()] = flag;
}


void SimulationBodyImpl::onDeviceStateChanged(int deviceIndex)
{
    deviceStateChangeFlag.set(deviceIndex);
}


void SimulationBody::storeResult()
{
    if(impl->doStoreResult){
        impl->storeResult();
    }
}


void SimulationBodyImpl::storeResult()
{
    if(jointPosBuf.colSize() > 0){
        Deque2D<double>::Row q = jointPosBuf.append();
        for(int i=0; i < q.size() ; ++i){
            q[i] = body->joint(i)->q();
        }
    }
    MultiSE3Deque::Row pos = linkPosBuf.append();
    for(int i=0; i < linkPosBuf.colSize(); ++i){
        Link* link = body->link(i);
        pos[i].set(link->p(), link->R());
    }

    if(!deviceStateBuf.empty()){
        Deque2D<DeviceStatePtr>::Row current = deviceStateBuf.append();
        Deque2D<DeviceStatePtr>::Row old = deviceStateBuf[deviceStateBuf.rowSize() - 2];
        const DeviceList<>& devices = body->devices();
        for(size_t i=0; i < devices.size(); ++i){
            if(deviceStateChangeFlag[i]){
                current[i] = devices[i]->cloneState();
                deviceStateChangeFlag.reset(i);
            } else {
                current[i] = old[i];
            }
        }
    }
}


bool SimulationBody::flushResult()
{
    if(!impl->doStoreResult){
        return false;
    }
    impl->flushResult();
    return true;
}


void SimulationBodyImpl::flushResult()
{
    if(simImpl->isRecordingEnabled){

        const int ringBufferSize = simImpl->ringBufferSize;
        const int numBufFrames = linkPosBuf.rowSize();

        for(int i=0; i < numBufFrames; ++i){
            MultiSE3Deque::Row buf = linkPosBuf.row(i);
            if(linkPosResult->numFrames() >= ringBufferSize){
                linkPosResult->popFrontFrame();
            }
            std::copy(buf.begin(), buf.end(), linkPosResult->appendFrame().begin());
        }
            
        if(jointPosBuf.colSize() > 0){
            for(int i=0; i < numBufFrames; ++i){
                Deque2D<double>::Row buf = jointPosBuf.row(i);
                if(jointPosResult->numFrames() >= ringBufferSize){
                    jointPosResult->popFrontFrame();
                }
                std::copy(buf.begin(), buf.end(), jointPosResult->appendFrame().begin());
            }
        }
        if(deviceStateBuf.colSize() > 0){
            for(int i=0; i < numBufFrames; ++i){
                Deque2D<DeviceStatePtr>::Row buf = deviceStateBuf.row(i);
                if(deviceStateResult->numFrames() >= ringBufferSize){
                    deviceStateResult->popFrontFrame();
                }
                std::copy(buf.begin(), buf.end(), deviceStateResult->appendFrame().begin());
            }
            // keep the last state so that unchanged states can be shared
            deviceStateBuf.pop_front(deviceStateBuf.rowSize() - 1);
        }
    } else {
        const Body* orgBody = bodyItem->body();
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
            const int n = body->numJoints();
            for(int i=0; i < n; ++i){
                orgBody->joint(i)->q() = last[i];
            }
        }
        if(!deviceStateBuf.empty()){
            devicesToNotifyResult.clear();
            const DeviceList<>& devices = orgBody->devices();
            Deque2D<DeviceStatePtr>::Row ds = deviceStateBuf.last();
            const int ndevices = devices.size();
            for(size_t i=0; i < ndevices; ++i){
                const DeviceStatePtr& s = ds[i];
                if(s != prevFlushedDeviceStateInDirectMode[i]){
                    Device* device = devices.get(i);
                    device->copyStateFrom(*s);
                    prevFlushedDeviceStateInDirectMode[i] = s;
                    devicesToNotifyResult.push_back(device);
                }
            }
            // keep the last state so that unchanged states can be shared
            deviceStateBuf.pop_front(deviceStateBuf.rowSize() - 1);
        }
    }

    jointPosBuf.resizeRow(0);
    linkPosBuf.resizeRow(0);
}


void SimulationBodyImpl::notifyResult()
{
    bodyItem->notifyKinematicStateChange(!simImpl->isAllLinkPositionOutputMode);

    for(size_t i=0; i < devicesToNotifyResult.size(); ++i){
        devicesToNotifyResult[i]->notifyStateChange();
    }
}


SimulatorItem::SimulatorItem()
{
    impl = new SimulatorItemImpl(this);
}


SimulatorItem::SimulatorItem(const SimulatorItem& org)
    : Item(org)
{
    impl = new SimulatorItemImpl(this);

    impl->isRealtimeSyncMode = org.impl->isRealtimeSyncMode;
    impl->isAllLinkPositionOutputMode = org.impl->isAllLinkPositionOutputMode;
    impl->isDeviceStateOutputEnabled = org.impl->isDeviceStateOutputEnabled;
    impl->recordingMode = org.impl->recordingMode;
    impl->timeRangeMode = org.impl->timeRangeMode;
    impl->isActiveControlPeriodOnlyMode = org.impl->isActiveControlPeriodOnlyMode;
    impl->useControllerThreadsProperty = org.impl->useControllerThreadsProperty;
    impl->recordCollisionData = org.impl->recordCollisionData;
}


SimulatorItemImpl::SimulatorItemImpl(SimulatorItem* self)
    : self(self),
      mv(MessageView::mainInstance()),
      os(mv->cout()),
      recordingMode(SimulatorItem::N_RECORDING_MODES, CNOID_GETTEXT_DOMAIN_NAME),
      timeRangeMode(SimulatorItem::N_TIME_RANGE_MODES, CNOID_GETTEXT_DOMAIN_NAME),
      itemTreeView(ItemTreeView::instance())
{
    controllerTarget.setSimImpl(this);
    
    flushTimer.sigTimeout().connect(boost::bind(&SimulatorItemImpl::flushResult, this));
    
    timeBar = TimeBar::instance();
    isDoingSimulationLoop = false;
    isRealtimeSyncMode = false;

    recordingMode.setSymbol(SimulatorItem::RECORD_FULL, N_("full"));
    recordingMode.setSymbol(SimulatorItem::RECORD_TAIL, N_("tail"));
    recordingMode.setSymbol(SimulatorItem::RECORD_NONE, N_("off"));
    recordingMode.select(SimulatorItem::RECORD_TAIL);
    
    timeRangeMode.setSymbol(SimulatorItem::TIMEBAR_RANGE, N_("TimeBar range"));
    timeRangeMode.setSymbol(SimulatorItem::SPECIFIED_PERIOD, N_("Specified period"));
    timeRangeMode.setSymbol(SimulatorItem::UNLIMITED, N_("Unlimited"));
    timeRangeMode.select(SimulatorItem::UNLIMITED);
    specifiedTimeLength = 180.0; // 3 min.
    isActiveControlPeriodOnlyMode = true;
    useControllerThreadsProperty = true;
    isAllLinkPositionOutputMode = false;
    isDeviceStateOutputEnabled = true;
    recordCollisionData = false;
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


void SimulatorItem::onDisconnectedFromRoot()
{
    impl->stopSimulation(true);
}


double SimulatorItem::worldTimeStep()
{
    return TimeBar::instance()->timeStep();
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


void SimulatorItem::setActiveControlPeriodOnlyMode(bool on)
{
    impl->isActiveControlPeriodOnlyMode = on;
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


bool SimulatorItemImpl::setSpecifiedRecordingTimeLength(double length)
{
    specifiedTimeLength = length;
    return true;
}


CollisionDetectorPtr SimulatorItem::collisionDetector()
{
    if(impl->collisionDetector){
        return impl->collisionDetector;
    }
    WorldItem* worldItem = findOwnerItem<WorldItem>();
    if(worldItem){
        return worldItem->collisionDetector()->clone();
    }
    return CollisionDetector::create(0); // the null collision detector
}


ControllerItem* SimulatorItem::createBodyMotionController(BodyItem* bodyItem, BodyMotionItem* bodyMotionItem)
{
    return 0;
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
            if(scriptItem->execTiming() == SimulationScriptItem::BEFORE_INITIALIZATION){
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


bool SimulatorItem::startSimulation(bool doReset)
{
    return impl->startSimulation(doReset);
}


bool SimulatorItemImpl::startSimulation(bool doReset)
{
    this->doReset = doReset;
    
    stopSimulation(true);

    WorldItem* worldItem = self->findOwnerItem<WorldItem>();
    if(!worldItem){
        os << (fmt(_("%1% must be in a WorldItem to do simulation.")) % self->name()) << endl;
        return false;
    }

    ItemList<Item> targetItems;
    findTargetItems(worldItem, false, targetItems);
    if(targetItems.empty()){
        return false;
    }

    sgCloneMap.clear();

    worldTimeStep = self->worldTimeStep();
    worldFrameRate = 1.0 / worldTimeStep;

    if(recordingMode.is(SimulatorItem::RECORD_NONE)){
        isRecordingEnabled = false;
        isRingBufferMode = false;
    } else {
        isRecordingEnabled = true;
        isRingBufferMode = recordingMode.is(SimulatorItem::RECORD_TAIL);
    }

    allSimBodies.clear();
    simBodiesWithBody.clear();;
    activeSimBodies.clear();
    bodyMotionEngines.clear();
    needToUpdateSimBodyLists = true;

    for(size_t i=0; i < targetItems.size(); ++i){

        if(BodyItem* bodyItem = dynamic_cast<BodyItem*>(targetItems.get(i))){
            if(doReset){
                bodyItem->restoreInitialState();
            }
            SimulationBodyPtr simBody = self->createSimulationBody(bodyItem->body());
            if(simBody->body()){
                if(simBody->impl->initialize(this, bodyItem)){
                    allSimBodies.push_back(simBody);
                    simBodiesWithBody.push_back(simBody);
                }
            }
        } else if(ControllerItem* controller = dynamic_cast<ControllerItem*>(targetItems.get(i))){
            // ControllerItem which is not associated with a body
            SimulationBodyPtr simBody = new SimulationBody(BodyPtr());
            if(simBody->impl->initialize(this, controller)){
                allSimBodies.push_back(simBody);
            }
        } else if(SimulationScriptItem* script = dynamic_cast<SimulationScriptItem*>(targetItems.get(i))){
            SimulationBodyPtr simBody = new SimulationBody(BodyPtr());
            if(simBody->impl->initialize(this, new ScriptControllerItem(script))){
                allSimBodies.push_back(simBody);
            }
        }
    }
    
    if(isRecordingEnabled && recordCollisionData){
        collisionPairsBuf.clear();
        string collisionSeqName = self->name() + "-collisions";
        CollisionSeqItem* collisionSeqItem = worldItem->findItem<CollisionSeqItem>(collisionSeqName);
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
        frame0[0]  = boost::make_shared<CollisionLinkPairList>();
    }

    bool result = self->initializeSimulation(simBodiesWithBody);

    if(result){

        currentFrame = 0;
        frameAtLastBufferWriting = 0;
        isDoingSimulationLoop = true;
        isWaitingForSimulationToStop = false;
        stopRequested = false;
        pauseRequested = false;

        ringBufferSize = std::numeric_limits<int>::max();
        
        if(timeRangeMode.is(SimulatorItem::SPECIFIED_PERIOD)){
            maxFrame = specifiedTimeLength / worldTimeStep;
        } else if(timeRangeMode.is(SimulatorItem::TIMEBAR_RANGE)){
            maxFrame = TimeBar::instance()->maxTime() / worldTimeStep;
        } else if(isRingBufferMode){
            maxFrame = std::numeric_limits<int>::max();
            ringBufferSize = specifiedTimeLength / worldTimeStep;
        } else {
            maxFrame = std::numeric_limits<int>::max();
        }

        preDynamicsFunctions.clear();
        postDynamicsFunctions.clear();
        subSimulatorItems.extractChildItems(self);
        ItemList<SubSimulatorItem>::iterator p = subSimulatorItems.begin();
        while(p != subSimulatorItems.end()){
            SubSimulatorItem* item = *p;
            bool initialized = false;
            if(item->isEnabled()){
                os << (fmt(_("SubSimulatorItem \"%1%\" has been detected.")) % item->name()) << endl;
                if(item->initializeSimulation(self)){
                    initialized = true;
                } else {
                    os << (fmt(_("The initialization of \"%1%\" failed.")) % item->name()) << endl;
                }
            } else {
                os << (fmt(_("SubSimulatorItem \"%1%\" is disabled.")) % item->name()) << endl;
            }
            if(initialized){
                ++p;
            } else {
                p = subSimulatorItems.erase(p);
            }
        }

        for(size_t i=0; i < simBodiesWithBody.size(); ++i){
            simBodiesWithBody[i]->impl->setupDeviceStateRecording();
        }

        for(size_t i=0; i < allSimBodies.size(); ++i){
            SimulationBodyImpl* simBodyImpl = allSimBodies[i]->impl;
            Body* body = simBodyImpl->body;
            vector<ControllerItemPtr>& controllers = simBodyImpl->controllers;
            vector<ControllerItemPtr>::iterator iter = controllers.begin();
            while(iter != controllers.end()){
                ControllerItem* controller = *iter;
                bool ready = false;
                controller->setSimulatorItem(self);
                if(body){
                    ready = controller->start(&simBodyImpl->controllerTarget);
                    if(!ready){
                        os << (fmt(_("%1% for %2% failed to initialize."))
                               % controller->name() % simBodyImpl->bodyItem->name()) << endl;
                    }
                } else {
                    ready = controller->start(&controllerTarget);
                    if(!ready){
                        os << (fmt(_("%1% failed to initialize."))
                               % controller->name()) << endl;
                    }
                }
                if(ready){
                    ++iter;
                } else {
                    controller->setSimulatorItem(0);
                    string message = controller->getMessage();
                    if(!message.empty()){
                        os << message << endl;
                    }
                    iter = controllers.erase(iter);
                }
            }
        }

        useControllerThreads = useControllerThreadsProperty;
        if(useControllerThreads){
            controlThread = boost::thread(boost::bind(&SimulatorItemImpl::concurrentControlLoop, this));
            isExitingControlLoopRequested = false;
            isControlRequested = false;
            isControlFinished = false;
        }

        aboutToQuitConnection.disconnect();
        aboutToQuitConnection = cnoid::sigAboutToQuit().connect(boost::bind(&SimulatorItemImpl::stopSimulation, this, true));

        if(isRecordingEnabled){
            fillLevelId = timeBar->startFillLevelUpdate();
        }
        if(!timeBar->isDoingPlayback()){
            timeBar->setTime(0.0);
            timeBar->startPlayback();
        }

        start();
        flushTimer.start(1000.0 / timeBar->playbackFrameRate());

        mv->notify(format(_("Simulation by %1% has started.")) % self->name());
    }

    return result;
}


void SimulatorItem::addPreDynamicsFunction(boost::function<void()> func)
{
    impl->preDynamicsFunctions.push_back(func);
}


void SimulatorItem::addMidDynamicsFunction(boost::function<void()> func)
{
    impl->midDynamicsFunctions.push_back(func);
}


void SimulatorItem::addPostDynamicsFunction(boost::function<void()> func)
{
    impl->postDynamicsFunctions.push_back(func);
}


SgCloneMap& SimulatorItem::sgCloneMap()
{
    return impl->sgCloneMap;
}


ControllerItem* SimulatorItemImpl::createBodyMotionController(BodyItem* bodyItem, BodyMotionItem* bodyMotionItem)
{
    return self->createBodyMotionController(bodyItem, bodyMotionItem);
}


void SimulatorItem::initializeSimulationThread()
{

}


const std::vector<SimulationBody*>& SimulatorItem::simulationBodies()
{
    return impl->simBodiesWithBody;
}
    

// Simulation loop
void SimulatorItemImpl::run()
{
    self->initializeSimulationThread();

    double elapsedTime = 0.0;
    QElapsedTimer timer;
    timer.start();

    int frame = 0;
    bool onPause = false;

    if(isRealtimeSyncMode){
        const double dt = worldTimeStep;
        const double compensationRatio = (dt > 0.1) ? 0.1 : dt;
        const double dtms = dt * 1000.0;
        double compensatedSimulationTime = 0.0;
        while(true){
        	if(!pauseRequested){
        		if(onPause){
        			timer.start();
        			onPause = false;
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
        	}else{
        		if(stopRequested)
        			break;
        		if(!onPause){
        			elapsedTime += timer.elapsed();
        			onPause = true;
        		}
        	}
        }
    } else {
        while(true){
        	if(!pauseRequested){
        		if(onPause){
        			timer.start();
        			onPause = false;
        		}
				if(!stepSimulationMain() || stopRequested || frame++ >= maxFrame){
					break;
				}
        	}else{
        		if(stopRequested)
        			break;
        		if(!onPause){
        			elapsedTime += timer.elapsed();
        			onPause = true;
        		}
        	}
        }
    }

    if(!onPause)
    	elapsedTime += timer.elapsed();
    actualSimulationTime = (elapsedTime / 1000.0);
    finishTime = frame / worldFrameRate;

    isDoingSimulationLoop = false;

    if(useControllerThreads){
        {
            boost::unique_lock<boost::mutex> lock(controlMutex);
            isExitingControlLoopRequested = true;
        }
        controlCondition.notify_all();
        controlThread.join();
    }

    if(!isWaitingForSimulationToStop){
        callLater(boost::bind(&SimulatorItemImpl::onSimulationLoopStopped, this));
    }
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
        if(simBodyImpl->body && !simBodyImpl->body->isStaticModel()){
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
    
    bool doContinue = hasActiveFreeBodies || !isActiveControlPeriodOnlyMode;

    for(size_t i=0; i < preDynamicsFunctions.size(); ++i){
        preDynamicsFunctions[i]();
    }

    if(useControllerThreads){
        if(activeControllers.empty()){
            isControlFinished = true;
        } else {
            for(size_t i=0; i < activeControllers.size(); ++i){
                activeControllers[i]->input();
            }
            {
                boost::unique_lock<boost::mutex> lock(controlMutex);                
                isControlRequested = true;
            }
            controlCondition.notify_all();
        }
    } else {
        for(size_t i=0; i < activeControllers.size(); ++i){
            ControllerItem* controller = activeControllers[i];
            controller->input();
            doContinue |= controller->control();
            if(controller->isImmediateMode()){
                controller->output();
            }
        }
    }

    for(size_t i=0; i < midDynamicsFunctions.size(); ++i){
        midDynamicsFunctions[i]();
    }
    self->stepSimulation(activeSimBodies);

    CollisionLinkPairListPtr collisionPairs;
    if(isRecordingEnabled && recordCollisionData){
        collisionPairs = self->getCollisions();
    }

    if(useControllerThreads){
        {
            boost::unique_lock<boost::mutex> lock(controlMutex);
            while(!isControlFinished){
                controlCondition.wait(lock);
            }
        }
        isControlFinished = false;
        doContinue |= isControlToBeContinued;
    }

    for(size_t i=0; i < postDynamicsFunctions.size(); ++i){
        postDynamicsFunctions[i]();
    }

    {
        mutex.lock();
        
        for(size_t i=0; i < activeSimBodies.size(); ++i){
            activeSimBodies[i]->storeResult();
        }
        collisionPairsBuf.push_back(collisionPairs);
        frameAtLastBufferWriting = currentFrame;

        mutex.unlock();
    }

    if(useControllerThreads){
        for(size_t i=0; i < activeControllers.size(); ++i){
            activeControllers[i]->output();
        }
    } else {
        for(size_t i=0; i < activeControllers.size(); ++i){
            ControllerItem* controller = activeControllers[i];
            if(!controller->isImmediateMode()){
                controller->output(); 
            }
        }
    }

    return doContinue;
}


void SimulatorItemImpl::concurrentControlLoop()
{
    while(true){
        {
            boost::unique_lock<boost::mutex> lock(controlMutex);
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
        for(size_t i=0; i < activeControllers.size(); ++i){
            doContinue |= activeControllers[i]->control();
        }
        
        {
            boost::unique_lock<boost::mutex> lock(controlMutex);
            isControlFinished = true;
            isControlToBeContinued = doContinue;
        }
        controlCondition.notify_all();
    }
    
exitConcurrentControlLoop:
    return;
}


void SimulatorItemImpl::flushResult()
{
    simBodyImplsToNotifyResult.clear();

    mutex.lock();
    
    for(size_t i=0; i < allSimBodies.size(); ++i){
        SimulationBody* simBody = allSimBodies[i];
        if(simBody->flushResult()){
            simBodyImplsToNotifyResult.push_back(simBody->impl);
        }
    }
    if(isRecordingEnabled && recordCollisionData){
        for(int i=0 ; i<collisionPairsBuf.size(); i++ ){
            if(collisionSeq->numFrames() >= ringBufferSize){
                collisionSeq->popFrontFrame();
            }
            CollisionSeq::Frame collisionSeq0 = collisionSeq->appendFrame();
            collisionSeq0[0] = collisionPairsBuf[i];
        }
    }
    collisionPairsBuf.clear();

    int frame = frameAtLastBufferWriting;
    
    mutex.unlock();

    if(isRecordingEnabled){
        double fillLevel = frame / worldFrameRate;
        timeBar->updateFillLevel(fillLevelId, fillLevel);
    } else {
        for(size_t i=0; i < simBodyImplsToNotifyResult.size(); ++i){
            simBodyImplsToNotifyResult[i]->notifyResult();
        }
        timeBar->setTime(frame / worldFrameRate);
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
            controller->setSimulatorItem(0);
        }
    }
    self->finalizeSimulation();

    for(size_t i=0; i < subSimulatorItems.size(); ++i){
        subSimulatorItems[i]->finalizeSimulation();
    }

    flushResult();

    if(isRecordingEnabled){
        timeBar->stopFillLevelUpdate(fillLevelId);
    }

    sigSimulationFinished_();

    allSimBodies.clear();
    activeSimBodies.clear();
    preDynamicsFunctions.clear();
    midDynamicsFunctions.clear();
    postDynamicsFunctions.clear();
    subSimulatorItems.clear();
    
    mv->notify(format(_("Simulation by %1% has finished at %2% [s].")) % self->name() % finishTime);
    mv->putln(format(_("Computation time is %1% [s], computation time / simulation time = %2%."))
              % actualSimulationTime % (actualSimulationTime / finishTime));
}


bool SimulatorItem::isRunning() const
{
    return impl->isDoingSimulationLoop;
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


SignalProxy<void()> SimulatorItem::sigSimulationFinished()
{
    return impl->sigSimulationFinished_;
}


bool SimulatorItemImpl::onRealtimeSyncChanged(bool on)
{
    isRealtimeSyncMode = on;
    return true;
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
    putProperty(_("Sync with realtime"), impl->isRealtimeSyncMode,
                boost::bind(&SimulatorItemImpl::onRealtimeSyncChanged, impl, _1));
    putProperty(_("Recording"), impl->recordingMode,
                boost::bind(&Selection::selectIndex, &impl->recordingMode, _1));
    putProperty(_("Time range"), impl->timeRangeMode,
                boost::bind(&Selection::selectIndex, &impl->timeRangeMode, _1));
    putProperty(_("Active control period only"), impl->isActiveControlPeriodOnlyMode,
                changeProperty(impl->isActiveControlPeriodOnlyMode));
    putProperty(_("Time length"), impl->specifiedTimeLength,
                boost::bind(&SimulatorItemImpl::setSpecifiedRecordingTimeLength, impl, _1));
    putProperty(_("All link positions"), impl->isAllLinkPositionOutputMode,
                boost::bind(&SimulatorItemImpl::onAllLinkPositionOutputModeChanged, impl, _1));
    putProperty(_("Device state output"), impl->isDeviceStateOutputEnabled,
                changeProperty(impl->isDeviceStateOutputEnabled));
    putProperty(_("Controller Threads"), impl->useControllerThreadsProperty,
                changeProperty(impl->useControllerThreadsProperty));
    putProperty(_("Record collision data"), impl->recordCollisionData,
                changeProperty(impl->recordCollisionData));
}


bool SimulatorItem::store(Archive& archive)
{
    return impl->store(archive);
}


bool SimulatorItemImpl::store(Archive& archive)
{
    archive.write("realtimeSync", isRealtimeSyncMode);
    archive.write("recording", recordingMode.selectedSymbol());
    archive.write("timeRangeMode", timeRangeMode.selectedSymbol());
    archive.write("onlyActiveControlPeriod", isActiveControlPeriodOnlyMode);
    archive.write("timeLength", specifiedTimeLength);
    archive.write("allLinkPositionOutputMode", isAllLinkPositionOutputMode);
    archive.write("deviceStateOutput", isDeviceStateOutputEnabled);
    archive.write("controllerThreads", useControllerThreadsProperty);
    archive.write("recordCollisionData", recordCollisionData);

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
    if(archive.read("timeRangeMode", symbol)){
        timeRangeMode.select(symbol);
    }
    if(archive.read("recording", symbol)){
        recordingMode.select(symbol);
    }
    // for the compatibility with older version
    else if(archive.read("recording", boolValue)){ 
        recordingMode.select(SimulatorItem::RECORD_FULL);
    } else if(archive.read("recordingMode", symbol)){
        if(symbol == "Direct"){
            recordingMode.select(SimulatorItem::RECORD_NONE);
            timeRangeMode.select(SimulatorItem::UNLIMITED);
        }
    }
    archive.read("realtimeSync", isRealtimeSyncMode);
    archive.read("onlyActiveControlPeriod", isActiveControlPeriodOnlyMode);
    archive.read("timeLength", specifiedTimeLength);
    self->setAllLinkPositionOutputMode(archive.get("allLinkPositionOutputMode", isAllLinkPositionOutputMode));
    archive.read("deviceStateOutput", isDeviceStateOutputEnabled);
    archive.read("controllerThreads", useControllerThreadsProperty);
    archive.read("recordCollisionData", recordCollisionData);

    archive.addPostProcess(
        boost::bind(&SimulatorItemImpl::restoreBodyMotionEngines, this, boost::ref(archive)));
    
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

    collisionSeqEngine = 0;
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
    WorldItem* worldItem = collisionSeqItem->findOwnerItem<WorldItem>();
    if(worldItem){
        collisionSeqEngine = new CollisionSeqEngine(worldItem, collisionSeqItem);
    }
}


bool SimulatorItemImpl::setPlaybackTime(double time)
{
    bool processed = false;
    for(size_t i=0; i < bodyMotionEngines.size(); ++i){
        processed |= bodyMotionEngines[i]->onTimeChanged(time);
    }
    if(collisionSeqEngine)
        processed |= collisionSeqEngine->onTimeChanged(time);
    return processed;
}


bool SimulatedMotionEngineManager::setTime(double time)
{
    bool isActive = false;
    currentTime = time;
    for(size_t i=0; i < simulatorItems.size(); ++i){
        isActive |= simulatorItems[i]->impl->setPlaybackTime(currentTime);
    }
    return isActive;
}


bool SubSimulatorItem::isEnabled()
{
    return true;
}
