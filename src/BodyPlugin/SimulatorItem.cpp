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

class SimulatedMotionEngine;
typedef boost::shared_ptr<SimulatedMotionEngine> SimulatedMotionEnginePtr;

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
    ControllerItemPtr controller;
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
    bool findControllerItem(Item* item, Item*& foundItem);
    bool initialize(SimulatorItemImpl* simImpl, BodyItem* bodyItem);
    bool initialize(SimulatorItemImpl* simImpl, ControllerItem* controllerItem);
    void cloneShapesOnce();
    void setupController(SimulatorItemImpl* simImpl, ControllerItem* controllerItem);
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
    bool stopRequested;
    bool isRealtimeSyncMode;
    bool needToUpdateSimBodyLists;
    bool hasActiveFreeBodies;

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

    SimulatedMotionEnginePtr motionEngine;

    Connection aboutToQuitConnection;

    SgCloneMap sgCloneMap;
        
    ItemTreeView* itemTreeView;

    double currentTime() const;
    void selectMotionItems();
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
    void onSimulationLoopStopped();
    bool onRealtimeSyncChanged(bool on);
    bool onAllLinkPositionOutputModeChanged(bool on);
    bool setSpecifiedRecordingTimeLength(double length);

    static TimeSyncItemEnginePtr getSimulatedMotionEngine(Item* sourceItem);
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


class SimulatedMotionEngine : public TimeSyncItemEngine
{
public:
    vector<BodyMotionEnginePtr> engines;

    void clear() {
        engines.clear();
    }

    void addMotionItem(BodyMotionItem* motionItem)
        {
            BodyItem* bodyItem = motionItem->findOwnerItem<BodyItem>();
            if(bodyItem){
                engines.push_back(boost::make_shared<BodyMotionEngine>(bodyItem, motionItem));
            }
        }

    virtual bool onTimeChanged(double time) {
        bool processed = false;
        for(size_t i=0; i < engines.size(); ++i){
            processed |= engines[i]->onTimeChanged(time);
        }
        return processed;
    }

    void store(Archive& archive) {
        ListingPtr idseq = new Listing();
        idseq->setFlowStyle(true);
        for(size_t i=0; i < engines.size(); ++i){
            BodyMotionEnginePtr engine = engines[i];
            ValueNodePtr id = archive.getItemId(engine->motionItem());
            if(id){
                idseq->append(id);
            }
        }
        if(!idseq->empty()){
            archive.insert("motionItems", idseq);
        }
    }

    void restore(const Archive& archive) {
        archive.addPostProcess(boost::bind(&SimulatedMotionEngine::restoreMotionItems, this, boost::ref(archive)));
    }

    void restoreMotionItems(const Archive& archive) {
        clear();
        const Listing& idseq = *archive.findListing("motionItems");
        if(idseq.isValid()){
            for(int i=0; i < idseq.size(); ++i){
                ValueNode* id = idseq.at(i);
                if(id){
                    BodyMotionItem* motionItem = dynamic_cast<BodyMotionItem*>(archive.findItem(id));
                    if(motionItem){
                        addMotionItem(motionItem);
                    }
                }
            }
        }
    }
};
}


void SimulatorItem::initializeClass(ExtensionManager* ext)
{
    ext->timeSyncItemEngineManger().addEngineFactory(SimulatorItemImpl::getSimulatedMotionEngine);
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
    return impl->bodyItem.get();
}


Body* SimulationBody::body() const
{
    return impl->body.get();
}


ControllerItem* SimulationBody::controller() const
{
    return impl->controller.get();
}


bool SimulationBodyImpl::findControllerItem(Item* item, Item*& foundItem)
{
    Item* typeMatchItem = 0;
    
    while(item){
        if(dynamic_cast<ControllerItem*>(item)){
            typeMatchItem = item;
        } else if(dynamic_cast<BodyMotionItem*>(item)){
            typeMatchItem = item;
        }
        if(typeMatchItem){
            if(ItemTreeView::instance()->isItemChecked(typeMatchItem)){
                foundItem = typeMatchItem;
                return true;
            }
            if(!foundItem){
                foundItem = typeMatchItem;
            }
        }
        if(item->childItem()){
            if(findControllerItem(item->childItem(), foundItem)){
                return true;
            }
        }
        item = item->nextItem();
    }

    return false;
}


bool SimulationBodyImpl::initialize(SimulatorItemImpl* simImpl, BodyItem* bodyItem)
{
    this->simImpl = simImpl;
    this->bodyItem = bodyItem;

    controllerTarget.setSimBodyImpl(this);

    // needed ?
    //body->clearExternalForces();
    //body->calcForwardKinematics(true, true);

    deviceStateConnections.disconnect();

    if(body->isStaticModel()){
        return true;
    }
    doStoreResult = true;

    Item* targetItem = 0;
    findControllerItem(bodyItem->childItem(), targetItem);
    ControllerItem* controllerItem = 0;

    string simulatedMotionName = simImpl->self->name() + "-" + bodyItem->name();
    
    if(targetItem){
        controllerItem = dynamic_cast<ControllerItem*>(targetItem);
        if(controllerItem){
            setupController(simImpl, controllerItem);
            setupResultMotion(simImpl, controllerItem, simulatedMotionName);

        } else {
            BodyMotionItem* motionItem = dynamic_cast<BodyMotionItem*>(targetItem);
            const string& name = motionItem->name();
            if(motionItem && motionItem->name() != simulatedMotionName){
                controllerItem = simImpl->createBodyMotionController(bodyItem, motionItem);
                if(controllerItem){
                    if(simImpl->doReset){
                        setInitialStateOfBodyMotion(motionItem->motion());
                    }
                    setupController(simImpl, controllerItem);
                    setupResultMotion(simImpl, motionItem, simulatedMotionName);
                }
            }
        }
    }
    if(!controllerItem){
        frameRate = simImpl->worldFrameRate;
        setupResultMotion(simImpl, bodyItem, simulatedMotionName);
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


void SimulationBodyImpl::setupController(SimulatorItemImpl* simImpl, ControllerItem* controllerItem)
{
    frameRate = simImpl->worldFrameRate;
    controller = controllerItem;
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
    
    BodyMotionItem* motionItem = dynamic_cast<BodyMotionItem*>(ownerItem);
    if(motionItem){
        BodyMotionItem* resultMotionItem = motionItem->findItem<BodyMotionItem>(simulatedMotionName);
        if(resultMotionItem){
            motionItem = resultMotionItem;
        } else {
            resultMotionItem = new BodyMotionItem();
            resultMotionItem->setTemporal();
            resultMotionItem->setName(simulatedMotionName);
            motionItem->addChildItem(resultMotionItem);
            motionItem = resultMotionItem;
        }
    } else {
        motionItem = ownerItem->findItem<BodyMotionItem>(simulatedMotionName);
        if(!motionItem){
            motionItem = new BodyMotionItem();
            motionItem->setTemporal();
            motionItem->setName(simulatedMotionName);
            ownerItem->addChildItem(motionItem);
        }
    }

    simImpl->motionEngine->addMotionItem(motionItem);
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
    this->controller = controllerItem;
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
    recordingMode.select(SimulatorItem::RECORD_FULL);
    
    timeRangeMode.setSymbol(SimulatorItem::TIMEBAR_RANGE, N_("TimeBar range"));
    timeRangeMode.setSymbol(SimulatorItem::SPECIFIED_PERIOD, N_("Specified period"));
    timeRangeMode.setSymbol(SimulatorItem::UNLIMITED, N_("Unlimited"));
    timeRangeMode.select(SimulatorItem::TIMEBAR_RANGE);
    specifiedTimeLength = 60.0;
    isActiveControlPeriodOnlyMode = true;
    useControllerThreadsProperty = true;
    isAllLinkPositionOutputMode = false;
    isDeviceStateOutputEnabled = true;
    
    motionEngine = boost::make_shared<SimulatedMotionEngine>();
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


void SimulatorItem::selectMotionItems()
{
    impl->selectMotionItems();
}


void SimulatorItemImpl::selectMotionItems()
{
    for(size_t i=0; i < allSimBodies.size(); ++i){
        SimulationBody& simBody = *allSimBodies[i];
        MultiSE3SeqItem* linkPosResultItem = simBody.impl->linkPosResultItem.get();
        if(linkPosResultItem){
            BodyMotionItem* bodyMotionItem = dynamic_cast<BodyMotionItem*>(linkPosResultItem->parentItem());
            if(bodyMotionItem){
                itemTreeView->selectItem(bodyMotionItem);
            } else {
                itemTreeView->selectItem(linkPosResultItem);
            }
        }
    }
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
    motionEngine->clear();
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
    
    bool result = self->initializeSimulation(simBodiesWithBody);

    if(result){

        currentFrame = 0;
        frameAtLastBufferWriting = 0;
        isDoingSimulationLoop = true;
        isWaitingForSimulationToStop = false;
        stopRequested = false;

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
        for(size_t i=0; i < subSimulatorItems.size(); ++i){
            SubSimulatorItem* item = subSimulatorItems.get(i);
            os << (fmt(_("SubSimulatorItem \"%1%\" has been detected.")) % item->name()) << endl;
            if(!subSimulatorItems[i]->initializeSimulation(self)){
                os << (fmt(_("The initialization of \"%1%\" failed.")) % item->name()) << endl;
            }
        }

        for(size_t i=0; i < simBodiesWithBody.size(); ++i){
            simBodiesWithBody[i]->impl->setupDeviceStateRecording();
        }

        for(size_t i=0; i < allSimBodies.size(); ++i){
            SimulationBodyImpl* simBodyImpl = allSimBodies[i]->impl;
            ControllerItemPtr controller = simBodyImpl->controller;
            if(controller){
                const BodyPtr& body = simBodyImpl->body;
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
                if(!ready){
                    controller->setSimulatorItem(0);
                    string message = controller->getMessage();
                    if(!message.empty()){
                        os << message << endl;
                    }
                    simBodyImpl->controller = 0;
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

    QElapsedTimer timer;
    timer.start();

    int frame = 0;

    if(isRealtimeSyncMode){
        const double dt = worldTimeStep;
        const double compensationRatio = (dt > 0.1) ? 0.1 : dt;
        const double dtms = dt * 1000.0;
        double compensatedSimulationTime = 0.0;
        while(true){
            if(!stepSimulationMain() || stopRequested || frame >= maxFrame){
                break;
            }
            double diff = (double)compensatedSimulationTime - timer.elapsed();
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
    } else {
        while(true){
            if(!stepSimulationMain() || stopRequested || frame++ >= maxFrame){
                break;
            }
        }
    }

    actualSimulationTime = (timer.elapsed() / 1000.0);
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
        ControllerItem* controller = simBodyImpl->controller.get();
        if(controller){
            activeControllers.push_back(controller);
            if(simBodyImpl->body && !simBodyImpl->body->isStaticModel()){
                activeSimBodies.push_back(simBody);
            }
        } else if(simBodyImpl->body && !simBodyImpl->body->isStaticModel()){
            activeSimBodies.push_back(simBody);
            hasActiveFreeBodies = true;
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
                ControllerItem* controller = activeControllers[i];
                controller->input();
            }
            isControlRequested = true;
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
        ControllerItem* controller = allSimBodies[i]->impl->controller.get();
        if(controller){
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


TimeSyncItemEnginePtr SimulatorItemImpl::getSimulatedMotionEngine(Item* sourceItem)
{
    SimulatorItem* simulatorItem = dynamic_cast<SimulatorItem*>(sourceItem);
    return simulatorItem ? simulatorItem->impl->motionEngine : TimeSyncItemEnginePtr();
}


bool SimulatorItemImpl::onRealtimeSyncChanged(bool on)
{
    isRealtimeSyncMode = on;
    return true;
}


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
                boost::bind((bool(Selection::*)(int))&Selection::select, &impl->recordingMode, _1));
    putProperty(_("Time range"), impl->timeRangeMode,
                boost::bind((bool(Selection::*)(int))&Selection::select, &impl->timeRangeMode, _1));
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
}


bool SimulatorItem::store(Archive& archive)
{
    archive.write("realtimeSync", impl->isRealtimeSyncMode);
    archive.write("recording", impl->recordingMode.selectedSymbol());
    archive.write("timeRangeMode", impl->timeRangeMode.selectedSymbol());
    archive.write("onlyActiveControlPeriod", impl->isActiveControlPeriodOnlyMode);
    archive.write("timeLength", impl->specifiedTimeLength);
    archive.write("allLinkPositionOutputMode", impl->isAllLinkPositionOutputMode);
    archive.write("deviceStateOutput", impl->isDeviceStateOutputEnabled);
    archive.write("controllerThreads", impl->useControllerThreadsProperty);
    impl->motionEngine->store(archive);
    return true;
}


bool SimulatorItem::restore(const Archive& archive)
{
    bool boolValue;
    string symbol;
    if(archive.read("recording", symbol)){
        impl->recordingMode.select(symbol);
    } else if(archive.read("recording", boolValue)){ // for the compatibility with older version
        impl->recordingMode.select(SimulatorItem::RECORD_FULL);
    }
    if(archive.read("timeRangeMode", symbol)){
        impl->timeRangeMode.select(symbol);
    }
    archive.read("realtimeSync", impl->isRealtimeSyncMode);
    archive.read("onlyActiveControlPeriod", impl->isActiveControlPeriodOnlyMode);
    archive.read("timeLength", impl->specifiedTimeLength);
    setAllLinkPositionOutputMode(archive.get("allLinkPositionOutputMode", impl->isAllLinkPositionOutputMode));
    archive.read("deviceStateOutput", impl->isDeviceStateOutputEnabled);
    archive.read("controllerThreads", impl->useControllerThreadsProperty);
    impl->motionEngine->restore(archive);
    return true;
}
