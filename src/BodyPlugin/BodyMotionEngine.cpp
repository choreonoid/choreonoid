#include "BodyMotionEngine.h"
#include "BodyItem.h"
#include "BodyMotionItem.h"
#include <cnoid/ExtensionManager>
#include <cnoid/ConnectionSet>
#include <map>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

typedef std::function<TimeSyncItemEngine*(BodyItem* bodyItem, AbstractSeqItem* seqItem)> ExtraSeqEngineFactory;
typedef map<string, ExtraSeqEngineFactory> ExtraSeqEngineFactoryMap;
ExtraSeqEngineFactoryMap extraSeqEngineFactories;

}


BodyMotionEngineCore::BodyMotionEngineCore(BodyItem* bodyItem)
    : bodyItemRef(bodyItem)
{
    auto& devices = bodyItem->body()->devices();
    int numDevices = devices.size();
    deviceInfos.resize(numDevices);
    for(size_t i=0; i < numDevices; ++i){
        auto& info = deviceInfos[i];
        info.device = devices[i];
        auto& prevState = info.prevState;
        info.connection =
            info.device->sigStateChanged().connect(
                [this, &prevState](){ prevState.reset(); });
    }
}


void BodyMotionEngineCore::updateBodyState(double time, const BodyState& state)
{
    if(auto bodyItem_ = bodyItemRef.lock()){
        bool needFk = updateBodyState_(time, bodyItem_->body(), state);
        bodyItem_->notifyKinematicStateChange(needFk);
    }
}


bool BodyMotionEngineCore::updateBodyState_(double time, Body* body, const BodyState& state)
{
    bool needFk = false;

    // Main body
    auto stateBlock = state.firstBlock();
    if(stateBlock.empty()){
        if(body->existence()){
            body->setExistence(false);
        }
        return false;
    }

    if(!body->existence()){
        body->setExistence(true);
    }
    if(updateSingleBodyState(time, body, stateBlock)){
        needFk = true;
    }

    // Multiplex bodies
    stateBlock = state.nextBlockOf(stateBlock);
    if(!stateBlock){
        body->clearMultiplexBodies();
    } else {
        Body* multiplexBody = body;
        while(stateBlock){
            multiplexBody = multiplexBody->getOrCreateNextMultiplexBody();
            updateSingleBodyState(time, multiplexBody, stateBlock);
            stateBlock = state.nextBlockOf(stateBlock);
        }
        multiplexBody->clearMultiplexBodies();
    }

    return needFk;
}


bool BodyMotionEngineCore::updateSingleBodyState(double time, Body* body, BodyStateBlock bodyStateBlock)
{
    bool needFk = false;

    int numAllLinks = body->numLinks();
    int numLinkPositions = bodyStateBlock.numLinkPositions();
    int numDeviceStates = bodyStateBlock.numDeviceStates();

    int numLinks = std::min(numAllLinks, numLinkPositions);
    for(int i=0; i < numLinks; ++i){
        auto link = body->link(i);
        auto linkPosition = bodyStateBlock.linkPosition(i);
        link->setTranslation(linkPosition.translation());
        link->setRotation(linkPosition.rotation());
    }
    if(numLinks < numAllLinks){
        needFk = true;
    }

    int numAllJoints = body->numAllJoints();
    int numJoints = std::min(numAllJoints, bodyStateBlock.numJointDisplacements());
    if(numJoints > 0){
        auto displacements = bodyStateBlock.jointDisplacements();
        for(int i=0; i < numJoints; ++i){
            body->joint(i)->q() = displacements[i];
        }
    }

    if(!deviceInfos.empty()){
        int deviceIndex = 0;
        int n = deviceInfos.size();
        int m = std::min(n, numDeviceStates);

        while(deviceIndex < m){
            DeviceState* deviceState = bodyStateBlock.deviceState(deviceIndex);
            auto& info = deviceInfos[deviceIndex];
            if(deviceState != info.prevState){
                info.device->copyStateFrom(*deviceState);
                info.connection.block();
                info.device->notifyStateChange();
                info.connection.unblock();
                info.prevState = deviceState;
            }
            info.device->notifyTimeChange(time);
            ++deviceIndex;
        }

        while(deviceIndex < n){
            deviceInfos[deviceIndex++].device->notifyTimeChange(time);
        }
    }

    return needFk;
}


// Note that updating velocities are only supported for the main body
void BodyMotionEngineCore::updateBodyVelocity(Body* body, const BodyState& prevState, double timeStep)
{
    // TODO: set link velocities
    
    int numAllJoints = body->numAllJoints();
    int n = std::min(numAllJoints, prevState.numJointDisplacements());
    int jointIndex = 0;
    if(n > 0){
        auto prevDisplacements = prevState.jointDisplacements();
        while(jointIndex < n){
            auto joint = body->joint(jointIndex);
            joint->dq() = (joint->q() - prevDisplacements[jointIndex]) / timeStep;
            ++jointIndex;
        }
    }
    while(jointIndex < numAllJoints){
        body->joint(jointIndex)->dq() = 0.0;
    }
}


static TimeSyncItemEngine* createBodyMotionEngine(BodyMotionItem* motionItem, BodyMotionEngine* engine0)
{
    if(auto bodyItem = motionItem->findOwnerItem<BodyItem>()){
        if(engine0 && engine0->bodyItem() == bodyItem){
            return engine0;
        } else {
            return new BodyMotionEngine(bodyItem, motionItem);
        }
    }
    return nullptr;
}


void BodyMotionEngine::initializeClass(ExtensionManager* ext)
{
    TimeSyncItemEngineManager::instance()
        ->registerFactory<BodyMotionItem, BodyMotionEngine>(createBodyMotionEngine);
}


void BodyMotionEngine::registerExtraSeqEngineFactory
(const std::string& contentName, std::function<TimeSyncItemEngine*(BodyItem* bodyItem, AbstractSeqItem* seqItem)> factory)
{
    extraSeqEngineFactories[contentName] = factory;
}


void BodyMotionEngine::addExtraSeqEngineFactory
(const std::string& contentName, std::function<TimeSyncItemEngine*(BodyItem* bodyItem, AbstractSeqItem* seqItem)> factory)
{
    registerExtraSeqEngineFactory(contentName, factory);
}


BodyMotionEngine::BodyMotionEngine(BodyItem* bodyItem, BodyMotionItem* motionItem)
    : TimeSyncItemEngine(motionItem),
      core(bodyItem),
      motionItem_(motionItem)
{
    auto motion = motionItem->motion();
    stateSeq = motion->stateSeq();
    
    updateExtraSeqEngines();
    
    connections.add(
        motionItem->sigUpdated().connect(
            [this](){ refresh(); }));
    
    connections.add(
        motionItem->sigExtraSeqItemsChanged().connect(
            [this](){ updateExtraSeqEngines(); }));
}


void BodyMotionEngine::updateExtraSeqEngines()
{
    extraSeqEngines.clear();

    if(auto bodyItem_ = core.bodyItemRef.lock()){
        const int n = motionItem_->numExtraSeqItems();
        for(int i=0; i < n; ++i){
            const string& contentName = motionItem_->extraSeqContentName(i);
            AbstractSeqItem* seqItem = motionItem_->extraSeqItem(i);
            ExtraSeqEngineFactoryMap::iterator q = extraSeqEngineFactories.find(contentName);
            if(q != extraSeqEngineFactories.end()){
                ExtraSeqEngineFactory& createEngine = q->second;
                extraSeqEngines.push_back(createEngine(bodyItem_, seqItem));
            }
        }
    }
}


void BodyMotionEngine::onPlaybackStarted(double time)
{
    if(auto bodyItem_ = core.bodyItemRef.lock()){
        bodyItem_->notifyKinematicStateUpdate(false);
    }
}


bool BodyMotionEngine::onTimeChanged(double time)
{
    bool isActive = false;

    auto bodyItem_ = core.bodyItemRef.lock();
    if(!bodyItem_){
        return false;
    }

    if(!stateSeq->empty()){
        auto body = bodyItem_->body();
        int prevNumMultiplexBodies = body->numMultiplexBodies();
        int frameIndex = stateSeq->clampFrameIndex(stateSeq->frameOfTime(time), isActive);

        bool needFk = core.updateBodyState_(time, body, stateSeq->frame(frameIndex));

        bool doUpdateVelocities = motionItem_->isBodyJointVelocityUpdateEnabled();
        if(doUpdateVelocities){
            auto& prevFrame = stateSeq->frame((frameIndex == 0) ? 0 : (frameIndex -1));
            core.updateBodyVelocity(body, prevFrame, stateSeq->timeStep());
        }
                
        if(needFk){
            body->calcForwardKinematics(doUpdateVelocities);
        }

        if(body->numMultiplexBodies() != prevNumMultiplexBodies){
            // Is it better to define and use a signal specific to multiplex body changes?
            bodyItem_->notifyUpdateWithProjectFileConsistency();
        }
    }
    
    for(size_t i=0; i < extraSeqEngines.size(); ++i){
        if(extraSeqEngines[i]->onTimeChanged(time)){
            isActive = true;
        }
    }
    
    bodyItem_->notifyKinematicStateChange();

    return isActive;
}


double BodyMotionEngine::onPlaybackStopped(double time, bool isStoppedManually)
{
    double lastValidTime = -1.0;

    if(auto bodyItem_ = core.bodyItemRef.lock()){

        bodyItem_->notifyKinematicStateUpdate(false);
    
        double last = std::max(0.0, stateSeq->timeOfFrame(stateSeq->numFrames() - 1));
        if(last < time && last > lastValidTime){
            lastValidTime = last;
        }
        for(auto& engine : extraSeqEngines){
            double last = engine->onPlaybackStopped(time, isStoppedManually);
            if(last < time && last > lastValidTime){
                lastValidTime = last;
            }
        }
    }

    return lastValidTime;
}
