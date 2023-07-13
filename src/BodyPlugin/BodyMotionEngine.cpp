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

}


void BodyMotionEngineCore::updateBodyPosition(const BodyPositionSeqFrame& frame)
{
    if(auto bodyItem_ = bodyItemRef.lock()){
        bool needFk = updateBodyPosition_(bodyItem_->body(), frame);
        bodyItem_->notifyKinematicStateChange(needFk);
    }
}


bool BodyMotionEngineCore::updateBodyPosition_(Body* body, const BodyPositionSeqFrame& frame)
{
    bool needFk = false;

    // Main body
    auto frameBlock = frame.firstBlock();
    if(updateSingleBodyPosition(body, frameBlock, true)){
        needFk = true;
    }

    // Multiplex bodies
    frameBlock = frame.nextBlockOf(frameBlock);
    if(!frameBlock){
        body->clearMultiplexBodies();
    } else {
        Body* multiplexBody = body;
        while(frameBlock){
            multiplexBody = multiplexBody->getOrCreateNextMultiplexBody();
            updateSingleBodyPosition(multiplexBody, frameBlock, false);
            frameBlock = frame.nextBlockOf(frameBlock);
        }
        multiplexBody->clearMultiplexBodies();
    }

    return needFk;
}


bool BodyMotionEngineCore::updateSingleBodyPosition(Body* body, BodyPositionSeqFrameBlock frameBlock, bool isMainBody)
{
    bool needFk = false;

    int numAllLinks = body->numLinks();
    int numLinkPositions = frameBlock.numLinkPositions();
    if(numLinkPositions == 0){
        if(body->existence() && isMainBody){
            body->setExistence(false);
        }
    } else {
        if(!body->existence() && isMainBody){
            body->setExistence(true);
        }
        int numLinks = std::min(numAllLinks, numLinkPositions);
        for(int i=0; i < numLinks; ++i){
            auto link = body->link(i);
            auto linkPosition = frameBlock.linkPosition(i);
            link->setTranslation(linkPosition.translation());
            link->setRotation(linkPosition.rotation());
        }
        if(numLinks < numAllLinks){
            needFk = true;
        }
    }

    int numAllJoints = body->numAllJoints();
    int numJoints = std::min(numAllJoints, frameBlock.numJointDisplacements());
    if(numJoints > 0){
        auto displacements = frameBlock.jointDisplacements();
        for(int i=0; i < numJoints; ++i){
            body->joint(i)->q() = displacements[i];
        }
    }

    return needFk;
}


// Note that updating velocities are only supported for the main body
void BodyMotionEngineCore::updateBodyVelocity(Body* body, const BodyPositionSeqFrame& prevFrame, double timeStep)
{
    // TODO: set link velocities
    
    int numAllJoints = body->numAllJoints();
    int n = std::min(numAllJoints, prevFrame.numJointDisplacements());
    int jointIndex = 0;
    if(n > 0){
        auto prevDisplacements = prevFrame.jointDisplacements();
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
    positionSeq = motion->positionSeq();
    
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

    if(!positionSeq->empty()){
        auto body = bodyItem_->body();
        int prevNumMultiplexBodies = body->numMultiplexBodies();
        int frameIndex = positionSeq->clampFrameIndex(positionSeq->frameOfTime(time), isActive);

        bool needFk = core.updateBodyPosition_(body, positionSeq->frame(frameIndex));

        bool doUpdateVelocities = motionItem_->isBodyJointVelocityUpdateEnabled();
        if(doUpdateVelocities){
            auto& prevFrame = positionSeq->frame((frameIndex == 0) ? 0 : (frameIndex -1));
            core.updateBodyVelocity(body, prevFrame, positionSeq->timeStep());
        }
                
        if(needFk){
            body->calcForwardKinematics(doUpdateVelocities);
        }
        
        if(body->numMultiplexBodies() != prevNumMultiplexBodies){
            // Is it better to define and use a signal specific to multiplex body changes?
            bodyItem_->notifyUpdate();
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
    
        double last = std::max(0.0, positionSeq->timeOfFrame(positionSeq->numFrames() - 1));
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
