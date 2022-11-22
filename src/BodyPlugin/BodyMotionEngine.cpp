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

namespace cnoid {

class BodyMotionEngine::Impl
{
public:
    BodyItemPtr bodyItem;
    BodyMotionItemPtr motionItem;
    BodyPtr body;
    std::shared_ptr<BodyPositionSeq> positionSeq;
    std::vector<TimeSyncItemEnginePtr> extraSeqEngines;
    ScopedConnectionSet connections;
        
    Impl(BodyMotionEngine* self, BodyItem* bodyItem, BodyMotionItem* motionItem);
    void updateExtraSeqEngines();
    bool onTimeChanged(double time);
    static bool setBodyPosition(Body* body, BodyPositionSeqFrameBlock frameBlock);
    void setBodyVelocities(Body* body, int frameIndex);
    double onPlaybackStopped(double time, bool isStoppedManually);
};

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


void BodyMotionEngine::addExtraSeqEngineFactory
(const std::string& key, std::function<TimeSyncItemEngine*(BodyItem* bodyItem, AbstractSeqItem* seqItem)> factory)
{
    extraSeqEngineFactories[key] = factory;
}


BodyMotionEngine::BodyMotionEngine(BodyItem* bodyItem, BodyMotionItem* motionItem)
    : TimeSyncItemEngine(motionItem)
{
    impl = new Impl(this, bodyItem, motionItem);
}


BodyMotionEngine::Impl::Impl(BodyMotionEngine* self, BodyItem* bodyItem, BodyMotionItem* motionItem)
    : bodyItem(bodyItem),
      motionItem(motionItem)
{
    body = bodyItem->body();
    
    auto motion = motionItem->motion();
    positionSeq = motion->positionSeq();
    
    updateExtraSeqEngines();
    
    connections.add(
        motionItem->sigUpdated().connect(
            [self](){ self->refresh(); }));
    
    connections.add(
        motionItem->sigExtraSeqItemsChanged().connect(
            [this](){ updateExtraSeqEngines(); }));
}


BodyMotionEngine::~BodyMotionEngine()
{
    delete impl;
}


BodyItem* BodyMotionEngine::bodyItem()
{
    return impl->bodyItem.get();
}


BodyMotionItem* BodyMotionEngine::motionItem()
{
    return impl->motionItem.get();
}


void BodyMotionEngine::Impl::updateExtraSeqEngines()
{
    extraSeqEngines.clear();
    
    const int n = motionItem->numExtraSeqItems();
    for(int i=0; i < n; ++i){
        const string& key = motionItem->extraSeqKey(i);
        AbstractSeqItem* seqItem = motionItem->extraSeqItem(i);
        ExtraSeqEngineFactoryMap::iterator q = extraSeqEngineFactories.find(key);
        if(q != extraSeqEngineFactories.end()){
            ExtraSeqEngineFactory& createEngine = q->second;
            extraSeqEngines.push_back(createEngine(bodyItem, seqItem));
        }
    }
}


void BodyMotionEngine::onPlaybackStarted(double time)
{
    impl->bodyItem->notifyKinematicStateUpdate(false);
}


bool BodyMotionEngine::onTimeChanged(double time)
{
    return impl->onTimeChanged(time);
}


bool BodyMotionEngine::Impl::onTimeChanged(double time)
{
    bool isActive = false;

    if(!positionSeq->empty()){
        int prevNumMultiplexBodies = body->numMultiplexBodies();
        int frameIndex = positionSeq->clampFrameIndex(positionSeq->frameOfTime(time), isActive);
        auto& frame = positionSeq->frame(frameIndex);

        bool needFk = updateBodyPositionWithBodyPositionSeqFrame(body, frame);

        bool doUpdateVelocities = motionItem->isBodyJointVelocityUpdateEnabled();
        if(doUpdateVelocities){
            setBodyVelocities(body, frameIndex);
        }
                
        if(needFk){
            body->calcForwardKinematics(doUpdateVelocities);
        }
        
        if(body->numMultiplexBodies() != prevNumMultiplexBodies){
            // Is it better to define and use a signal specific to multiplex body changes?
            bodyItem->notifyUpdate();
        }
    }
    
    for(size_t i=0; i < extraSeqEngines.size(); ++i){
        if(extraSeqEngines[i]->onTimeChanged(time)){
            isActive = true;
        }
    }
    
    bodyItem->notifyKinematicStateChange();

    return isActive;
}


bool BodyMotionEngine::updateBodyPositionWithBodyPositionSeqFrame(Body* body, const BodyPositionSeqFrame& frame)
{
    bool needFk = false;

    // Main body
    auto frameBlock = frame.firstBlock();
    if(Impl::setBodyPosition(body, frameBlock)){
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
            Impl::setBodyPosition(multiplexBody, frameBlock);
            frameBlock = frame.nextBlockOf(frameBlock);
        }
        multiplexBody->clearMultiplexBodies();
    }

    return needFk;
}


bool BodyMotionEngine::Impl::setBodyPosition(Body* body, BodyPositionSeqFrameBlock frameBlock)
{
    bool needFk = false;

    int numAllLinks = body->numLinks();
    int numLinkPositions = frameBlock.numLinkPositions();
    if(numLinkPositions == 0){
        body->rootLink()->translation().x() = 1.0e10; // Hide the body
        if(numAllLinks >= 2){
            needFk = true;
        }
    } else {
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
void BodyMotionEngine::Impl::setBodyVelocities(Body* body, int frameIndex)
{
    // TODO: set link velocities
    
    auto prevFrame = positionSeq->frame((frameIndex == 0) ? 0 : (frameIndex -1));
    int numAllJoints = body->numAllJoints();
    int n = std::min(numAllJoints, prevFrame.numJointDisplacements());
    int jointIndex = 0;
    if(n > 0){
        auto prevDisplacements = prevFrame.jointDisplacements();
        const double dt = positionSeq->timeStep();
        while(jointIndex < n){
            auto joint = body->joint(jointIndex);
            joint->dq() = (joint->q() - prevDisplacements[jointIndex]) / dt;
            ++jointIndex;
        }
    }
    while(jointIndex < numAllJoints){
        body->joint(jointIndex)->dq() = 0.0;
    }
}


double BodyMotionEngine::onPlaybackStopped(double time, bool isStoppedManually)
{
    return impl->onPlaybackStopped(time, isStoppedManually);
}


double BodyMotionEngine::Impl::onPlaybackStopped(double time, bool isStoppedManually)
{
    bodyItem->notifyKinematicStateUpdate(false);

    double lastValidTime = -1.0;
    
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

    return lastValidTime;
}
