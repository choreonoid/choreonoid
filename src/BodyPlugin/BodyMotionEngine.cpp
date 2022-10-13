/**
   \file
   \author Shin'ichiro Nakaoka
*/

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

const bool TRACE_FUNCTIONS = false;

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
    shared_ptr<MultiValueSeq> qSeq;
    shared_ptr<MultiSE3Seq> positions;
    std::vector<TimeSyncItemEnginePtr> extraSeqEngines;
    ScopedConnectionSet connections;
        
    Impl(BodyMotionEngine* self, BodyItem* bodyItem, BodyMotionItem* motionItem);
    void updateExtraSeqEngines();
    bool onTimeChanged(double time);
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
    qSeq = motion->jointPosSeq();
    positions = motion->linkPosSeq();
    
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
    bool needFk = false;
    
    if(qSeq){
        const int numAllJoints = std::min(body->numAllJoints(), qSeq->numParts());
        const int numFrames = qSeq->numFrames();
        if(numAllJoints > 0 && numFrames > 0){
            const int frame = qSeq->frameOfTime(time);
            if(frame < numFrames){
                isActive = true;
            }
            const int clampedFrame = qSeq->clampFrameIndex(frame);
            const MultiValueSeq::Frame q = qSeq->frame(clampedFrame);
            for(int i=0; i < numAllJoints; ++i){
                body->joint(i)->q() = q[i];
            }
            if(motionItem->isBodyJointVelocityUpdateEnabled()){
                const double dt = qSeq->timeStep();
                const MultiValueSeq::Frame q_prev = qSeq->frame((clampedFrame == 0) ? 0 : (clampedFrame -1));
                for(int i=0; i < numAllJoints; ++i){
                    body->joint(i)->dq() = (q[i] - q_prev[i]) / dt;
                }
            }
            needFk = true;
        }
    }
    
    if(positions){
        const int numLinks = positions->numParts();
        const int numFrames = positions->numFrames();
        if(numLinks > 0 && numFrames > 0){
            const int frame = positions->frameOfTime(time);
            if(frame < numFrames){
                isActive = true;
            }
            const int clampedFrame = positions->clampFrameIndex(frame);
            for(int i=0; i < numLinks; ++i){
                Link* link = body->link(i);
                const SE3& position = positions->at(clampedFrame, i);
                link->p() = position.translation();
                link->R() = position.rotation().toRotationMatrix();
            }

            if(numLinks >= 2){
                needFk = false;
            }
        }
    }

    if(needFk){
        body->calcForwardKinematics();
    }
    
    for(size_t i=0; i < extraSeqEngines.size(); ++i){
        isActive |= extraSeqEngines[i]->onTimeChanged(time);
    }
    
    bodyItem->notifyKinematicStateChange();
    
    return isActive;
}


double BodyMotionEngine::onPlaybackStopped(double time, bool isStoppedManually)
{
    return impl->onPlaybackStopped(time, isStoppedManually);
}


double BodyMotionEngine::Impl::onPlaybackStopped(double time, bool isStoppedManually)
{
    bodyItem->notifyKinematicStateUpdate(false);

    double lastValidTime = -1.0;
    
    if(qSeq){
        double last = std::max(0.0, qSeq->timeOfFrame(qSeq->numFrames() - 1));
        if(last < time && last > lastValidTime){
            lastValidTime = last;
        }
    }
    if(positions){
        double last = std::max(0.0, positions->timeOfFrame(positions->numFrames() - 1));
        if(last < time && last > lastValidTime){
            lastValidTime = last;
        }
    }
    for(auto& engine : extraSeqEngines){
        double last = engine->onPlaybackStopped(time, isStoppedManually);
        if(last < time && last > lastValidTime){
            lastValidTime = last;
        }
    }

    return lastValidTime;
}
