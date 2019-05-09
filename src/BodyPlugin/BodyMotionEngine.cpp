/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "BodyMotionEngine.h"
#include "BodyItem.h"
#include "BodyMotionItem.h"
#include <cnoid/MenuManager>
#include <cnoid/ConnectionSet>
#include <cnoid/Archive>
#include <map>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

const bool TRACE_FUNCTIONS = false;

typedef std::function<TimeSyncItemEngine*(BodyItem* bodyItem, AbstractSeqItem* seqItem)> ExtraSeqEngineFactory;
typedef map<string, ExtraSeqEngineFactory> ExtraSeqEngineFactoryMap;
ExtraSeqEngineFactoryMap extraSeqEngineFactories;

Action* updateVelocityCheck;

}

static bool storeProperties(Archive& archive)
{
    archive.write("updateJointVelocities", updateVelocityCheck->isChecked());
    return true;
}

static void restoreProperties(const Archive& archive)
{
    updateVelocityCheck->setChecked(archive.get("updateJointVelocities", updateVelocityCheck->isChecked()));
}

namespace cnoid {

class BodyMotionEngineImpl
{
public:
    BodyItemPtr bodyItem;
    BodyMotionItemPtr motionItem;
    BodyPtr body;
    shared_ptr<MultiValueSeq> qSeq;
    shared_ptr<MultiSE3Seq> positions;
    bool calcForwardKinematics;
    std::vector<TimeSyncItemEnginePtr> extraSeqEngines;
    ConnectionSet connections;
        
    BodyMotionEngineImpl(BodyMotionEngine* self, BodyItem* bodyItem, BodyMotionItem* motionItem){

        this->bodyItem = bodyItem;
        body = bodyItem->body();
        this->motionItem = motionItem;
        
        auto motion = motionItem->motion();
        qSeq = motion->jointPosSeq();
        positions = motion->linkPosSeq();
        calcForwardKinematics = !(positions && positions->numParts() > 1);
        
        updateExtraSeqEngines();
        
        connections.add(
            motionItem->sigUpdated().connect(
                std::bind(&BodyMotionEngine::notifyUpdate, self)));
        
        connections.add(
            motionItem->sigExtraSeqItemsChanged().connect(
                std::bind(&BodyMotionEngineImpl::updateExtraSeqEngines, this)));
    }
    
    void updateExtraSeqEngines(){

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

    ~BodyMotionEngineImpl(){
        connections.disconnect();
    }
        
    bool onTimeChanged(double time){

        bool isActive = false;
        bool fkDone = false;
            
        if(qSeq){
            bool isValid = false;
            const int numAllJoints = std::min(body->numAllJoints(), qSeq->numParts());
            const int numFrames = qSeq->numFrames();
            if(numAllJoints > 0 && numFrames > 0){
                const int frame = qSeq->frameOfTime(time);
                isValid = (frame < numFrames);
                const int clampedFrame = qSeq->clampFrameIndex(frame);
                const MultiValueSeq::Frame q = qSeq->frame(clampedFrame);
                for(int i=0; i < numAllJoints; ++i){
                    body->joint(i)->q() = q[i];
                }
                if(updateVelocityCheck->isChecked()){
                    const double dt = qSeq->timeStep();
                    const MultiValueSeq::Frame q_prev = qSeq->frame((clampedFrame == 0) ? 0 : (clampedFrame -1));
                    for(int i=0; i < numAllJoints; ++i){
                        body->joint(i)->dq() = (q[i] - q_prev[i]) / dt;
                    }
                }
            }
            isActive = isValid;
        }

        if(positions){
            bool isValid = false;
            const int numLinks = positions->numParts();
            const int numFrames = positions->numFrames();
            if(numLinks > 0 && numFrames > 0){
                const int frame = positions->frameOfTime(time);
                isValid = (frame < numFrames);
                const int clampedFrame = positions->clampFrameIndex(frame);
                for(int i=0; i < numLinks; ++i){
                    Link* link = body->link(i);
                    const SE3& position = positions->at(clampedFrame, i);
                    link->p() = position.translation();
                    link->R() = position.rotation().toRotationMatrix();
                }
            }
            isActive |= isValid;

            if(positions->numParts() == 1){
                body->calcForwardKinematics(); // FK from the root
                fkDone = true;
            }
        }

        for(size_t i=0; i < extraSeqEngines.size(); ++i){
            isActive |= extraSeqEngines[i]->onTimeChanged(time);
        }

        bodyItem->notifyKinematicStateChange(!fkDone && calcForwardKinematics);

        return isActive;
    }
};


TimeSyncItemEngine* createBodyMotionEngine(Item* sourceItem)
{
    BodyMotionItem* motionItem = dynamic_cast<BodyMotionItem*>(sourceItem);
    if(motionItem){
        BodyItem* bodyItem = motionItem->findOwnerItem<BodyItem>();
        if(bodyItem){
            return new BodyMotionEngine(bodyItem, motionItem);
        }
    }
    return 0;
}

}


BodyMotionEngine::BodyMotionEngine(BodyItem* bodyItem, BodyMotionItem* motionItem)
{
    impl = new BodyMotionEngineImpl(this, bodyItem, motionItem);
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


bool BodyMotionEngine::onTimeChanged(double time)
{
    return impl->onTimeChanged(time);
}


void BodyMotionEngine::initialize(ExtensionManager* ext)
{
    ext->timeSyncItemEngineManger().addEngineFactory(createBodyMotionEngine);

    MenuManager& mm = ext->menuManager();
    mm.setPath("/Options").setPath(N_("Body Motion Engine"));
    updateVelocityCheck = mm.addCheckItem(_("Update Joint Velocities"));

    ext->setProjectArchiver("BodyMotionEngine", storeProperties, restoreProperties);
}


void BodyMotionEngine::addExtraSeqEngineFactory
(const std::string& key, std::function<TimeSyncItemEngine*(BodyItem* bodyItem, AbstractSeqItem* seqItem)> factory)
{
    extraSeqEngineFactories[key] = factory;
}
