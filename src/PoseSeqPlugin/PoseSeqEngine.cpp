/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "PoseSeqEngine.h"
#include "PoseSeqItem.h"
#include "BodyMotionGenerationBar.h"
#include <cnoid/BodyItem>
#include <cnoid/TimeSyncItemEngine>

using namespace cnoid;

namespace {

class PoseSeqEngine : public TimeSyncItemEngine
{
public:
    BodyItemPtr bodyItem;
    PoseSeqInterpolatorPtr interpolator;
    BodyMotionGenerationBar* bodyMotionGenerationBar;
    LinkTraverse fkTraverse;
    ScopedConnectionSet connections;

    PoseSeqEngine(PoseSeqItem* poseSeqItem, BodyItem* bodyItem)
        : TimeSyncItemEngine(poseSeqItem),
          bodyItem(bodyItem)
    {
        interpolator = poseSeqItem->interpolator();
        bodyMotionGenerationBar = BodyMotionGenerationBar::instance();

        connections.add(
            poseSeqItem->sigUpdated().connect([this](){ refresh(); }));
        connections.add(
            interpolator->sigUpdated().connect([this](){ refresh(); }));
    }
        
    virtual bool onTimeChanged(double time)
    {
        BodyPtr body = bodyItem->body();

        interpolator->enableLipSyncMix(bodyMotionGenerationBar->isLipSyncMixMode());

        if(interpolator->interpolate(time)){
        
            const int numJoints = body->numJoints();
            for(int i=0; i < numJoints; ++i){
                auto q = interpolator->jointPosition(i);
                if(q){
                    body->joint(i)->q() = (*q);
                }
            }
            int baseLinkIndex = interpolator->baseLinkIndex();
            if(baseLinkIndex >= 0){
                Link* link = body->link(baseLinkIndex);
                interpolator->getBaseLinkPosition(link->T());
                if(link != fkTraverse.rootLink()){
                    fkTraverse.find(link, true, true);
                }
                fkTraverse.calcForwardKinematics();
            }
            
            auto zmp = interpolator->ZMP();
            if(zmp){
                bodyItem->setZmp(*zmp);
            }

            bodyItem->notifyKinematicStateChange(true);
        }
    
        return (time <= interpolator->endingTime());
    }
};

typedef ref_ptr<PoseSeqEngine> PoseSeqEnginePtr;
    

TimeSyncItemEngine* createPoseSeqEngine(PoseSeqItem* item, PoseSeqEngine* engine0)
{
    if(auto bodyItem = item->findOwnerItem<BodyItem>()){
        if(engine0 && engine0->bodyItem == bodyItem){
            return engine0;
        } else {
            return new PoseSeqEngine(item, bodyItem);
        }
    }
    return nullptr;
}

}


void cnoid::initializePoseSeqEngine()
{
    TimeSyncItemEngineManager::instance()
        ->registerFactory<PoseSeqItem, PoseSeqEngine>(createPoseSeqEngine);
}
