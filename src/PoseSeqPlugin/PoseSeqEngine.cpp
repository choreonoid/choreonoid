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

    PoseSeqEngine(PoseSeqItem* poseSeqItem, BodyItem* bodyItem) :
        bodyItem(bodyItem) {

        interpolator = poseSeqItem->interpolator();
        bodyMotionGenerationBar = BodyMotionGenerationBar::instance();

        poseSeqItem->sigUpdated().connect(std::bind(&PoseSeqEngine::notifyUpdate, this));
        interpolator->sigUpdated().connect(std::bind(&PoseSeqEngine::notifyUpdate, this));
    }
        
    virtual bool onTimeChanged(double time){

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
    

TimeSyncItemEngine* createPoseSeqEngine(Item* sourceItem)
{
    PoseSeqEngine* engine = 0;
    PoseSeqItem* poseSeqItem = dynamic_cast<PoseSeqItem*>(sourceItem);
    if(poseSeqItem){
        BodyItem* bodyItem = poseSeqItem->findOwnerItem<BodyItem>();
        if(bodyItem){
            engine = new PoseSeqEngine(poseSeqItem, bodyItem);
        }
    }
    return engine;
}

}


void cnoid::initializePoseSeqEngine(ExtensionManager* em)
{
    em->timeSyncItemEngineManger().addEngineFactory(createPoseSeqEngine);
}
