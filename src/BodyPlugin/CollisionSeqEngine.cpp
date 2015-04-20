/**
   \file
   \author Shizuko Hattori
*/

#include "CollisionSeqEngine.h"
#include "CollisionSeqItem.h"
#include <cnoid/CollisionSeq>
#include <cnoid/SceneCollision>

using namespace std;
using namespace cnoid;

namespace cnoid {

class CollisionSeqEngineImpl
{
public:
    WorldItemPtr worldItem;
    CollisionSeqItemPtr collisionSeqItem;
    CollisionSeqPtr colSeq;
    CollisionSeqEngineImpl(CollisionSeqEngine* self, WorldItem* worldItem, CollisionSeqItem* collisionSeqItem){
        this->worldItem = worldItem;
        this->collisionSeqItem = collisionSeqItem;
        colSeq = collisionSeqItem->collisionSeq();
    }

    virtual bool onTimeChanged(double time){
        bool isValid = false;

        if(colSeq){
            const int numFrames = colSeq->numFrames();
            if(numFrames > 0){
                const int frame = colSeq->frameOfTime(time);
                isValid = (frame < numFrames);
                const int clampedFrame = colSeq->clampFrameIndex(frame);
                const CollisionSeq::Frame collisionPairs0 = colSeq->frame(clampedFrame);
                CollisionLinkPairList& collisionPairs = worldItem->collisions();
                collisionPairs.clear();
                for(int i=0; i<collisionPairs0[0]->size(); i++){
                    collisionPairs.push_back(collisionPairs0[0]->at(i));
                }
            }
        }
        dynamic_cast<SceneCollision*>(worldItem->getScene())->setDirty();
        dynamic_cast<SceneCollision*>(worldItem->getScene())->notifyUpdate(SgUpdate::MODIFIED);

        return isValid;
    }
};

}

TimeSyncItemEngine* createCollisionSeqEngine(Item* sourceItem)
{
    CollisionSeqItem* collisionSeqItem = dynamic_cast<CollisionSeqItem*>(sourceItem);
    if(collisionSeqItem){
        Item* ownerItem = collisionSeqItem->findOwnerItem<Item>();
        WorldItem* worldItem = dynamic_cast<WorldItem*>(ownerItem);
        if(worldItem){
            return new CollisionSeqEngine(worldItem, collisionSeqItem);
        }
    }
    return 0;
}


void CollisionSeqEngine::initialize(ExtensionManager* ext)
{
    ext->timeSyncItemEngineManger().addEngineFactory(createCollisionSeqEngine);
}


CollisionSeqEngine::CollisionSeqEngine(WorldItem* worldItem, CollisionSeqItem* collisionSeqItem)
{
    impl = new CollisionSeqEngineImpl(this, worldItem, collisionSeqItem);
}


CollisionSeqEngine::~CollisionSeqEngine()
{
    delete impl;
}


bool CollisionSeqEngine::onTimeChanged(double time)
{
    return impl->onTimeChanged(time);
}
