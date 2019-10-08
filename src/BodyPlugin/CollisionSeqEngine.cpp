/**
   \file
   \author Shizuko Hattori
*/

#include "CollisionSeq.h"
#include "CollisionSeqEngine.h"
#include "WorldItem.h"
#include "CollisionSeqItem.h"
#include <cnoid/ExtensionManager>
#include <cnoid/SceneCollision>

using namespace std;
using namespace cnoid;

namespace cnoid {

class CollisionSeqEngineImpl
{
public:
    WorldItemPtr worldItem;
    CollisionSeqItemPtr collisionSeqItem;
    shared_ptr<CollisionSeq> colSeq;
    CollisionSeqEngineImpl(CollisionSeqEngine* self, WorldItem* worldItem, CollisionSeqItem* collisionSeqItem){
        this->worldItem = worldItem;
        this->collisionSeqItem = collisionSeqItem;
        colSeq = collisionSeqItem->collisionSeq();
    }

    bool onTimeChanged(double time){
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
                for(size_t i=0; i < collisionPairs0[0]->size(); i++){
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


void CollisionSeqEngine::initializeClass(ExtensionManager* ext)
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


CollisionSeqItem* CollisionSeqEngine::collisionSeqItem()
{
    return impl->collisionSeqItem.get();
}


bool CollisionSeqEngine::onTimeChanged(double time)
{
    return impl->onTimeChanged(time);
}
