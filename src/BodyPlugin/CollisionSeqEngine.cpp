#include "CollisionSeq.h"
#include "CollisionSeqEngine.h"
#include "WorldItem.h"
#include "CollisionSeqItem.h"
#include <cnoid/SceneCollision>

using namespace std;
using namespace cnoid;

namespace cnoid {

class CollisionSeqEngine::Impl
{
public:
    weak_ref_ptr<WorldItem> worldItemRef;
    CollisionSeqItem* collisionSeqItem;
    shared_ptr<CollisionSeq> colSeq;

    Impl(CollisionSeqEngine* self, WorldItem* worldItem, CollisionSeqItem* collisionSeqItem);
    bool onTimeChanged(double time);
};

}


TimeSyncItemEngine* CollisionSeqEngine::create(CollisionSeqItem* item, CollisionSeqEngine* engine0)
{
    if(auto worldItem = item->findOwnerItem<WorldItem>()){
        if(engine0 && engine0->impl->worldItemRef.lock() == worldItem){
            return engine0;
        } else {
            return new CollisionSeqEngine(worldItem, item);
        }
    }
    return nullptr;
}


void CollisionSeqEngine::initializeClass()
{
    TimeSyncItemEngineManager::instance()
        ->registerFactory<CollisionSeqItem, CollisionSeqEngine>(CollisionSeqEngine::create);
}


CollisionSeqEngine::CollisionSeqEngine(WorldItem* worldItem, CollisionSeqItem* collisionSeqItem)
    : TimeSyncItemEngine(collisionSeqItem)
{
    impl = new Impl(this, worldItem, collisionSeqItem);
}


CollisionSeqEngine::Impl::Impl(CollisionSeqEngine* self, WorldItem* worldItem, CollisionSeqItem* collisionSeqItem)
    : worldItemRef(worldItem),
      collisionSeqItem(collisionSeqItem)
{
    colSeq = collisionSeqItem->collisionSeq();
}


CollisionSeqEngine::~CollisionSeqEngine()
{
    delete impl;
}


CollisionSeqItem* CollisionSeqEngine::collisionSeqItem()
{
    return impl->collisionSeqItem;
}


bool CollisionSeqEngine::onTimeChanged(double time)
{
    return impl->onTimeChanged(time);
}


bool CollisionSeqEngine::Impl::onTimeChanged(double time)
{
    bool isValid = false;

    auto worldItem = worldItemRef.lock();

    if(worldItem){
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
    }

    return isValid;
}
