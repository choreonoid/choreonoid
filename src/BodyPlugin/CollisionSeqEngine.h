#ifndef CNOID_BODYPLUGIN_COLLISION_SEQ_ENGINE_H
#define CNOID_BODYPLUGIN_COLLISION_SEQ_ENGINE_H

#include <cnoid/TimeSyncItemEngine>
#include "exportdecl.h"

namespace cnoid {

class WorldItem;
class CollisionSeqItem;

class CNOID_EXPORT CollisionSeqEngine : public TimeSyncItemEngine
{
public:
    static void initializeClass();
    static TimeSyncItemEngine* create(CollisionSeqItem* item, CollisionSeqEngine* engine0);

    CollisionSeqEngine(WorldItem* worldItem, CollisionSeqItem* collisionSeqItem);
    ~CollisionSeqEngine();

    CollisionSeqItem* collisionSeqItem();

    virtual bool onTimeChanged(double time) override;

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<CollisionSeqEngine> CollisionSeqEnginePtr;

}
#endif
