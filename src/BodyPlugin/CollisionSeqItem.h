/**
   @file
   @author Shizuko Hattori
*/

#ifndef CNOID_BODY_PLUGIN_COLLISION_SEQ_ITEM_H
#define CNOID_BODY_PLUGIN_COLLISION_SEQ_ITEM_H

#include "CollisionSeq.h"
#include <cnoid/MultiValueSeqItem>
#include "exportdecl.h"

namespace cnoid {

class CollisionSeqItemImpl;

class CNOID_EXPORT CollisionSeqItem : public AbstractMultiSeqItem
{
public :
    static void initislizeClass(ExtensionManager* ext);

    CollisionSeqItem();
    CollisionSeqItem(const CollisionSeqItem& org);
    ~CollisionSeqItem();

    virtual std::shared_ptr<AbstractMultiSeq> abstractMultiSeq();

    const std::shared_ptr<CollisionSeq> collisionSeq() {
        return collisionSeq_;
    }

protected:
    virtual Item* doDuplicate() const;
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

private:
    std::shared_ptr<CollisionSeq> collisionSeq_;
    CollisionSeqItemImpl* impl;
};

typedef ref_ptr<CollisionSeqItem> CollisionSeqItemPtr;

}

#endif
