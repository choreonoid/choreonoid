#ifndef CNOID_BODY_COLLISION_LINK_PAIR_LIST_H
#define CNOID_BODY_COLLISION_LINK_PAIR_LIST_H

#include "CollisionLinkPair.h"
#include <memory>

namespace cnoid {

typedef std::vector<std::shared_ptr<CollisionLinkPair>> CollisionLinkPairList;

}

#endif

