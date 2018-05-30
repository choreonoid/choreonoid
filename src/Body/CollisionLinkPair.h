/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_COLLISION_LINK_PAIR_H
#define CNOID_BODY_COLLISION_LINK_PAIR_H

#include "Body.h"
#include <cnoid/CollisionDetector>
#include <memory>

namespace cnoid {
    
struct CollisionLinkPair
{
    CollisionLinkPair() {
        link[0] = 0;
        link[1] = 0;
    }

    CollisionLinkPair(Body* body1, Link* link1, Body* body2, Link* link2, const CollisionPair& collisionPair)
        : collisions(collisionPair.collisions())
    {
        body[0] = body1;
        body[1] = body2;
        link[0] = link1;
        link[1] = link2;
    }

    bool isSelfCollision() const {
        return (body[0] == body[1]);
    }
        
    BodyPtr body[2];
    Link* link[2];
    std::vector<Collision> collisions;
};
    
typedef std::shared_ptr<CollisionLinkPair> CollisionLinkPairPtr;

}

#endif
