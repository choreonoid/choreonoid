#ifndef CNOID_BODY_COLLISION_LINK_PAIR_H
#define CNOID_BODY_COLLISION_LINK_PAIR_H

#include "Link.h"
#include <cnoid/Collision>

namespace cnoid {

class CollisionLinkPair
{
public:
    CollisionLinkPair() = default;
    CollisionLinkPair(const CollisionLinkPair& org) = default;
    CollisionLinkPair(CollisionLinkPair&& org) = default;
    CollisionLinkPair(Link* link1, Link* link2){
        setLinkPair(link1, link2);
    }
    CollisionLinkPair(Link* link1, Link* link2, const std::vector<Collision>& collisions)
        : collisions_(collisions) {
        setLinkPair(link1, link2);
    }
    CollisionLinkPair(Link* link1, Link* link2, std::vector<Collision>&& collisions)
        : collisions_(collisions) {
        setLinkPair(link1, link2);
    }
    
    CollisionLinkPair& operator=(const CollisionLinkPair& rhs) = default;

    Link* link(int which) const { return links[which]; }
    void setLink(int which, Link* link){ links[which] = link; }
    void setLinkPair(Link* link1, Link* link2){
        links[0] = link1;
        links[1] = link2;
    }
    Body* body(int which) const { return links[which]->body(); }

    Collision& collision(int i) { return collisions_[i]; }
    const Collision& collision(int i) const { return collisions_[i]; }
    std::vector<Collision>& collisions() { return collisions_; }
    const std::vector<Collision>& collisions() const { return collisions_; }
    void setCollisions(const std::vector<Collision>& collisions){
        collisions_ = collisions;
    }
    void clearCollisions(){ collisions_.clear(); }
    bool empty() const { return collisions_.empty(); }
    int numCollisions() const { return collisions_.size(); }

    bool isSelfCollision() const {
        return (links[0]->body() == links[1]->body());
    }

private:
    LinkPtr links[2];
    std::vector<Collision> collisions_;
};

}

#endif
