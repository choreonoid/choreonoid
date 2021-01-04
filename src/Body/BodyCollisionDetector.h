/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_BODY_COLLISION_DETECTOR_H
#define CNOID_BODY_BODY_COLLISION_DETECTOR_H

#include <cnoid/CollisionDetector>
#include "exportdecl.h"

namespace cnoid {

class Body;
class Link;

class CNOID_EXPORT BodyCollisionDetector
{
public:
    BodyCollisionDetector();
    virtual ~BodyCollisionDetector();

    void setCollisionDetector(CollisionDetector* collisionDetector);
    CollisionDetector* collisionDetector();

    void clearBodies();

    void enableGeometryHandleMap(bool on);
    stdx::optional<CollisionDetector::GeometryHandle> findGeometryHandle(Link* link);

    void addBody(Body* body, bool isSelfCollisionDetectionEnabled);
    void addBody(Body* body, bool isSelfCollisionDetectionEnabled,
                 std::function<Referenced*(Link* link, CollisionDetector::GeometryHandle geometry)> getObjectAssociatedWithLink);
    bool makeReady();

    void updatePositions();
    void updatePositions(std::function<void(Referenced* object, Isometry3*& out_position)> positionQuery);

    void detectCollisions(std::function<void(const CollisionPair& collisionPair)> callback);

private:
    class Impl;
    Impl* impl;
};

}

#endif
