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
class BodyCollisionDetectorImpl;

class CNOID_EXPORT BodyCollisionDetector
{
public:
    BodyCollisionDetector();
    virtual ~BodyCollisionDetector();

    void setCollisionDetector(CollisionDetector* collisionDetector);
    CollisionDetector* collisionDetector();

    void clearBodies();
    void addBody(Body* body, bool isSelfCollisionDetectionEnabled);
    void addBody(Body* body, bool isSelfCollisionDetectionEnabled,
                 std::function<Referenced*(Link* link)> getObjectAssociatedWithLink);
    bool makeReady();

    void updatePositions();
    void updatePositions(std::function<void(Referenced* object, Position*& out_Position)> positionQuery);
    void detectCollisions(std::function<void(const CollisionPair& collisionPair)> callback);

    bool isFindClosestPointsAvailable() const;
    double findClosestPoints(Link* link1, Link* link2, Vector3& out_point1, Vector3& out_point2);

private:
    BodyCollisionDetectorImpl* impl;
};

}

#endif
