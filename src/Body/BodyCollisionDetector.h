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

class CNOID_EXPORT BodyCollisionDetector : public Referenced
{
public:
    BodyCollisionDetector();
    virtual ~BodyCollisionDetector();

    void setCollisionDetecotr(CollisionDetector* collisionDetector);
    CollisionDetector* collisionDetector();
    
    void addBody(Body* body, bool isSelfCollisionDetectionEnabled);
    bool makeReady();

    void updatePositions();
    void detectCollisions(std::function<void(const CollisionPair& collisionPair)> callback);

    bool isFindClosestPointsAvailable() const;
    double findClosestPoints(Link* link1, Link* link2, Vector3& out_point1, Vector3& out_point2);

private:
    BodyCollisionDetectorImpl* impl;
};

}

#endif
