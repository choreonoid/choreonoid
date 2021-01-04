/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_BODY_COLLISION_DETECTOR_UTIL_H
#define CNOID_BODY_BODY_COLLISION_DETECTOR_UTIL_H

#include <cnoid/CollisionDetector>
#include <cnoid/Body>
#include "exportdecl.h"

namespace cnoid {

[[deprecated("Use the BodyCollisionDetector class")]]
CNOID_EXPORT int addBodyToCollisionDetector(Body& body, CollisionDetector& detector, bool enableSelfCollisions = true);

}

#endif
