/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_BODY_COLLISION_DETECTOR_UTIL_H_INCLUDED
#define CNOID_BODY_BODY_COLLISION_DETECTOR_UTIL_H_INCLUDED

#include <cnoid/CollisionDetector>
#include <cnoid/Body>
#include "exportdecl.h"

namespace cnoid {

CNOID_EXPORT int addBodyToCollisionDetector(Body& body, CollisionDetector& detector, bool enableSelfCollisions = true);

}

#endif
