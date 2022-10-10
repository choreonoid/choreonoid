#ifndef CNOID_POSE_SEQ_PLUGIN_POSE_FILTERS_H
#define CNOID_POSE_SEQ_PLUGIN_POSE_FILTERS_H

#include "PoseSeq.h"
#include <cnoid/EigenTypes>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class PoseSeq;
class KeyPose;
class Body;

CNOID_EXPORT bool adjustStepPositions(
    PoseSeq* seq, const std::vector<int>& footLinkIndices, PoseSeq::iterator origin);

CNOID_EXPORT bool flipPoses(PoseSeq* seq, Body* body);
        
CNOID_EXPORT void adjustWaistHeight(
    KeyPose* pose, int waistLinkIndex, const std::vector<int>& footLinkIndices, double offset);

CNOID_EXPORT void rotateYawOrientations(
    PoseSeq* seq, PoseSeq::iterator begin, const Vector3& center, double angle);
}

#endif
