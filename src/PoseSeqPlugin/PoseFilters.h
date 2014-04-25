/**
   @file
   @author Shin'ichiro NAKAOKA
*/

#ifndef CNOID_CHOREOGRAPHY_POSE_FILTERS_H_INCLUDED
#define CNOID_CHOREOGRAPHY_POSE_FILTERS_H_INCLUDED

#include "PoseSeq.h"
#include "exportdecl.h"

namespace cnoid {

CNOID_EXPORT void adjustStepPositions(
    PoseSeqPtr seq, const std::vector<int>& footLinkIndices, PoseSeq::iterator origin);

CNOID_EXPORT void flipPoses(PoseSeqPtr seq, BodyPtr body);
        
CNOID_EXPORT void adjustWaistHeight(
    PosePtr pose, int waistLinkIndex, const std::vector<int>& footLinkIndices, double offset);

CNOID_EXPORT void rotateYawOrientations(
    PoseSeqPtr seq, PoseSeq::iterator begin, const Vector3& center, double angle);
}


#endif


