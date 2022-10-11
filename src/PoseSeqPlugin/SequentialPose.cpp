#include "SequentialPose.h"
#include <cnoid/CloneMap>

using namespace std;
using namespace cnoid;


SequentialPose::SequentialPose(AbstractPose* pose, double time)
    : pose_(pose),
      time_(time)
{
    maxTransitionTime_ = 0.0;
}


SequentialPose::SequentialPose(const SequentialPose& org, CloneMap* cloneMap)
{
    pose_ = CloneMap::getClone(org.pose_, cloneMap);
    time_ = org.time_;
    maxTransitionTime_ = org.maxTransitionTime_;
}
