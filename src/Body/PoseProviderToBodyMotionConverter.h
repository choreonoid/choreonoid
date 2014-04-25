/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_POSE_PROVIDER_TO_BODY_MOTION_CONVERTER_H_INCLUDED
#define CNOID_BODY_POSE_PROVIDER_TO_BODY_MOTION_CONVERTER_H_INCLUDED

#include "Body.h"
#include "exportdecl.h"

namespace cnoid {

class BodyMotion;
class PoseProvider;

class CNOID_EXPORT PoseProviderToBodyMotionConverter
{
public:
    PoseProviderToBodyMotionConverter();
    void setTimeRange(double lower, double upper);
    void setFullTimeRange();
    void setAllLinkPositionOutput(bool on);
    bool convert(BodyPtr body, PoseProvider* provider, BodyMotion& motion);

private:
    double lowerTime;
    double upperTime;
    bool allLinkPositionOutputMode;
};
}

#endif
