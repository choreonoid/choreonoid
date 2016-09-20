/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_BODY_MOTION_UTIL_H
#define CNOID_BODY_BODY_MOTION_UTIL_H

#include <cnoid/NullOut>
#include <iosfwd>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class Body;
class BodyMotion;
class Vector3Seq;
class MultiSE3Seq;
class MultiValueSeq;
class PoseProvider;
class AccelerationSensor;

CNOID_EXPORT bool loadHrpsysSeqFileSet(
    BodyMotion& motion, const std::string& filename, std::ostream& os);
        
CNOID_EXPORT bool saveHrpsysSeqFileSet(
    BodyMotion& motion, Body* body, const std::string& filename, std::ostream& os);

CNOID_EXPORT void calcLinkAccSeq(
    MultiSE3Seq& linkPosSeq, AccelerationSensor* gsens, int frameBegin, int numFrames, Vector3Seq& out_accSeq);

CNOID_EXPORT bool applyVelocityLimitFilter(
    MultiValueSeq& seq, Body* body, std::ostream& os = nullout());

CNOID_EXPORT bool applyVelocityLimitFilter2(MultiValueSeq& seq, int part, double absLimit);

CNOID_EXPORT bool applyVelocityLimitFilterDummy();
    
CNOID_EXPORT bool applyPollardVelocityLimitFilter(
    MultiValueSeq& seq, Body* body, double ks, std::ostream& os = nullout());
    
CNOID_EXPORT void applyGaussianFilter(
    MultiValueSeq& seq, double sigma, int range, std::ostream& os = nullout());
    
CNOID_EXPORT void applyRangeLimitFilter(
    MultiValueSeq& seq, Body* body, double limitGrad, double edgeGradRatio, double margin,
    std::ostream& os = nullout());
}

#endif
