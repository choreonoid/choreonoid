/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_BODY_MOTION_POSE_PROVIDER_H
#define CNOID_BODY_BODY_MOTION_POSE_PROVIDER_H

#include "Body.h"
#include "PoseProvider.h"
#include "BodyMotion.h"
#include "ZMPSeq.h"
#include <cnoid/MultiSE3MatrixSeq>
#include "exportdecl.h"

namespace cnoid {

class Link;

class JointPath;
typedef std::shared_ptr<JointPath> JointPathPtr;

class CNOID_EXPORT BodyMotionPoseProvider : public PoseProvider
{
public:
    BodyMotionPoseProvider();
    BodyMotionPoseProvider(Body* body, BodyMotionPtr motion);

    void initialize(Body* body, BodyMotionPtr motion);

    bool updateMotion();

    virtual Body* body() const;
    virtual double beginningTime() const;
    virtual double endingTime() const;
    virtual bool seek(double time);
    virtual bool seek(double time, int waistLinkIndex, const Vector3& waistTranslation);
    virtual int baseLinkIndex() const;
    virtual bool getBaseLinkPosition(Position& out_T) const;
    virtual void getJointPositions(std::vector< boost::optional<double> >& out_q) const;
    virtual boost::optional<Vector3> ZMP() const;

private:
    BodyPtr body_;
    BodyMotionPtr motion;
    ZMPSeqPtr zmpSeq;
    int minNumJoints;
    std::vector<Link*> footLinks;
    std::vector<JointPathPtr> ikPaths;
    MultiSE3MatrixSeqPtr footLinkPositions;
    std::vector<double> qTranslated;
    Vector3 p_waist;
    Matrix3 R_waist;
    Vector3 ZMP_;

    bool seek(double time, int waistLinkIndex, const Vector3& waistTranslation, bool applyWaistTranslation);
};

}

#endif
