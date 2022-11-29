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

class CNOID_EXPORT BodyMotionPoseProvider : public PoseProvider
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    BodyMotionPoseProvider();
    bool setMotion(Body* body, std::shared_ptr<BodyMotion> motion);
    bool updateMotion();

    virtual Body* body() const override;
    virtual double beginningTime() const override;
    virtual double endingTime() const override;
    virtual bool seek(double time) override;
    virtual bool seek(double time, int waistLinkIndex, const Vector3& waistTranslation) override;
    virtual int baseLinkIndex() const override;
    virtual bool getBaseLinkPosition(Isometry3& out_T) const override;
    virtual void getJointDisplacements(std::vector<stdx::optional<double>>& out_q) const override;
    virtual stdx::optional<Vector3> ZMP() const override;

private:
    BodyPtr body_;
    std::shared_ptr<BodyPositionSeq> positionSeq;
    std::shared_ptr<ZMPSeq> zmpSeq;
    int numJointDisplacements;
    bool isReady;
    std::vector<Link*> footLinks;
    std::vector<std::shared_ptr<JointPath>> ikPaths;
    MultiSE3MatrixSeq footLinkPositions;
    std::vector<double> qTranslated;
    Isometry3 T_waist;
    Vector3 ZMP_;

    bool seek(double time, int waistLinkIndex, const Vector3& waistTranslation, bool applyWaistTranslation);
};

}

#endif
