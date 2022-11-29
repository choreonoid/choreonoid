#ifndef CNOID_POSE_SEQ_PLUGIN_POSE_SEQ_INTERPOLATOR_H
#define CNOID_POSE_SEQ_PLUGIN_POSE_SEQ_INTERPOLATOR_H

#include "PoseSeq.h"
#include <cnoid/PoseProvider>
#include <cnoid/Signal>
#include <memory>
#include "exportdecl.h"

namespace cnoid {

class Mapping;
        
class CNOID_EXPORT PoseSeqInterpolator : public PoseProvider
{
public:
    PoseSeqInterpolator();

    void setBody(Body* body);
    virtual Body* body() const override;

    void setLinearInterpolationJoint(int jointId);

    void setLipSyncShapes(const Mapping& info);
    const std::vector<int>& lipSyncLinkIndices();
            
    void setPoseSeq(PoseSeq* seq);

    void setTimeScaleRatio(double ratio);

    virtual double beginningTime() const override;
    virtual double endingTime() const override;

    enum StepTrajectoryAdjustmentMode {
        NoStepAdjustmentMode,
        StealthyStepMode,
        ToeStepMode
    };

    int stepTrajectoryAdjustmentMode() const;
    void setStepTrajectoryAdjustmentMode(int mode);

    void setStealthyStepParameters(
        double heightRatioThresh,
        double flatLiftingHeight, double flatLandingHeight,
        double impactReductionHeight, double impactReductionTime);

    void setToeStepParameters(double toeContactAngle, double toeContactTime);

    void enableAutoZmpAdjustmentMode(bool on);
    void setZmpAdjustmentParameters(
        double minTransitionTime, double centeringTimeThresh,
        double timeMarginBeforeLifting, double maxDistanceFromCenter);

    void enableLipSyncMix(bool on);

    /**
       This function has not been implemented yet.
    */
    void setAutoUpdateMode(bool on);
            
    bool update();

    SignalProxy<void()> sigUpdated();
            
    bool interpolate(double time);
    bool interpolate(double time, int waistLinkIndex, const Vector3& waistTranslation);

    virtual bool seek(double time) override;
    virtual bool seek(double time, int waistLinkIndex, const Vector3& waistTranslation) override;
            
    /**
       @return -1 if base link is not set for the time segment
    */
    virtual int baseLinkIndex() const override;
    virtual bool getBaseLinkPosition(Isometry3& out_T) const override;

    stdx::optional<double> jointPosition(int jointId) const;
    virtual stdx::optional<Vector3> ZMP() const override;

    virtual void getJointDisplacements(std::vector<stdx::optional<double>>& out_q) const override;

private:
    class Impl;
    Impl* impl;
};

typedef std::shared_ptr<PoseSeqInterpolator> PoseSeqInterpolatorPtr;

}

#endif
