/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_POSE_SEQ_PLUGIN_POSE_SEQ_INTERPOLATOR_H
#define CNOID_POSE_SEQ_PLUGIN_POSE_SEQ_INTERPOLATOR_H

#include "PoseSeq.h"
#include <cnoid/PoseProvider>
#include <cnoid/Signal>
#include <memory>
#include "exportdecl.h"

namespace cnoid {

class Mapping;
class PSIImpl;
        
class CNOID_EXPORT PoseSeqInterpolator : public PoseProvider
{
public:
    PoseSeqInterpolator();

    void setBody(Body* body);
    Body* body() const;

    void setLinearInterpolationJoint(int jointId);

    void addFootLink(int linkIndex, const Vector3& soleCenter);

    void setLipSyncShapes(const Mapping& info);
    const std::vector<int>& lipSyncLinkIndices();
            
    void setPoseSeq(PoseSeqPtr seq);

    void setTimeScaleRatio(double ratio);

    double beginningTime() const;
    double endingTime() const;
            
    void enableStealthyStepMode(bool on);
    void setStealthyStepParameters(
        double heightRatioThresh,
        double flatLiftingHeight, double flatLandingHeight,
        double impactReductionHeight, double impactReductionTime);

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

    virtual bool seek(double time);
    virtual bool seek(double time, int waistLinkIndex, const Vector3& waistTranslation);
            
    /**
       @return -1 if base link is not set for the time segment
    */
    int baseLinkIndex() const;
    virtual bool getBaseLinkPosition(Position& out_T) const;

    stdx::optional<double> jointPosition(int jointId) const;
    stdx::optional<Vector3> ZMP() const;

    virtual void getJointPositions(std::vector<stdx::optional<double>>& out_q) const;

private:

    PSIImpl* impl;
};

typedef std::shared_ptr<PoseSeqInterpolator> PoseSeqInterpolatorPtr;

}

#endif
