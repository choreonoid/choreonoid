#ifndef CNOID_BODY_MOTION_FILTER_PLUGIN_YAW_MOMENT_COMPENSATION_FILTER_H
#define CNOID_BODY_MOTION_FILTER_PLUGIN_YAW_MOMENT_COMPENSATION_FILTER_H

#include <cnoid/MultiValueSeq>
#include <cnoid/Vector3Seq>
#include <memory>

namespace cnoid {

class Body;
class BodyMotion;
class MessageOut;

class YawMomentCompensationFilter
{
public:
    YawMomentCompensationFilter();
    ~YawMomentCompensationFilter();
    void setMessageSink(MessageOut* ms);

    void clearJointWeights();
    void setJointWeight(int jointId, bool isEnabled, double weight = 1.0, double vLimitRatio = 1.0);
    void setRecoveryForceGains(double kp, double kd);
    void setRecoveryCoefficients(double omega, double zita);

    enum JointDisplacementRangeMode {
        InputMotionJointDisplacementRange,
        ModelJointDisplacementRange
    };
    void setJointDisplacementRangeMode(JointDisplacementRangeMode mode);
    void setJointDisplacementRangeConstraintEnabled(bool on);

    void setOpposingAdjustmentJoint(int adjustmentJointId, int referenceJointId);
    void resetOpposingAdjustmentJoint();
    
    void setFrictionCoefficient(double c);
    void setSoleRadius(double r);
    void setDynamicNormalForceMode(bool on);
    void setConstantNormalForceFactor(double factor);
    void enableMomentOfBothFeetSupporting(bool on);
    void setYawMomentSeqOutput(std::shared_ptr<MultiValueSeq> yawMomentSeq);
    void setCopSeqOutput(std::shared_ptr<Vector3Seq> copSeq);

    bool apply(Body* body, BodyMotion& motion);

    static double defaultFrictionCoefficient();
    static double defaultSoleRadius();
    static double defaultConstantNormalForceFactor();
    static double defaultRecoveryForcePGain();
    static double defaultRecoveryForceDGain();

private:
    class Impl;
    Impl* impl;
};

}

#endif
