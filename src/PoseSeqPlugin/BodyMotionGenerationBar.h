#ifndef CNOID_POSE_SEQ_PLUGIN_BODY_MOTION_GENERATION_BAR_H
#define CNOID_POSE_SEQ_PLUGIN_BODY_MOTION_GENERATION_BAR_H

#include <cnoid/ToolBar>
#include <functional>
#include "exportdecl.h"

namespace cnoid {

class ExtensionManager;
class BodyItem;
class Body;
class PoseProvider;
class BodyMotionItem;

class CNOID_EXPORT BodyMotionGenerationBar : public ToolBar
{
public:
    static void initializeInstance(ExtensionManager* ext);
            
    static BodyMotionGenerationBar* instance();

    BodyMotionGenerationBar();
    ~BodyMotionGenerationBar();

    bool shapeBodyMotion(
        BodyItem* bodyItem, PoseProvider* provider, BodyMotionItem* outputMotionItem, bool putMessages = false);

    class Balancer
    {
    public:
        virtual QWidget* panel() = 0;
        virtual bool apply(BodyItem* bodyItem, PoseProvider* provider, BodyMotionItem* outputMotionItem, bool putMessages) = 0;
        virtual void storeState(Archive* archive) = 0;
        virtual void restoreState(const Archive* archive) = 0;
    };

    void setBalancer(Balancer* balancer);
    void unsetBalancer();

    void addMotionFilter(
        const char* key,
        const QIcon& buttonIcon,
        const char* toolTip,
        std::function<bool(BodyItem* bodyItem, BodyMotionItem* motionItem)> filter);

    SignalProxy<void()> sigInterpolationParametersChanged();

    bool isBalancerEnabled() const;
    bool isAutoGenerationMode() const;
    bool isAutoGenerationForNewBodyEnabled() const;
            
    double timeScaleRatio() const;
    double preInitialDuration() const;
    double postFinalDuration() const;
        
    double timeToStartBalancer() const;
    int balancerIterations() const;
    int boundaryConditionType() const;
    int boundarySmootherType() const;
    double boundarySmootherTime() const;
    double dynamicsTimeRatio() const;
    bool isTimeBarRangeOnly() const;
    int initialWaistTrajectoryMode() const;
    
    int stepTrajectoryAdjustmentMode() const;

    // Stealthy step mode parameters
    double stealthyHeightRatioThresh() const;
    double flatLiftingHeight() const;
    double flatLandingHeight() const;
    double impactReductionHeight() const;
    double impactReductionTime() const;

    // Toe step mode parameters
    double toeContactTime() const;
    double toeContactAngle() const;
    
    bool isAutoZmpAdjustmentMode() const;
    double minZmpTransitionTime() const;
    double zmpCenteringTimeThresh() const;
    double zmpTimeMarginBeforeLifting() const;
    double zmpMaxDistanceFromCenter() const;
    bool isSe3Enabled() const;
    bool isLipSyncMixMode() const;
            
    class Impl;

protected:
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;
            
private:
    Impl* impl;
};

}

#endif
