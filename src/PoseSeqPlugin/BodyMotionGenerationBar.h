/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_POSE_SEQ_PLUGIN_BODY_MOTION_GENERATION_BAR_H
#define CNOID_POSE_SEQ_PLUGIN_BODY_MOTION_GENERATION_BAR_H

#include <cnoid/ToolBar>
#include <cnoid/LazySignal>
#include <cnoid/ConnectionSet>
#include <cnoid/Body>
#include <cnoid/BodyMotionItem>
#include "exportdecl.h"

namespace cnoid {

class ExtensionManager;
class TimeBar;
class PoseProvider;
class BodyMotionPoseProvider;
class PoseProviderToBodyMotionConverter;
class BodyMotionGenerationSetupDialog;
class ToggleToolButton;
class Action;

class CNOID_EXPORT BodyMotionGenerationBar : public ToolBar
{
public:
    static void initializeInstance(ExtensionManager* ext);
            
    static BodyMotionGenerationBar* instance();

    virtual ~BodyMotionGenerationBar();

    bool shapeBodyMotion(
        BodyPtr body, PoseProvider* provider, BodyMotionItemPtr motionItem, bool putMessages = false);

    class Balancer
    {
    public:
        virtual bool apply(BodyPtr& body, PoseProvider* provider, BodyMotionItemPtr motionItem, bool putMessages) = 0;
        virtual void storeState(Archive& archive) = 0;
        virtual void restoreState(const Archive& archive) = 0;
        virtual QWidget* panel() = 0;
    };

    //void setBalancer(BalancerFunc func, QWidget* panel);
    void setBalancer(Balancer* balancer);
    void unsetBalancer();

    bool isAutoInterpolationUpdateMode() const;
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
    bool isStealthyStepMode() const;
    double stealthyHeightRatioThresh() const;
    double flatLiftingHeight() const;
    double flatLandingHeight() const;
    double impactReductionHeight() const;
    double impactReductionTime() const;
    bool isAutoZmpAdjustmentMode() const;
    double minZmpTransitionTime() const;
    double zmpCenteringTimeThresh() const;
    double zmpTimeMarginBeforeLifting() const;
    double zmpMaxDistanceFromCenter() const;
    bool isSe3Enabled() const;
    bool isLipSyncMixMode() const;
            
    SignalProxy<void()> sigInterpolationParametersChanged() {
        return sigInterpolationParametersChanged_.signal();
    }
            
private:
    
    BodyMotionPoseProvider* bodyMotionPoseProvider;
    PoseProviderToBodyMotionConverter* poseProviderToBodyMotionConverter;
    
    Balancer* balancer;

    TimeBar* timeBar;
    BodyMotionGenerationSetupDialog* setup;
            
    Action* autoInterpolationUpdateCheck;
    Action* autoGenerationForNewBodyCheck;
    ToolButton* balancerToggle;
    ToolButton* autoGenerationToggle;

    LazySignal< Signal<void()> >sigInterpolationParametersChanged_;

    ConnectionSet interpolationParameterWidgetsConnection;

    BodyMotionGenerationBar();

    void notifyInterpolationParametersChanged();

    void onGenerationButtonClicked();

    bool shapeBodyMotionWithSimpleInterpolation
        (BodyPtr& body, PoseProvider* provider, BodyMotionItemPtr motionItem);
            
    virtual bool storeState(Archive& archive);
    virtual bool restoreState(const Archive& archive);
};

}

#endif
