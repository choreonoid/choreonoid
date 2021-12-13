/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_KINEMATICS_BAR_H
#define CNOID_BODY_PLUGIN_KINEMATICS_BAR_H

#include <cnoid/ToolBar>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT KinematicsBar : public ToolBar
{
public:
    static KinematicsBar* instance();
            
    virtual ~KinematicsBar();

    enum Mode {
        NoKinematics = 0,
        PresetKinematics = 1,
        ForwardKinematics = 2,
        InverseKinematics = 3,
        // deprecated
        AUTO_MODE = PresetKinematics,
        FK_MODE = ForwardKinematics,
        IK_MODE = InverseKinematics
    };
    
    int mode() const;
    bool isForwardKinematicsEnabled() const;
    bool isInverseKinematicsEnabled() const;
    SignalProxy<void()> sigKinematicsModeChanged();

    bool isPositionDraggerEnabled() const;
    bool isFootSnapMode() const;
    void getSnapThresholds(double& distance, double& angle) const;
    bool isJointPositionLimitMode() const;
    bool isPenetrationBlockMode() const;
    double penetrationBlockDepth() const;

    bool isCollisionLinkHighlihtMode() const;
    int collisionDetectionPriority() const;
    SignalProxy<void()> sigCollisionVisualizationChanged();

protected:
    virtual bool storeState(Archive& archive);
    virtual bool restoreState(const Archive& archive);
            
private:
    KinematicsBar();

    class Impl;
    Impl* impl;
};

}

#endif
