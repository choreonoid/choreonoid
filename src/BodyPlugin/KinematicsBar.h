/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_KINEMATICS_BAR_H
#define CNOID_BODY_PLUGIN_KINEMATICS_BAR_H

#include <cnoid/ToolBar>
#include "exportdecl.h"

namespace cnoid {

class KinematicsBarSetupDialog;

class CNOID_EXPORT KinematicsBar : public ToolBar
{
public:
    static KinematicsBar* instance();
            
    virtual ~KinematicsBar();

    enum Mode { AUTO_MODE, FK_MODE, IK_MODE };

    int mode() const;

    bool isAttitudeMode() const { return attitudeToggle->isChecked(); }
    bool isFootSnapMode() const { return footSnapToggle->isChecked(); }
    void getSnapThresholds(double& distance, double& angle) const;
    bool isJointPositionLimitMode() const { return jointPositionLimitToggle->isChecked(); }
    bool isPenetrationBlockMode() const { return penetrationBlockToggle->isChecked(); }
    double penetrationBlockDepth() const;
    bool isCollisionLinkHighlihtMode() const { return collisionLinkHighlightToggle->isChecked(); }
    int collisionDetectionPriority() const { return collisionDetectionPriority_; }

    SignalProxy<void()> sigCollisionVisualizationChanged() {
        return sigCollisionVisualizationChanged_;
    }

protected:
    virtual bool storeState(Archive& archive);
    virtual bool restoreState(const Archive& archive);
            
private:
    ToolButton* autoModeRadio;
    ToolButton* fkModeRadio;
    ToolButton* ikModeRadio;

    ToolButton* attitudeToggle;
    ToolButton* footSnapToggle;
    ToolButton* jointPositionLimitToggle;
    ToolButton* penetrationBlockToggle;
    ToolButton* collisionLinkHighlightToggle;

    int collisionDetectionPriority_;

    Signal<void()> sigCollisionVisualizationChanged_;            

    KinematicsBarSetupDialog* setup;

    KinematicsBar();

    void onCollisionVisualizationChanged();
    void onLazyCollisionDetectionModeToggled();
};

}

#endif
