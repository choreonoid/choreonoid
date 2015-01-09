/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_KINEMATICS_BAR_H
#define CNOID_BODY_PLUGIN_KINEMATICS_BAR_H

#include <cnoid/ToolBar>
#include "exportdecl.h"

namespace cnoid {

class KinematicsBarImpl;

class CNOID_EXPORT KinematicsBar : public ToolBar
{
public:
    static KinematicsBar* instance();
            
    virtual ~KinematicsBar();

    enum Mode { AUTO_MODE, FK_MODE, IK_MODE };
    int mode() const;

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

    KinematicsBarImpl* impl;
};

}

#endif
