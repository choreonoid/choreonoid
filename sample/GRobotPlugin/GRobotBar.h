/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GROBOT_GROBOT_BAR_H
#define CNOID_GROBOT_GROBOT_BAR_H

#include <cnoid/ToolBar>
#include <cnoid/Signal>

namespace cnoid {

class GRobotBar : public ToolBar
{
public:
    static GRobotBar* instance();
        
    SignalProxy<void(bool on)> sigSyncModeToggled() {
        return sigSyncModeToggled_;
    }
    SignalProxy<void(bool on)> sigServoSwitchRequest() {
        return sigServoSwitchRequest_;
    }
    SignalProxy<void()> sigPoseSendRequest() {
        return sigPoseSendRequest_;
    }

    bool isSyncMode() {
        return syncCheck->isChecked();
    }

private:
    GRobotBar();

    ToolButton* syncCheck;
    Signal<void(bool on)> sigSyncModeToggled_;
    Signal<void(bool on)> sigServoSwitchRequest_;
    Signal<void()> sigPoseSendRequest_;

    void onServoButtonToggled(bool on);
};

}

#endif
