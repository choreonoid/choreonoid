/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GROBOT_GROBOT_BAR_H_INCLUDED
#define CNOID_GROBOT_GROBOT_BAR_H_INCLUDED

#include <cnoid/ToolBar>
#include <cnoid/SignalProxy>

namespace cnoid {

class GRobotBar : public ToolBar
{
public:
    static GRobotBar* instance();
        
    SignalProxy< boost::signal<void(bool on)> > sigSyncModeToggled() {
        return sigSyncModeToggled_;
    }
    SignalProxy< boost::signal<void(bool on)> > sigServoSwitchRequest() {
        return sigServoSwitchRequest_;
    }
    SignalProxy< boost::signal<void()> > sigPoseSendRequest() {
        return sigPoseSendRequest_;
    }

    bool isSyncMode() {
        return syncCheck->isChecked();
    }

private:
    GRobotBar();

    ToolButton* syncCheck;
    boost::signal<void(bool on)> sigSyncModeToggled_;
    boost::signal<void(bool on)> sigServoSwitchRequest_;
    boost::signal<void()> sigPoseSendRequest_;

    void onServoButtonToggled(bool on);
};
}

#endif
