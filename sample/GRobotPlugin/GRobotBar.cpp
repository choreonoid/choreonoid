/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "GRobotBar.h"
#include "gettext.h"

using namespace cnoid;

GRobotBar* GRobotBar::instance()
{
    static GRobotBar* instance = new GRobotBar;
    return instance;
}


GRobotBar::GRobotBar() : ToolBar(N_("GRobotBar"))
{
    addImage(":/GRobot/icons/grobo-logo.svg")
        ->setToolTip(_("G-Robot toolbar which provides buttons for handling actual G-Robots"));

    addSpacing();
    addSeparator();

    auto servoButton = addToggleButton(QIcon(":/GRobot/icons/servo-on.svg"));
    servoButton->setToolTip(_("Turn on / off servo gains"));
    servoButton->sigToggled().connect([&](bool on){ onServoButtonToggled(on); });

    auto sendButton = addButton(QIcon(":/GRobot/icons/sendpose.svg"));
    sendButton->setToolTip(_("Send the current pose of virtual robots to actual robots"));
    sendButton->sigClicked().connect([&](){ sigPoseSendRequest_(); });
    
    syncCheck = addToggleButton(QIcon(":/GRobot/icons/syncpose.svg"));
    syncCheck->setToolTip(_("Synchronize the pose of actual robots pose with virtual robots"));
    
    syncCheck->sigToggled().connect([&](bool on){ sigSyncModeToggled_(on); });
}


void GRobotBar::onServoButtonToggled(bool on)
{
    sigServoSwitchRequest_(on);
}
