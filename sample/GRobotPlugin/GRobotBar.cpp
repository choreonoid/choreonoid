/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "GRobotBar.h"
#include <boost/bind.hpp>
#include "gettext.h"

using namespace boost;
using namespace cnoid;

GRobotBar* GRobotBar::instance()
{
    static GRobotBar* instance = new GRobotBar();
    return instance;
}


GRobotBar::GRobotBar() : ToolBar(N_("GRobotBar"))
{
    addImage(":/GRobot/icons/grobo-logo.png")
        ->setToolTip(_("G-Robot toolbar which provides buttons for handling actual G-Robots"));

    addSpacing(4);
    addSeparator();

    addToggleButton(QIcon(":/GRobot/icons/servo-on.png"), _("Turn on / off servo gains"))
        ->sigToggled().connect(bind(&GRobotBar::onServoButtonToggled, this, _1));

    addButton(QIcon(":/GRobot/icons/sendpose.png"), _("Send the current pose of virtual robots to actual robots"))
        ->sigClicked().connect(bind(ref(sigPoseSendRequest_)));
    
    syncCheck = addToggleButton(QIcon(":/GRobot/icons/syncpose.png"), _("Synchronize the pose of actual robots pose with virtual robots"));
    
    syncCheck->sigToggled().connect(bind(ref(sigSyncModeToggled_),  _1));
}


void GRobotBar::onServoButtonToggled(bool on)
{
    sigServoSwitchRequest_(on);
}
