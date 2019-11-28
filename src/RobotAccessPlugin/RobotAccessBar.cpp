/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "RobotAccessBar.h"
#include "RobotAccessItem.h"
#include <cnoid/RootItem>
#include <cnoid/RootItem>
#include <cnoid/ItemList>
#include <cnoid/MessageView>
#include "gettext.h"

using namespace std::placeholders;
using namespace cnoid;

namespace cnoid {

class RobotAccessBarImpl
{
public:
    ToolButton* syncToggle;

    RobotAccessBarImpl(RobotAccessBar* self);
    void connectToRobots();
    void disconnectFromRobots();
    void turnOnServos();
    void turnOffServos();
    void onSendPoseButtonClicked();
    void onServoButtonToggled(bool on);
};

}


namespace {

void forEachCheckedRobotAccessItems
(std::function<void(RobotAccessItem* item)> func, const char* noTargetMessage)
{
    auto items = RootItem::instance()->checkedItems<RobotAccessItem>();
    if(items.empty()){
        MessageView::instance()->putln(noTargetMessage);
    } else {
        for(size_t i=0; i < items.size(); ++i){
            func(items[i].get());
        }
    }
}

}
        

RobotAccessBar::RobotAccessBar()
    : ToolBar(N_("RobotAccessBar"))
{
    impl = new RobotAccessBarImpl(this);
}


RobotAccessBar::~RobotAccessBar()
{

}

    
RobotAccessBarImpl::RobotAccessBarImpl(RobotAccessBar* self)
{
    self->addButton(_("C"), _("Connect to robots"))
        ->sigClicked().connect(std::bind(&RobotAccessBarImpl::connectToRobots, this));

    self->addButton(_("D"), _("Disconnect from robots"))
        ->sigClicked().connect(std::bind(&RobotAccessBarImpl::disconnectFromRobots, this));
    
    self->addButton(QIcon(":/RobotAccess/icons/servo-on.png"), _("Turn on servos"))
        ->sigClicked().connect(std::bind(&RobotAccessBarImpl::turnOnServos, this));

    self->addButton(_("OFF"), _("Turn off servos"))
        ->sigClicked().connect(std::bind(&RobotAccessBarImpl::turnOffServos, this));
    
    self->addButton(QIcon(":/RobotAccess/icons/sendpose.png"),
                    _("Send the current pose of virtual robots to actual robots"))
        ->sigClicked().connect(std::bind(&RobotAccessBarImpl::onSendPoseButtonClicked, this));
    
    syncToggle = self->addToggleButton(QIcon(":/RobotAccess/icons/syncpose.png"),
                                       _("Synchronize the pose of actual robots pose with virtual robots"));
}


void RobotAccessBarImpl::connectToRobots()
{
    forEachCheckedRobotAccessItems(
        std::bind(&RobotAccessItem::connectToRobot, _1),
        _("There are no checked items to connect to robots"));
}


void RobotAccessBarImpl::disconnectFromRobots()
{
    forEachCheckedRobotAccessItems(
        std::bind(&RobotAccessItem::disconnectFromRobot, _1),
        _("There are no checked items to disconnect from robots"));
}


void RobotAccessBarImpl::turnOnServos()
{
    forEachCheckedRobotAccessItems(
        std::bind(&RobotAccessItem::activateServos, _1, true),
        _("There are no checked items to activate servos"));
}


void RobotAccessBarImpl::turnOffServos()
{
    forEachCheckedRobotAccessItems(
        std::bind(&RobotAccessItem::activateServos, _1, false),
        _("There are no checked items to deactivate servos"));
}


void RobotAccessBarImpl::onSendPoseButtonClicked()
{

}


void RobotAccessBarImpl::onServoButtonToggled(bool on)
{

}
