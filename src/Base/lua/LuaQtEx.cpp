/*!
  @author Shin'ichiro Nakaoka
*/

#include "../Buttons.h"
#include "../Action.h"
#include "../Timer.h"
#include <cnoid/LuaUtil>
#include <cnoid/LuaSignal>

namespace cnoid {

void exportLuaQtExTypes(sol::table& module)
{
    module.new_usertype<ToolButton>(
        "ToolButton",
        sol::base_classes, sol::bases<QToolButton, QAbstractButton, QWidget, QObject>(),
        "new", sol::factories(
            []() { return new ToolButton(); },
            [](const char* label) { return new ToolButton(label); }),
        "sigClicked", &ToolButton::sigClicked,
        //.add_property("clicked", &ToolButton::sigClicked)
        "sigToggled", &ToolButton::sigToggled
        //.add_property("toggled", &ToolButton::sigToggled)
        );

    module.new_usertype<Timer>(
        "Timer",
        sol::base_classes, sol::bases<QTimer, QObject>(),
        "new", sol::factories([]() { return new Timer; }),
        "sigTimeout", &Timer::sigTimeout
        //.add_property("timeout", &Timer::sigTimeout)
        );
}

}
