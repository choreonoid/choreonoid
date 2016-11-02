/*!
  @author Shin'ichiro Nakaoka
*/

#include "../ToolBar.h"
#include "../ToolBarArea.h"
#include <cnoid/LuaUtil>

using namespace cnoid;

namespace cnoid {

void exportLuaToolBarTypes(sol::table& module)
{
    module.new_usertype<ToolBar>(
        "ToolBar",
        sol::base_classes, sol::bases<QWidget, QObject>(),
        "new", [](const char* title) { return new ToolBar(title); },
        "addButton", sol::overload(
            [](ToolBar* self, const char* text) { return self->addButton(text); },
            [](ToolBar* self, const char* text, const char* tooltip) { return self->addButton(text, tooltip); }),
        "addToggleButton", sol::overload(
            [](ToolBar* self, const char* text) { return self->addToggleButton(text); },
            [](ToolBar* self, const char* text, const char* tooltip) { return self->addToggleButton(text, tooltip); }),
        "requestNewRadioGroup", &ToolBar::requestNewRadioGroup,
        "addRadioButton", sol::overload(
            [](ToolBar* self, const char* text) { return self->addRadioButton(text); },
            [](ToolBar* self, const char* text, const char* tooltip) { return self->addRadioButton(text, tooltip); }),
        "addWidget", &ToolBar::addWidget,
        "addSeparator", &ToolBar::addSeparator,
        "addSpacing", &ToolBar::addSpacing,
        "setVisibleByDefault", &ToolBar::setVisibleByDefault,
        "isVisibleByDefault", &ToolBar::isVisibleByDefault,
        "setStretchable", &ToolBar::setStretchable,
        "isStretchable", &ToolBar::isStretchable);

    module.new_usertype<ToolBarArea>(
        "ToolBarArea",
        sol::base_classes, sol::bases<QWidget, QObject>(),
        "new", sol::no_constructor,
        "addToolBar", &ToolBarArea::addToolBar,
        "removeToolBar", &ToolBarArea::removeToolBar);
}

}
