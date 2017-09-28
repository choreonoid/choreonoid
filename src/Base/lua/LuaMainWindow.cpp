/*!
  @author Shin'ichiro Nakaoka
*/

#include "../MainWindow.h"
#include "../ToolBarArea.h"
#include "../ViewArea.h"
#include <cnoid/LuaUtil>

namespace cnoid {

void exportLuaMainWindowTypes(sol::table& module)
{
    module.new_usertype<MainWindow>(
        "MainWindow",
        sol::base_classes, sol::bases<QMainWindow>(),
        "new", sol::no_constructor,
        "instance", &MainWindow::instance,
        "setProjectTitle", &MainWindow::setProjectTitle,
        "toolBarArea", &MainWindow::toolBarArea,
        "viewArea", &MainWindow::viewArea,
        "addToolBar", &MainWindow::addToolBar,
        "removeToolBar", &MainWindow::removeToolBar);

    module.new_usertype<ViewArea>(
        "ViewArea",
        sol::base_classes, sol::bases<QWidget>(),
        "new", sol::no_constructor);
}

}
