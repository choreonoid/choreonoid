/*!
  @author Shin'ichiro Nakaoka
*/

#include "../MainWindow.h"
#include "../ToolBar.h"
#include "../ToolBarArea.h"
#include "../ViewArea.h"
#include "../View.h"
#include <cnoid/PyUtil>

using namespace boost::python;
using namespace cnoid;

// for MSVC++2015 Update3
CNOID_PYTHON_DEFINE_GET_POINTER(MainWindow)
CNOID_PYTHON_DEFINE_GET_POINTER(ViewArea)

namespace cnoid {

void exportPyMainWindow()
{
    class_<MainWindow, MainWindow*, bases<QMainWindow>, boost::noncopyable>("MainWindow", no_init)
        .def("instance", &MainWindow::instance, return_value_policy<reference_existing_object>()).staticmethod("instance")
        .def("getInstance", &MainWindow::instance, return_value_policy<reference_existing_object>()).staticmethod("getInstance")
        .def("setProjectTitle", &MainWindow::setProjectTitle)
        //.def("toolBarArea", &MainWindow::toolBarArea)
        .def("viewArea", &MainWindow::viewArea, return_value_policy<reference_existing_object>())
        .def("getViewArea", &MainWindow::viewArea, return_value_policy<reference_existing_object>())
        .def("addToolBar", &MainWindow::addToolBar);

/*
    class_<ToolBarArea, ToolBarArea*, bases<QWidget>, boost::noncopyable>("ToolBarArea", no_init)
        .def("addToolBar", &ToolBarArea::addToolBar)
        .def("removeToolBar", &ToolBarArea::removeToolBar);
*/

    class_<ViewArea, ViewArea*, bases<QWidget>, boost::noncopyable>("ViewArea", no_init)
        .def("addView", &ViewArea::addView)
        .def("removeView", &ViewArea::removeView)
        .def("numViews", &ViewArea::numViews)
        .def("getNumViews", &ViewArea::numViews);
}

}
