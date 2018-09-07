/*!
  @author Shin'ichiro Nakaoka
*/

#include "../MainWindow.h"
#include "../ToolBar.h"
#include "../ToolBarArea.h"
#include "../ViewArea.h"
#include "../View.h"
#include "PyQString.h"

using namespace cnoid;
namespace py = pybind11;

namespace cnoid {

void exportPyMainWindow(py::module m)
{
    py::class_<MainWindow, QMainWindow>(m, "MainWindow")
        .def_property_readonly_static(
            "instance", [](py::object){ return MainWindow::instance(); }, py::return_value_policy::reference)
        .def("setProjectTitle", &MainWindow::setProjectTitle)
        .def_property_readonly("toolBarArea", &MainWindow::toolBarArea)
        .def_property_readonly("viewArea", &MainWindow::viewArea, py::return_value_policy::reference)
        .def("addToolBar", &MainWindow::addToolBar)

        // deprecated
        .def_static("getInstance", &MainWindow::instance, py::return_value_policy::reference)
        .def("getToolBarArea", &MainWindow::toolBarArea)
        .def("getViewArea", &MainWindow::viewArea, py::return_value_policy::reference)
        ;

    py::class_<ViewArea, QWidget>(m, "ViewArea")
        .def("addView", &ViewArea::addView)
        .def("removeView", &ViewArea::removeView)
        .def_property_readonly("numViews", &ViewArea::numViews)

        // deprecated
        .def("getNumViews", &ViewArea::numViews)
        ;
}

}
