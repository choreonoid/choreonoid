/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyQObjectHolder.h"
#include "PyQString.h"
#include "../MainWindow.h"
#include "../ToolBar.h"
#include "../ToolBarArea.h"
#include "../ViewArea.h"
#include "../View.h"
#include <cnoid/PyUtil>

using namespace cnoid;
namespace py = pybind11;

namespace cnoid {

void exportPyMainWindow(py::module m)
{
    py::class_<MainWindow, PyQObjectHolder<MainWindow>, QMainWindow>(m, "MainWindow")
        .def_property_readonly_static(
            "instance",
            [](py::object){ return releaseFromPythonSideManagement(MainWindow::instance()); }
            )
        .def("setProjectTitle", &MainWindow::setProjectTitle)
        .def_property_readonly("toolBarArea", &MainWindow::toolBarArea)
        .def_property_readonly("viewArea", &MainWindow::viewArea)
        .def_property_readonly("toolBars", &MainWindow::toolBars, py::return_value_policy::reference)
        .def_property_readonly("visibleToolBars", &MainWindow::visibleToolBars, py::return_value_policy::reference)

        // The following functions should not be used in Python scripts.
        // Use PythonPlugin.PythonPlugin.instance to add a tool bar
        //.def("addToolBar", &MainWindow::addToolBar)
        ;

    py::class_<ViewArea, PyQObjectHolder<ViewArea>, QWidget>(m, "ViewArea")
        .def("addView", &ViewArea::addView)
        .def("removeView", &ViewArea::removeView)
        .def_property_readonly("numViews", &ViewArea::numViews)
        ;
}

}
