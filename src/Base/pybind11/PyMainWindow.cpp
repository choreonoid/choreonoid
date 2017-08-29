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
        .def_static("instance", &MainWindow::instance, py::return_value_policy::reference)
        .def("setProjectTitle", &MainWindow::setProjectTitle)
        .def("toolBarArea", &MainWindow::toolBarArea)
        .def("viewArea", &MainWindow::viewArea, py::return_value_policy::reference)
        .def("addToolBar", &MainWindow::addToolBar);

    py::class_<ViewArea, QWidget>(m, "ViewArea")
        .def("addView", &ViewArea::addView)
        .def("removeView", &ViewArea::removeView)
        .def("numViews", &ViewArea::numViews);
}

}
