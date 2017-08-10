/*!
  @author Shin'ichiro Nakaoka
*/

#include "../MainWindow.h"
#include "../ToolBar.h"
#include "../ToolBarArea.h"
#include "../ViewArea.h"
#include <cnoid/PyUtil>

namespace py = pybind11;
using namespace cnoid;

// for MSVC++2015 Update3
CNOID_PYTHON_DEFINE_GET_POINTER(MainWindow)

namespace cnoid {

void exportPyMainWindow(py::module m)
{
    py::class_<MainWindow, std::unique_ptr<MainWindow, py::nodelete> >(m, "MainWindow")
        .def_static("instance", &MainWindow::instance, py::return_value_policy::reference)
        .def("setProjectTitle", &MainWindow::setProjectTitle)
        //.def("toolBarArea", &MainWindow::toolBarArea)
        //.def("viewArea", &MainWindow::viewArea)
        .def("addToolBar", &MainWindow::addToolBar);
}

}
