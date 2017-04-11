/*!
  @author Shin'ichiro Nakaoka
*/

#include "../MainWindow.h"
#include "../ToolBar.h"
#include "../ToolBarArea.h"
#include "../ViewArea.h"
#include <cnoid/Py3Util>

namespace py = pybind11;
using namespace cnoid;

// for MSVC++2015 Update3
CNOID_PYTHON_DEFINE_GET_POINTER(MainWindow)

namespace cnoid {

void exportPyMainWindow(py::module m)
{
    py::class_<MainWindow, std::unique_ptr<MainWindow, py::nodelete>, QMainWindow>(m, "MainWindow")
        .def_static("instance", &MainWindow::instance, py::return_value_policy::reference)
        .def("setProjectTitle", &MainWindow::setProjectTitle)
        //.def("toolBarArea", &MainWindow::toolBarArea)
        //.def("viewArea", &MainWindow::viewArea)
        .def("addToolBar", &MainWindow::addToolBar);

/*
    class_<ToolBarArea, ToolBarArea*, bases<QWidget>, boost::noncopyable>("ToolBarArea", no_init)
        .def("addToolBar", &ToolBarArea::addToolBar)
        .def("removeToolBar", &ToolBarArea::removeToolBar);
*/

/*
    class_<ViewArea, ViewArea*, bases<QWidget>, boost::noncopyable>("ViewArea", no_init)
        .def("addView", &ViewArea::addView)
        .def("removeView", &ViewArea::removeView)
        .def("numViews", &ViewArea::numViews);
*/
}

}
