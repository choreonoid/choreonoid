#include "PyQString.h"
#include "../MainWindow.h"
#include "../ToolBar.h"
#include "../ToolBarArea.h"
#include "../ViewArea.h"
#include "../View.h"
#include <nanobind/stl/vector.h>
#include <cnoid/PyUtil>

using namespace cnoid;
namespace nb = nanobind;

namespace cnoid {

void exportPyMainWindow(nb::module_ m)
{
    // The singleton MainWindow is owned by the application. Returning it with
    // rv_policy::reference keeps nanobind from taking ownership, which replaces
    // the pybind11 version's releaseFromPythonSideManagement (pyref = -1).
    nb::class_<MainWindow, QMainWindow>(m, "MainWindow")
        .def_prop_ro_static(
            "instance",
            [](nb::handle){ return MainWindow::instance(); }, nb::rv_policy::reference)
        .def("setProjectTitle", &MainWindow::setProjectTitle)
        .def_prop_ro("toolBarArea", &MainWindow::toolBarArea, nb::rv_policy::reference)
        .def_prop_ro("viewArea", &MainWindow::viewArea, nb::rv_policy::reference)
        .def_prop_ro("toolBars", &MainWindow::toolBars, nb::rv_policy::reference)
        .def_prop_ro("visibleToolBars", &MainWindow::visibleToolBars, nb::rv_policy::reference)
        ;

    nb::class_<ViewArea, QWidget>(m, "ViewArea")
        // A View may be created and owned by Python (e.g. a future Python-defined
        // View subclass), in which case the view area takes over its ownership.
        .def("addView",
             [](ViewArea& self, python::OwnershipReleased<View> view){
                 return self.addView(view);
             })
        .def("removeView", &ViewArea::removeView)
        .def_prop_ro("numViews", &ViewArea::numViews)
        ;
}

}
