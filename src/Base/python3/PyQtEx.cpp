/*!
  @author Shin'ichiro Nakaoka
*/

#include <cnoid/Py3Signal>
#include "../Buttons.h"
#include "../Action.h"
#include "../Timer.h"

namespace py = pybind11;
using namespace cnoid;

namespace cnoid {

void exportPyQtExTypes(py::module m)
{
    py::class_<ToolButton, QToolButton>(m, "ToolButton")
        .def("sigClicked", &ToolButton::sigClicked)
        .def_property_readonly("clicked", &ToolButton::sigClicked)
        .def("sigToggled", &ToolButton::sigToggled)
        .def_property_readonly("toggled", &ToolButton::sigToggled)
        ;

    py::class_<Timer, QTimer>(m, "Timer")
        .def("sigTimeout", &Timer::sigTimeout)
        .def_property_readonly("timeout", &Timer::sigTimeout)
        ;
}

}

