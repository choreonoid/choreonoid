/*!
  @author Shin'ichiro Nakaoka
*/

#include <cnoid/PySignal>
#include "PyQtCore.h"
#include "../Buttons.h"
#include "../Action.h"
#include "../Timer.h"

namespace py = pybind11;
using namespace cnoid;

namespace cnoid {

void exportPyQtExTypes(py::module m)
{
    py::class_<ToolButton, QToolButton>(m, "ToolButton")
        .def(py::init<QWidget*>(), py::arg("parent")=(QWidget*)(0) )
        .def(py::init<const QString&, QWidget*>(), py::arg("text"), py::arg("parent")=(QWidget*)(0) )
        .def("sigClicked", &ToolButton::sigClicked)
        .def_property_readonly("clicked", &ToolButton::sigClicked)
        .def("sigToggled", &ToolButton::sigToggled)
        .def_property_readonly("toggled", &ToolButton::sigToggled)
        ;

    py::class_<Timer, QTimer>(m, "Timer")
        .def(py::init<QObject*>(), py::arg("parent")=(QObject*)(0) )
        .def("sigTimeout", &Timer::sigTimeout)
        .def_property_readonly("timeout", &Timer::sigTimeout)
        ;
}

}
