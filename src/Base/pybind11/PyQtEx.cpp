/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyQString.h"
#include "../Buttons.h"
#include "../Action.h"
#include "../Timer.h"

namespace py = pybind11;
using namespace cnoid;

namespace cnoid {

void exportPyQtExTypes(py::module m)
{
    py::class_<ToolButton, QToolButton>(m, "ToolButton")
        .def(py::init<>())
        .def(py::init<QWidget*>())
        .def(py::init<const QString&>())
        .def(py::init<const QString&, QWidget*>())
        .def_property_readonly("sigClicked", &ToolButton::sigClicked)
        .def_property_readonly("clicked", &ToolButton::sigClicked)
        .def_property_readonly("toggled", &ToolButton::sigToggled)
        ;

    py::class_<Timer, QTimer>(m, "Timer")
        .def(py::init<QObject*>(), py::arg("parent")=(QObject*)(0) )
        .def_property_readonly("timeout", &Timer::sigTimeout)
        ;
}

}
