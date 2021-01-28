/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyQObjectHolder.h"
#include "PyQString.h"
#include "../Buttons.h"
#include "../Action.h"
#include "../Timer.h"

namespace py = pybind11;
using namespace cnoid;

namespace cnoid {

void exportPyQtExTypes(py::module m)
{
    py::class_<ToolButton, PyQObjectHolder<ToolButton>, QToolButton>(m, "ToolButton")
        .def(py::init<>())
        .def(py::init<QWidget*>())
        .def(py::init<const QString&>())
        .def(py::init<const QString&, QWidget*>())
        .def_property_readonly("sigClicked", &ToolButton::sigClicked)
        ;

    py::class_<Timer, PyQObjectHolder<Timer>, QTimer>(m, "Timer")
        .def(py::init<QObject*>(), py::arg("parent") = (QObject*)(nullptr))
        .def_property_readonly("sigTimeout", &Timer::sigTimeout)
        ;
}

}
