#include "PyQString.h"
#include "../Buttons.h"
#include "../Action.h"
#include "../Timer.h"
#include <nanobind/nanobind.h>

namespace nb = nanobind;
using namespace cnoid;

namespace cnoid {

void exportPyQtExTypes(nb::module_ m)
{
    // Created on the heap (nb::new_) like the other widgets so that they can be
    // reparented to a Qt parent (e.g. a tool bar) and deleted by Qt.
    nb::class_<ToolButton, QToolButton>(m, "ToolButton")
        .def(nb::new_([]{ return new ToolButton(); }))
        .def(nb::new_([](QWidget* parent){ return new ToolButton(parent); }))
        .def(nb::new_([](const QString& text){ return new ToolButton(text); }))
        .def(nb::new_([](const QString& text, QWidget* parent){ return new ToolButton(text, parent); }))
        .def_prop_ro("sigClicked", &ToolButton::sigClicked)
        ;

    nb::class_<Timer, QTimer>(m, "Timer")
        .def(nb::new_([](QObject* parent){ return new Timer(parent); }), nb::arg("parent").none() = (QObject*)(nullptr))
        .def_prop_ro("sigTimeout", &Timer::sigTimeout)
        ;
}

}
