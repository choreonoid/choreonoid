/*!
  @author Shin'ichiro Nakaoka
*/

#include <cnoid/PySignal>
#include "../Buttons.h"
#include "../Action.h"
#include "../Timer.h"

using namespace boost;
using namespace boost::python;
using namespace cnoid;

namespace cnoid {

void exportPyQtExTypes()
{
    class_<ToolButton, ToolButton*, bases<QToolButton>, boost::noncopyable>("ToolButton")
        .def("sigClicked", &ToolButton::sigClicked)
        .add_property("clicked", &ToolButton::sigClicked)
        .def("sigToggled", &ToolButton::sigToggled)
        .add_property("toggled", &ToolButton::sigToggled)
        ;

    class_<Timer, Timer*, bases<QTimer>, boost::noncopyable>("Timer")
        .def("sigTimeout", &Timer::sigTimeout)
        .add_property("timeout", &Timer::sigTimeout)
        ;
}

}
