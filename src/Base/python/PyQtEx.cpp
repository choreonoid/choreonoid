/*!
  @author Shin'ichiro Nakaoka
*/

#include <cnoid/PySignal>
#include "../Button.h"
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
        .def("sigToggled", &ToolButton::sigClicked);

    class_<Timer, Timer*, bases<QTimer>, boost::noncopyable>("Timer")
        .def("sigTimeout", &Timer::sigTimeout);
}

}

