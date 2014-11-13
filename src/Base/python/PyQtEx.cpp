/*!
  @author Shin'ichiro Nakaoka
*/

#include <cnoid/PySignal>
#include "../Button.h"
#include "../Action.h"

using namespace boost;
using namespace boost::python;
using namespace cnoid;

namespace cnoid {

void exportQtExTypes()
{
    class_<ToolButton, ToolButton*, bases<QToolButton>, boost::noncopyable>("ToolButton")
        .def("sigClicked", &ToolButton::sigClicked)
        .def("sigToggled", &ToolButton::sigClicked);
}

}

