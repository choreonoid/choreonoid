/*!
  @author Shin'ichiro Nakaoka
*/

#include "../OpenHRPInterpreterServiceItem.h"
#include <cnoid/Py3Base>

namespace py = pybind11;
using namespace cnoid;

PYBIND11_PLUGIN(OpenHRP31Plugin)
{
    py::module m("OpenHRP31Plugin", "OpenHRP32Plugin Python Module");

    py::class_< OpenHRPInterpreterServiceItem, OpenHRPInterpreterServiceItemPtr, Item >
        (m, "OpenHRPInterpreterServiceItem")
        .def("setRTCInstanceName", &OpenHRPInterpreterServiceItem::setRTCInstanceName);

    PyItemList<OpenHRPInterpreterServiceItem>(m, "OpenHRPInterpreterServiceItemList");
};
