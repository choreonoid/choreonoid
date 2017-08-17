/*!
  @author Shin'ichiro Nakaoka
*/

#include "../OpenHRPInterpreterServiceItem.h"
#include <cnoid/PyBase>

using namespace cnoid;
namespace py = pybind11;

PYBIND11_PLUGIN(OpenHRP31Plugin)
{
    py::module m("OpenHRP31Plugin", "OpenHRP32Plugin Python Module");

    py::module::import("cnoid.Base");
    
    py::class_<OpenHRPInterpreterServiceItem, OpenHRPInterpreterServiceItemPtr, Item>
        (m, "OpenHRPInterpreterServiceItem")
        .def("setRTCInstanceName", &OpenHRPInterpreterServiceItem::setRTCInstanceName);

    PyItemList<OpenHRPInterpreterServiceItem>(m, "OpenHRPInterpreterServiceItemList");
};
