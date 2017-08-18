/*!
  @author Shin'ichiro Nakaoka
*/

#include "../OpenHRPInterpreterServiceItem.h"
#include <cnoid/PyBase>

using namespace cnoid;
namespace py = pybind11;

PYBIND11_MODULE(OpenHRP31Plugin, m)
{
    m.doc() = "Choreonoid OpenHRP3Plugin module";

    py::module::import("cnoid.Base");
    
    py::class_<OpenHRPInterpreterServiceItem, OpenHRPInterpreterServiceItemPtr, Item>
        (m, "OpenHRPInterpreterServiceItem")
        .def("setRTCInstanceName", &OpenHRPInterpreterServiceItem::setRTCInstanceName);

    PyItemList<OpenHRPInterpreterServiceItem>(m, "OpenHRPInterpreterServiceItemList");
};
