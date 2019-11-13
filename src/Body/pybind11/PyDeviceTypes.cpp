/*!
  @author Shin'ichiro Nakaoka
 */

#include "../Device.h"
#include "../Link.h"
#include <cnoid/PyReferenced>
#include <cnoid/PyEigenTypes>

using namespace std;
using namespace cnoid;
namespace py = pybind11;

namespace cnoid {

void exportPyDeviceTypes(py::module& m)
{
    py::class_<Device, DevicePtr, Referenced>(m, "Device")
        .def_property("index", &Device::index, &Device::setIndex)
        .def("setIndex", &Device::setIndex)
        .def_property("id", &Device::id, &Device::setId)
        .def("setId", &Device::setId)
        .def_property("name", &Device::name, &Device::setName)
        .def("setName", &Device::setName)
        .def("link", (Link*(Device::*)())&Device::link)
        .def("setLink", &Device::setLink)
        .def("clone", (Device*(Device::*)()const) &Device::clone)
        .def("clearState", &Device::clearState)
        .def("hasStateOnly", &Device::hasStateOnly)
        .def_property("T_local", [](Device& self) ->Position { return self.T_local(); },
                      [](Device& self, const Position& T) { self.T_local() = T.matrix(); })

        // deprecated
        .def("getIndex", &Device::index)
        .def("getId", &Device::id)
        .def("getName", &Device::name)
        .def("getLink", (Link*(Device::*)())&Device::link)
        ;
}

}
