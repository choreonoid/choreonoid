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
        .def("setIndex", &Device::setIndex)
        .def("setId", &Device::setId)
        .def("setName", &Device::setName)
        .def("setLink", &Device::setLink)
        .def("clone", &Device::clone)
        .def("clearState", &Device::clearState)
        .def("hasStateOnly", &Device::hasStateOnly)
        .def("index", &Device::index)
        .def("id", &Device::id)
        .def("name", &Device::name)
        .def("link", (Link*(Device::*)())&Device::link)
        .def_property("T_local", [](Device& self) ->Position { return self.T_local(); },
                      [](Device& self, const Position& T) { self.T_local() = T.matrix(); })
        ;
}

}
