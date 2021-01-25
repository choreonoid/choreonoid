/*!
  @author Shin'ichiro Nakaoka
 */

#include "PyDeviceList.h"
#include "../Device.h"
#include "../Link.h"
#include "../ForceSensor.h"
#include <cnoid/PyUtil>

using namespace std;
using namespace cnoid;
namespace py = pybind11;

namespace {

using Matrix4RM = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;

}

namespace cnoid {

void exportPyDeviceTypes(py::module& m)
{
    py::class_<Device, DevicePtr, Referenced>(m, "Device")
        .def("__repr__",
             [](const Device &self) {
                 return string("<cnoid.Body.") + self.typeName() + " named '" + self.name() + "'>"; })
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
        .def_property(
            "T_local",
            [](Device& self) -> Isometry3::MatrixType& { return self.T_local().matrix(); },
            [](Device& self, Eigen::Ref<const Matrix4RM> T) { self.T_local() = T; })

        // deprecated
        .def("getIndex", &Device::index)
        .def("getId", &Device::id)
        .def("getName", &Device::name)
        .def("getLink", (Link*(Device::*)())&Device::link)
        ;

    PyDeviceList<Device>(m, "DeviceList");
    PyDeviceList<ForceSensor>(m, "ForceSensorList");
}

}
