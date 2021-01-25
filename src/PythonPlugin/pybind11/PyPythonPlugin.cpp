/*!
  @author Shin'ichiro Nakaoka
*/

#include "../PythonPlugin.h"
#include <cnoid/PyBase>

using namespace cnoid;
namespace py = pybind11;

PYBIND11_MODULE(PythonPlugin, m)
{
    m.doc() = "Choreonoid PythonPlugin module";

    py::class_<PythonPlugin, std::unique_ptr<PythonPlugin, py::nodelete>, Plugin>(m, "PythonPlugin")
        .def_property_readonly_static("instance", [](py::object){ return PythonPlugin::instance(); });
};
