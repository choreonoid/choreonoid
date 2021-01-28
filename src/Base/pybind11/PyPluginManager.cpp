/*!
  @author Shin'ichiro Nakaoka
*/

#include "../PluginManager.h"
#include <cnoid/PyUtil>

using namespace cnoid;
namespace py = pybind11;

namespace cnoid {

void exportPyPluginManager(py::module m)
{
    py::class_<PluginManager>(m, "PluginManager")
        .def_property_readonly_static(
            "instance", [](py::object){ return PluginManager::instance(); }, py::return_value_policy::reference)
        .def("unloadPlugin", (bool (PluginManager::*)(const std::string&)) &PluginManager::unloadPlugin)
        .def("reloadPlugin", &PluginManager::reloadPlugin)
        ;
}

}
