/*!
  @author Shin'ichiro Nakaoka
*/

#include "../PluginManager.h"
#include <pybind11/pybind11.h>

namespace py = pybind11;
using namespace cnoid;

namespace {

bool PluginManager_unloadPlugin2(PluginManager& self, const std::string& name) { return self.unloadPlugin(name); }

}

namespace cnoid {

void exportPyPluginManager(py::module m)
{
    py::class_<PluginManager>(m, "PluginManager")
        .def_static("instance", &PluginManager::instance, py::return_value_policy::reference)
        .def("unloadPlugin", (bool (PluginManager::*)(const std::string&)) &PluginManager::unloadPlugin)
        .def("reloadPlugin", &PluginManager::reloadPlugin)
        ;
}

}
