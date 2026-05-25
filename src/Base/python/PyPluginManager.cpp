#include "../PluginManager.h"
#include <cnoid/PyUtil>

using namespace cnoid;
namespace nb = nanobind;

namespace cnoid {

void exportPyPluginManager(nb::module_ m)
{
    nb::class_<PluginManager>(m, "PluginManager")
        .def_prop_ro_static(
            "instance", [](nb::handle){ return PluginManager::instance(); }, nb::rv_policy::reference)
        .def("unloadPlugin", (bool (PluginManager::*)(const std::string&)) &PluginManager::unloadPlugin)
        .def("reloadPlugin", &PluginManager::reloadPlugin)
        ;
}

}
