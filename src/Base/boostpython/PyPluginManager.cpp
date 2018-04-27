/*!
  @author Shin'ichiro Nakaoka
*/

#include "../PluginManager.h"
#include <boost/python.hpp>

using namespace boost::python;
using namespace cnoid;

namespace {

bool PluginManager_unloadPlugin2(PluginManager& self, const std::string& name) { return self.unloadPlugin(name); }

}

namespace cnoid {

void exportPyPluginManager()
{
    class_<PluginManager, PluginManager*, boost::noncopyable>("PluginManager", no_init)
        .def("instance", &PluginManager::instance, return_value_policy<reference_existing_object>()).staticmethod("instance")
        .def("getInstance", &PluginManager::instance, return_value_policy<reference_existing_object>()).staticmethod("getInstance")
        .def("unloadPlugin", PluginManager_unloadPlugin2)
        .def("reloadPlugin", &PluginManager::reloadPlugin)
        ;
}

}
