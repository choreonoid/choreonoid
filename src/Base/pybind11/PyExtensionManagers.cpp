#include "../ExtensionManager.h"
#include "../Plugin.h"
#include "../ToolBar.h"
#include <cnoid/PyUtil>

using namespace cnoid;
namespace py = pybind11;

namespace cnoid {

void exportPyExtensionManagers(py::module m)
{
    py::class_<ExtensionManager, std::unique_ptr<ExtensionManager, py::nodelete>>(m, "ExtensionManager")
        .def("addToolBar", &ExtensionManager::addToolBar)
        .def("mountToolBar", &ExtensionManager::mountToolBar)
        ;

    py::class_<Plugin, std::unique_ptr<Plugin, py::nodelete>, ExtensionManager>(m, "Plugin")
        ;
}

}
