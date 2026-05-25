#include "../ExtensionManager.h"
#include "../Plugin.h"
#include "../ToolBar.h"
#include <cnoid/PyUtil>

using namespace cnoid;
namespace nb = nanobind;

namespace cnoid {

void exportPyExtensionManagers(nb::module_ m)
{
    // ExtensionManager / Plugin are not owned by Python (they are managed by the
    // application), so they are registered without an init and not deleted by
    // nanobind (the pybind11 version used a py::nodelete holder for this).
    nb::class_<ExtensionManager>(m, "ExtensionManager")
        .def("addToolBar",
             [](ExtensionManager& self, python::OwnershipReleased<ToolBar> toolBar){
                 self.addToolBar(toolBar);
             })
        .def("mountToolBar",
             [](ExtensionManager& self, python::OwnershipReleased<ToolBar> toolBar){
                 self.mountToolBar(toolBar);
             })
        ;

    nb::class_<Plugin, ExtensionManager>(m, "Plugin")
        ;
}

}
