#include "../ProjectManager.h"
#include "../Item.h"
#include "PyItemList.h"
#include <nanobind/stl/string.h>
#include <cnoid/PyUtil>

using namespace std;
using namespace cnoid;
namespace nb = nanobind;

namespace cnoid {

void exportPyProjectManager(nb::module_ m)
{
    nb::class_<ProjectManager>(m, "ProjectManager")
        .def_prop_ro_static(
            "instance", [](nb::handle){ return ProjectManager::instance(); }, nb::rv_policy::reference)
        .def("clearProject", &ProjectManager::clearProject)
        .def("loadProject", &ProjectManager::loadProject, nb::arg("filename"), nb::arg("parentItem").none() = nullptr)
        .def("isLoadingProject", &ProjectManager::isLoadingProject)
        .def("saveProject", &ProjectManager::saveProject, nb::arg("filename"), nb::arg("item").none() = nullptr)
        .def("overwriteCurrentProject", &ProjectManager::overwriteCurrentProject)
        .def_prop_ro("currentProjectDirectory", &ProjectManager::currentProjectDirectory)
        .def("setCurrentProjectName", &ProjectManager::setCurrentProjectName)
        ;
}

}
