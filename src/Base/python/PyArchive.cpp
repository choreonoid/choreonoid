#include "../Archive.h"
#include "../Item.h"
#include <cnoid/PyUtil>

using namespace cnoid;
namespace nb = nanobind;

namespace cnoid {

void exportPyArchive(nb::module_ m)
{
    // Archive derives from Mapping (bound in cnoid.Util), so the inherited
    // read/write methods are used for the bulk of storeState()/restoreState()
    // work. Only the Archive-specific helpers commonly used from those methods
    // are added here. Sub-archives and items are owned by the archive / item
    // tree, so they are returned as non-owning references.
    nb::class_<Archive, Mapping>(m, "Archive")
        .def(nb::init<>())
        .def("findSubArchive", (Archive*(Archive::*)(const std::string&)) &Archive::findSubArchive,
             nb::rv_policy::reference)
        .def("openSubArchive", &Archive::openSubArchive, nb::rv_policy::reference)
        .def("writeItemId", &Archive::writeItemId)
        .def("findItem", [](const Archive& self, const std::string& key){
            return self.findItem<Item>(key); }, nb::rv_policy::reference)
        .def("currentParentItem", &Archive::currentParentItem, nb::rv_policy::reference)
        .def("resolveRelocatablePath", &Archive::resolveRelocatablePath,
             nb::arg("relocatable"), nb::arg("doAbsolutize") = true)
        ;
}

}
