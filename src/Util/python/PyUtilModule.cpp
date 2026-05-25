#include "PyUtil.h"
#include "../ExecutablePath.h"
#include "../FloatingNumberString.h"
#include "../CloneMap.h"

namespace nb = nanobind;
using namespace cnoid;

namespace cnoid {

void exportPyReferenced(nb::module_& m);
void exportPySignalTypes(nb::module_& m);
void exportPyMessageOut(nb::module_& m);
void exportPyValueTree(nb::module_& m);
void exportPyEigenTypes(nb::module_& m);
void exportPyEigenArchive(nb::module_& m);
void exportPyFilePathVariableProcessor(nb::module_& m);
void exportPySeqTypes(nb::module_& m);
void exportPyGeometryTypes(nb::module_& m);
void exportPySceneGraph(nb::module_& m);
void exportPySceneDrawables(nb::module_& m);
void exportPyMeshUtils(nb::module_& m);
void exportPySceneRenderer(nb::module_& m);
void exportPyTaskTypes(nb::module_& m);

}

NB_MODULE(Util, m)
{
    m.doc() = "Choreonoid Util module";

    exportPyReferenced(m);
    exportPySignalTypes(m);
    exportPyMessageOut(m);
    exportPyValueTree(m);
    exportPyEigenTypes(m);
    exportPyEigenArchive(m);
    exportPyFilePathVariableProcessor(m);
    exportPySeqTypes(m);
    exportPyGeometryTypes(m);
    exportPySceneGraph(m);
    exportPySceneDrawables(m);
    exportPyMeshUtils(m);
    exportPySceneRenderer(m);
    exportPyTaskTypes(m);

    m.attr("shareDirectory") = shareDir();
    m.attr("executableFile") = executableFile();
    m.attr("executableBasename") = executableBasename();
    m.attr("executableTopDir") = cnoid::executableTopDir();

    nb::class_<FloatingNumberString>(m, "FloatingNumberString")
        .def(nb::init<const std::string&>())
        .def("set", &FloatingNumberString::set)
        .def("setPositiveValue", &FloatingNumberString::setPositiveValue)
        .def("setNonNegativeValue", &FloatingNumberString::setNonNegativeValue)
        .def_prop_ro("value", &FloatingNumberString::value)
        ;

    nb::class_<CloneMap> cloneMap(m, "CloneMap");
}
