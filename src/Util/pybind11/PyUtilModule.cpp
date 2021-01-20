/*!
  @author Shizuko Hattori
*/

#include "PyUtil.h"
#include "../ExecutablePath.h"
#include "../FloatingNumberString.h"

using namespace cnoid;
namespace py = pybind11;

namespace cnoid {

void exportPyReferenced(py::module& m);
void exportPySignalTypes(py::module& m);
void exportPyValueTree(py::module& m);
void exportPyEigenTypes(py::module& m);
void exportPyEigenArchive(py::module& m);
void exportPySeqTypes(py::module& m);
void exportPySceneGraph(py::module& m);
void exportPySceneDrawables(py::module& m);
void exportPySceneRenderer(py::module& m);
void exportPyMeshUtils(py::module& m);
void exportPyGeometryTypes(py::module& m);
void exportPyTaskTypes(py::module& m);

}

PYBIND11_MODULE(Util, m)
{
    m.doc() = "Choreonoid Util module";

    exportPyReferenced(m);
    exportPySignalTypes(m);
    exportPyValueTree(m);
    exportPyEigenTypes(m);
    exportPyEigenArchive(m);
    exportPySeqTypes(m);
    exportPySceneGraph(m);
    exportPySceneDrawables(m);
    exportPySceneRenderer(m);
    exportPyMeshUtils(m);
    exportPyGeometryTypes(m);
    exportPyTaskTypes(m);

    m.attr("shareDirectory") = shareDir();
    m.attr("executableFile") = executableFile();
    m.attr("executablePath") = executableFile(); // deprecated
    m.attr("executableBasename") = executableBasename();
    m.attr("executableTopDir") = cnoid::executableTopDir();
    m.attr("executableTopDirectory") = cnoid::executableTopDir(); // deprecated

    // deprecated
    m.def("getShareDirectory", &cnoid::shareDir);
    m.def("getExecutablePath", &cnoid::executableFile);
    m.def("getExecutableBasename", &cnoid::executableBasename);
    m.def("getExecutableTopDirectory", &cnoid::executableTopDir);

    py::class_<FloatingNumberString>(m, "FloatingNumberString")
        .def(py::init<const std::string&>())
        .def("set", &FloatingNumberString::set)
        .def("setPositiveValue", &FloatingNumberString::setPositiveValue)
        .def("setNonNegativeValue", &FloatingNumberString::setNonNegativeValue)
        .def_property_readonly("value", &FloatingNumberString::value)

        // deprecated
        .def("getValue", &FloatingNumberString::value)
        ;
}
