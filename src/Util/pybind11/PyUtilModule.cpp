/*!
  @author Shizuko Hattori
*/

#include "../ExecutablePath.h"
#include "../FloatingNumberString.h"
#include <pybind11/pybind11.h>

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
    exportPyGeometryTypes(m);
    exportPyTaskTypes(m);

    m.attr("shareDirectory") = shareDirectory();
    m.attr("executablePath") = executablePath();
    m.attr("executableBasename") = executableBasename();
    m.attr("executableTopDirectory") = cnoid::executableTopDirectory();

    // deprecated
    m.def("getShareDirectory", &cnoid::shareDirectory);
    m.def("getExecutablePath", &cnoid::executablePath);
    m.def("getExecutableBasename", &cnoid::executableBasename);
    m.def("getExecutableTopDirectory", &cnoid::executableTopDirectory);

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
