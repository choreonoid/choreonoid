/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyUtil.h"
#include "../ExecutablePath.h"
#include "../FloatingNumberString.h"
#include "../Deque2D.h"

namespace py = pybind11;
using namespace cnoid;

namespace
{

typedef Deque2D< double, std::allocator<double> > Deque2DDouble;
void Deque2DDouble_Row_setitem(Deque2DDouble::Row& self, const int i, const double x) { self[i] = x; }
double& (Deque2DDouble::Row::*Row_getitem)(int) = &Deque2DDouble::Row::operator[];
const double& (Deque2DDouble::Row::*Row_getitem_const)(int) const = &Deque2DDouble::Row::operator[];

}

namespace cnoid {

void exportPySignalTypes(py::module& m);
void exportPyValueTree(py::module& m);
void exportPyEigenTypes(py::module& m);
void exportPyEigenArchive(py::module& m);
void exportPySeqTypes(py::module& m);
void exportPySceneGraph(py::module& m);
void exportPyGeometryTypes(py::module& m);
void exportPyTaskTypes(py::module& m);

}

PYBIND11_PLUGIN(Util)
{
    py::module m("Util", "Python Utility Module");

    py::class_<Referenced>(m, "Referenced");
    //class_<Referenced, ReferencedPtr, boost::noncopyable>("Referenced", no_init);

    exportPySignalTypes(m);
    exportPyValueTree(m);
    exportPyEigenTypes(m);
    exportPyEigenArchive(m);
    exportPySeqTypes(m);
    exportPySceneGraph(m);
    exportPyGeometryTypes(m);
    exportPyTaskTypes(m);

    m.def("shareDirectory", &cnoid::shareDirectory, py::return_value_policy::reference);
    m.def("executablePath", &cnoid::executablePath, py::return_value_policy::reference);
    m.def("executableBasename", &cnoid::executableBasename, py::return_value_policy::reference);
    m.def("executableTopDirectory", &cnoid::executableTopDirectory, py::return_value_policy::reference);

    py::class_<FloatingNumberString>(m, "FloatingNumberString")
        .def(py::init<const std::string&>())
        .def("set", &FloatingNumberString::set)
        .def("setPositiveValue", &FloatingNumberString::setPositiveValue)
        .def("setNonNegativeValue", &FloatingNumberString::setNonNegativeValue)
        .def("value", &FloatingNumberString::value);

    py::class_<Deque2DDouble::Row>(m, "Row")
        .def(py::init<>())
        .def("size", &Deque2DDouble::Row::size)
        .def("at", &Deque2DDouble::Row::at, py::return_value_policy::reference)
        .def("__getitem__", Row_getitem)
        .def("__getitem__", Row_getitem_const)
        .def("__setitem__", Deque2DDouble_Row_setitem)
        ;

    return m.ptr();
}
