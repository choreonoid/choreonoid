/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyUtil.h"
#include "../ExecutablePath.h"
#include "../FloatingNumberString.h"
#include "../Deque2D.h"

using namespace boost::python;
using namespace cnoid;

namespace
{

typedef Deque2D< double, std::allocator<double> > Deque2DDouble;
void Deque2DDouble_Row_setitem(Deque2DDouble::Row& self, const int i, const double x) { self[i] = x; }
double& (Deque2DDouble::Row::*Row_getitem)(int) = &Deque2DDouble::Row::operator[];
const double& (Deque2DDouble::Row::*Row_getitem_const)(int) const = &Deque2DDouble::Row::operator[];

}

namespace cnoid {

void exportPySignalTypes();
void exportPyValueTree();
void exportPyEigenTypes();
void exportPyEigenArchive();
void exportPySeqTypes();
void exportPySceneGraph();
void exportPyGeometryTypes();
void exportPyTaskTypes();

}

BOOST_PYTHON_MODULE(Util)
{
    class_<Referenced, ReferencedPtr, boost::noncopyable>("Referenced", no_init);

    exportPySignalTypes();
    exportPyValueTree();
    exportPyEigenTypes();
    exportPyEigenArchive();
    exportPySeqTypes();
    exportPySceneGraph();
    exportPyGeometryTypes();
    exportPyTaskTypes();

    def("shareDirectory", &cnoid::shareDirectory, return_value_policy<copy_const_reference>());
    def("getShareDirectory", &cnoid::shareDirectory, return_value_policy<copy_const_reference>());
    def("executablePath", &cnoid::executablePath, return_value_policy<copy_const_reference>());
    def("getExecutablePath", &cnoid::executablePath, return_value_policy<copy_const_reference>());
    def("executableBasename", &cnoid::executableBasename, return_value_policy<copy_const_reference>());
    def("getExecutableBasename", &cnoid::executableBasename, return_value_policy<copy_const_reference>());
    def("executableTopDirectory", &cnoid::executableTopDirectory, return_value_policy<copy_const_reference>());
    def("getExecutableTopDirectory", &cnoid::executableTopDirectory, return_value_policy<copy_const_reference>());

    class_<FloatingNumberString>("FloatingNumberString", init<const std::string&>())
        .def("set", &FloatingNumberString::set)
        .def("setPositiveValue", &FloatingNumberString::setPositiveValue)
        .def("setNonNegativeValue", &FloatingNumberString::setNonNegativeValue)
        .def("value", &FloatingNumberString::value)
        .def("getValue", &FloatingNumberString::value);

    class_<Deque2DDouble::Row>("Row", init<>())
        .def("size", &Deque2DDouble::Row::size)
        .def("getSize", &Deque2DDouble::Row::size)
        .def("at", &Deque2DDouble::Row::at, return_value_policy<copy_non_const_reference>())
        .def("__getitem__", Row_getitem, return_value_policy<return_by_value>())
        .def("__getitem__", Row_getitem_const, return_value_policy<return_by_value>())
        .def("__setitem__", Deque2DDouble_Row_setitem)
        ;
}
