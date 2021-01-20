/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyUtil.h"
#include "../EigenArchive.h"
#include "../ValueTree.h"

using namespace std;
namespace py = pybind11;
using namespace cnoid;

namespace {

template <class T> py::object readValue(MappingPtr mapping, const string& key)
{
    T value;
    if(cnoid::read(mapping, key, value)){
        return py::cast(value);
    }
    return py::none();
}

template <class T> ListingPtr writeValue(MappingPtr mapping, const std::string& key, const T& value)
{
    return cnoid::write(mapping, key, value);
}

}

namespace cnoid {

void exportPyEigenArchive(py::module& m)
{
    m.def("readVector3", readValue<Vector3>);
    m.def("readVector4", readValue<Vector4>);
    m.def("readMatrix4", readValue<Matrix4>);
    m.def("readAffine3", readValue<Affine3>);
    m.def("writeVector3", writeValue<Vector3>);
    m.def("writeVector4", writeValue<Vector4>);
    m.def("writeAffine3", writeValue<Matrix4>);
}

}
