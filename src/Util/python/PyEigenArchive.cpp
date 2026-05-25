#include "PyUtil.h"
#include "PyEigenTypes.h"
#include "../EigenArchive.h"
#include "../ValueTree.h"

using namespace std;
namespace nb = nanobind;
using namespace cnoid;

namespace {

template <class T> nb::object readValue(MappingPtr mapping, const string& key)
{
    T value;
    if(cnoid::read(mapping, key, value)){
        return nb::cast(value);
    }
    return nb::none();
}

template <class T> ListingPtr writeValue(MappingPtr mapping, const std::string& key, const T& value)
{
    return cnoid::write(mapping, key, value);
}

}

namespace cnoid {

void exportPyEigenArchive(nb::module_& m)
{
    m.def("readVector3", readValue<Vector3>);
    m.def("readVector4", readValue<Vector4>);
    m.def("readMatrix4", readValue<Matrix4>);
    m.def("readAffine3", readValue<Affine3>);
    m.def("writeVector3", [](MappingPtr mapping, const std::string& key, const python::Vector3Arg& value){
        return writeValue<Vector3>(mapping, key, value.value); });
    m.def("writeVector4", [](MappingPtr mapping, const std::string& key, const python::Vector4Arg& value){
        return writeValue<Vector4>(mapping, key, value.value); });
    m.def("writeAffine3", writeValue<Affine3>);
}

}
