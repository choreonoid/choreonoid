/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyUtil.h"
#include "../EigenArchive.h"
#include "../ValueTree.h"
#include <iostream>

using namespace std;
namespace py = pybind11;
using namespace cnoid;

namespace {

py::object readVector3(MappingPtr mapping, const std::string& key){
    Vector3 v;
    if(cnoid::read(*mapping, key, v)){
        return py::object(py::cast(v));
    }
    return py::object();
}

py::object readVector4(MappingPtr mapping, const std::string& key){
    Vector4 v;
    if(cnoid::read(*mapping, key, v)){
        return py::cast(v);
    }
    return py::object();
}

py::object readMatrix4(MappingPtr mapping, const std::string& key){
    Matrix4 T;
    if(cnoid::read(*mapping, key, T)){
        return py::cast(T);
    }
    return py::object();
}


py::object readAffine3(MappingPtr mapping, const std::string& key){
    Affine3 T;
    try {
        if(cnoid::read(*mapping, key, T.matrix())){
            return py::cast(T);
        }
    }
    catch(const ValueNode::ScalarTypeMismatchException& ex){
        cout << ex.message() << endl;
    }
    return py::object();
}

ListingPtr writeVector3(MappingPtr mapping, const std::string& key, const Vector3& v){
    return &cnoid::write(*mapping, key, v);
}

ListingPtr writeVector4(MappingPtr mapping, const std::string& key, const Vector4& v){
    return &cnoid::write(*mapping, key, v);
}

ListingPtr writeAffine3(MappingPtr mapping, const std::string& key, const Affine3& T){
    return &cnoid::write(*mapping, key, T.matrix());
}

}

namespace cnoid {

void exportPyEigenArchive(py::module& m)
{
    m.def("readVector3", readVector3);
    m.def("readVector4", readVector4);
    m.def("readMatrix4", readMatrix4);
    m.def("readAffine3", readAffine3);
    m.def("writeVector3", writeVector3);
    m.def("writeVector4", writeVector4);
    m.def("writeAffine3", writeAffine3);
}

}
