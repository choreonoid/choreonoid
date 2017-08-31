/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyUtil.h"
#include "../EigenArchive.h"
#include "../ValueTree.h"
#include <iostream>

using namespace std;
using namespace cnoid;
namespace py = boost::python;

namespace {

py::object readVector3(MappingPtr mapping, const std::string& key){
    Vector3 v;
    if(cnoid::read(*mapping, key, v)){
        return py::object(v);
    }
    return py::object();
}

py::object readVector4(MappingPtr mapping, const std::string& key){
    Vector4 v;
    if(cnoid::read(*mapping, key, v)){
        return py::object(v);
    }
    return py::object();
}

py::object readMatrix4(MappingPtr mapping, const std::string& key){
    Matrix4 T;
    if(cnoid::read(*mapping, key, T)){
        return py::object(T);
    }
    return py::object();
}


py::object readAffine3(MappingPtr mapping, const std::string& key){
    Affine3 T;
    try {
        if(cnoid::read(*mapping, key, T.matrix())){
            return py::object(T);
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

void exportPyEigenArchive()
{
    py::def("readVector3", readVector3);
    py::def("readVector4", readVector4);
    py::def("readMatrix4", readMatrix4);
    py::def("readAffine3", readAffine3);
    py::def("writeVector3", writeVector3);
    py::def("writeVector4", writeVector4);
    py::def("writeAffine3", writeAffine3);
}

}
