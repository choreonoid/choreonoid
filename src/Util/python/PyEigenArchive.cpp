/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyUtil.h"
#include "../EigenArchive.h"
#include "../ValueTree.h"
#include <iostream>

using namespace std;
namespace python = boost::python;
using namespace cnoid;

namespace {

python::object readVector3(MappingPtr mapping, const std::string& key){
    Vector3 v;
    if(cnoid::read(*mapping, key, v)){
        return python::object(v);
    }
    return python::object();
}

python::object readMatrix4(MappingPtr mapping, const std::string& key){
    Matrix4 T;
    if(cnoid::read(*mapping, key, T)){
        return python::object(T);
    }
    return python::object();
}


python::object readAffine3(MappingPtr mapping, const std::string& key){
    Affine3 T;
    try {
        if(cnoid::read(*mapping, key, T.matrix())){
            return python::object(T);
        }
    }
    catch(const ValueNode::ScalarTypeMismatchException& ex){
        cout << ex.message() << endl;
    }
    return python::object();
}

ListingPtr writeVector3(MappingPtr mapping, const std::string& key, const Vector3& v){
    return &cnoid::write(*mapping, key, v);
}

ListingPtr writeAffine3(MappingPtr mapping, const std::string& key, const Affine3& T){
    return &cnoid::write(*mapping, key, T.matrix());
}

}

namespace cnoid {

void exportPyEigenArchive()
{
    python::def("readVector3", readVector3);
    python::def("readMatrix4", readMatrix4);
    python::def("readAffine3", readAffine3);
    python::def("write", writeVector3);
    python::def("write", writeAffine3);
}

}
