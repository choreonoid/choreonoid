/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyUtil.h"
#include "../EigenTypes.h"

using namespace boost;
using namespace boost::python;
using namespace cnoid;

namespace {

struct Vector3_to_pylist_converter {
    static PyObject* convert(const Vector3& v){
        python::list retval;
        retval.append(v.x());
        retval.append(v.y());
        retval.append(v.z());
        return python::incref(retval.ptr());
    }
};

struct pylist_to_Vector3_converter {
    pylist_to_Vector3_converter(){
        converter::registry::push_back(
            &pylist_to_Vector3_converter::convertible,
            &pylist_to_Vector3_converter::construct,
            python::type_id<Vector3>());
    }
    static void* convertible(PyObject* pyo){
        if(!PyList_Check(pyo) || PySequence_Size(pyo) != 3){
            return 0;
        }
        return pyo;
    }
    static void construct(PyObject* pyo, python::converter::rvalue_from_python_stage1_data* data){
        Vector3* pv = new(reinterpret_cast<python::converter::rvalue_from_python_storage<Vector3>*>(data)->storage.bytes) Vector3();
        for(python::ssize_t i = 0; i < 3; ++i) {
            (*pv)[i] = python::extract<Vector3::Scalar>(PySequence_GetItem(pyo, i));
        }
        data->convertible = pv;
    }
};

}

namespace cnoid {

void exportEigenTypes()
{
    to_python_converter<Vector3, Vector3_to_pylist_converter>();
    pylist_to_Vector3_converter();
    
    void (SE3::*SE3_set1)(const Vector3& translation, const Quat& rotation) = &SE3::set;
    void (SE3::*SE3_set2)(const Vector3& translation, const Matrix3& R) = &SE3::set;
    void (SE3::*SE3_set3)(const Position& T) = &SE3::set;
    const Vector3& (SE3::*SE3_translation_const)() const = &SE3::translation;
    const Quat& (SE3::*SE3_rotation_const)() const = &SE3::rotation;

    class_<SE3, boost::noncopyable>("SE3", init<>()) 
        .def("set", SE3_set1)
        .def("set", SE3_set2)
        .def("set", SE3_set3)
        .def("translation", SE3_translation_const, return_value_policy<copy_const_reference>())
        .def("rotation", SE3_rotation_const, return_value_policy<copy_const_reference>());   
}

}

