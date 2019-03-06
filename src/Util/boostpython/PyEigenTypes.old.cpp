/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyUtil.h"
#include "../EigenTypes.h"
#include "../EigenUtil.h"

namespace python = boost::python;
using namespace boost::python;
using namespace cnoid;

namespace {

template<typename VectorType>
struct Vector_to_pylist_converter {
    static PyObject* convert(const VectorType& v){
        python::list retval;
        for(int i=0; i < v.size(); ++i){
            retval.append(v[i]);
        }
        return python::incref(retval.ptr());
    }
};

template<typename VectorType, int dim>
struct pylist_to_Vector_converter {
    pylist_to_Vector_converter(){
        converter::registry::push_back(
            &pylist_to_Vector_converter::convertible,
            &pylist_to_Vector_converter::construct,
            python::type_id<VectorType>());
    }
    static void* convertible(PyObject* pyo){
        if(PySequence_Check(pyo) && PySequence_Size(pyo) == dim){
            return pyo;
        }
        return 0;
    }
    static void construct(PyObject* pyo, python::converter::rvalue_from_python_stage1_data* data){
        VectorType* pv = new(reinterpret_cast<python::converter::rvalue_from_python_storage<VectorType>*>(data)->storage.bytes) VectorType();
        for(python::ssize_t i = 0; i < dim; ++i) {
            (*pv)[i] = python::extract<typename VectorType::Scalar>(PySequence_GetItem(pyo, i));
        }
        data->convertible = pv;
    }
};

template<typename MatrixType>
struct Matrix_to_pylist_converter {
    static PyObject* convert(const MatrixType& R){
        python::list Rout;
        for(int i=0; i < R.rows(); ++i){
            python::list row;
            for(int j=0; j < R.cols(); ++j){
                row.append(R(i, j));
            }
            Rout.append(row);
        }
        return python::incref(Rout.ptr());
    }
};

template<typename MatrixType, int rows, int cols>
struct pylist_to_Matrix_converter {
    pylist_to_Matrix_converter(){
        converter::registry::push_back(
            &pylist_to_Matrix_converter::convertible,
            &pylist_to_Matrix_converter::construct,
            python::type_id<MatrixType>());
    }
    static void* convertible(PyObject* pyo){
        if(PySequence_Check(pyo)){
            return pyo;
        }
        return 0;
    }
    static void construct(PyObject* pyo, python::converter::rvalue_from_python_stage1_data* data){
        MatrixType* M =
            new(reinterpret_cast<python::converter::rvalue_from_python_storage<MatrixType>*>(data)->storage.bytes)
            MatrixType();
        for(python::ssize_t i = 0; i < rows; ++i) {
            python::list row = python::extract<python::list>(PySequence_GetItem(pyo, i));
            for(python::ssize_t j = 0; j < rows; ++j) {
                (*M)(i, j) = python::extract<typename MatrixType::Scalar>(row[j]);
            }
        }
        data->convertible = M;
    }
};

template<typename TransformType>
struct Transform_to_pylist_converter {
    static PyObject* convert(const TransformType& T){
        python::list Tout;
        typename TransformType::ConstLinearPart R = T.linear();
        typename TransformType::ConstTranslationPart p = T.translation();
        python::list bottom;
        for(int i=0; i < 3; ++i){
            python::list row;
            row.append(R(i, 0));
            row.append(R(i, 1));
            row.append(R(i, 2));
            row.append(p[i]);
            Tout.append(row);
            bottom.append(static_cast<typename TransformType::Scalar>(0.0));
        }
        bottom.append(static_cast<typename TransformType::Scalar>(1.0));
        Tout.append(bottom);
        return python::incref(Tout.ptr());
    }
};

template<typename TransformType>
struct pylist_to_Transform_converter {
    pylist_to_Transform_converter(){
        converter::registry::push_back(
            &pylist_to_Transform_converter::convertible,
            &pylist_to_Transform_converter::construct,
            python::type_id<TransformType>());
    }
    static void* convertible(PyObject* pyo){
        if(PySequence_Check(pyo)){
            int numRows = PySequence_Size(pyo);
            if(numRows == 3 || numRows == 4){
                return pyo;
            }
        }
        return 0;
    }
    static void construct(PyObject* pyo, python::converter::rvalue_from_python_stage1_data* data){
        TransformType* T =
            new(reinterpret_cast<python::converter::rvalue_from_python_storage<TransformType>*>(data)->storage.bytes)
            TransformType();
        for(python::ssize_t i = 0; i < 3; ++i) {
            typename TransformType::LinearPart R = T->linear();
            typename TransformType::TranslationPart p = T->translation();
            python::list row = python::extract<python::list>(PySequence_GetItem(pyo, i));
            R(i, 0) = python::extract<typename TransformType::Scalar>(row[0]);
            R(i, 1) = python::extract<typename TransformType::Scalar>(row[1]);
            R(i, 2) = python::extract<typename TransformType::Scalar>(row[2]);
            p[i] = python::extract<typename TransformType::Scalar>(row[3]);
        }
        data->convertible = T;
    }
};


//! \todo replace this function with another python functions
Affine3 getAffine3FromAngleAxis(double angle, const Vector3& vec){
    /*
    Affine3 m;
    m = AngleAxis(angle, vec);
    return m;
    */
    return Affine3(AngleAxis(angle, vec));
}


//! \todo replace this function with another python functions
Vector3 getNormalized(const Vector3& vec){
    return vec.normalized();
}

}

namespace cnoid {

void exportPyEigenTypes()
{
    to_python_converter<Vector2, Vector_to_pylist_converter<Vector2> >();
    pylist_to_Vector_converter<Vector2, 2>();
    
    to_python_converter<Vector3, Vector_to_pylist_converter<Vector3> >();
    pylist_to_Vector_converter<Vector3, 3>();

    to_python_converter<Vector4, Vector_to_pylist_converter<Vector4> >();
    pylist_to_Vector_converter<Vector4, 4>();
    
    to_python_converter<Vector6, Vector_to_pylist_converter<Vector6> >();
    pylist_to_Vector_converter<Vector6, 6>();

    to_python_converter<Matrix3, Matrix_to_pylist_converter<Matrix3> >();
    pylist_to_Matrix_converter<Matrix3, 3, 3>();

    to_python_converter<Matrix4, Matrix_to_pylist_converter<Matrix4> >();
    pylist_to_Matrix_converter<Matrix4, 4, 4>();

    to_python_converter<Affine3, Transform_to_pylist_converter<Affine3> >();
    pylist_to_Transform_converter<Affine3>();

    to_python_converter<Position, Transform_to_pylist_converter<Position> >();
    pylist_to_Transform_converter<Position>();

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

    python::def("rpyFromRot", cnoid::rpyFromRot);
    Matrix3 (*cnoid_rotFromRpy1)(const Vector3& rpy) = cnoid::rotFromRpy;
    python::def("rotFromRpy", cnoid_rotFromRpy1);
    python::def("omegaFromRot", cnoid::omegaFromRot);

    //! \todo replace the following functions with another python functions
    python::def("angleAxis", getAffine3FromAngleAxis);
    python::def("normalized", getNormalized);
}

}

