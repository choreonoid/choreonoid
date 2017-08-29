/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyUtil.h"
#include "PySignal.h"
#include "../EigenTypes.h"
#include "../EigenUtil.h"

namespace python = boost::python;
using namespace cnoid;

namespace {

python::object numpy;
python::object numpy_ndarray;
python::object numpy_array;
python::object numpy_ndarray_tolist;

template<typename VectorType>
struct Vector_to_ndarray_converter {
    static PyObject* convert(const VectorType& v){
        python::list elements;
        for(int i=0; i < v.size(); ++i){
            elements.append(v[i]);
        }
        python::object array = numpy_array(elements);
        return python::incref(array.ptr());
    }
};

template<typename VectorType, int dim>
struct ndarray_to_Vector_converter {
    ndarray_to_Vector_converter(){
        python::converter::registry::push_back(
            &ndarray_to_Vector_converter::convertible,
            &ndarray_to_Vector_converter::construct,
            python::type_id<VectorType>());
    }
    static void* convertible(PyObject* pyobj){
        if(PyObject_IsInstance(pyobj, numpy_ndarray.ptr()) == 1){
            return pyobj;
        }
        return 0;
    }
    static void construct(PyObject* pyobj, python::converter::rvalue_from_python_stage1_data* data){
        VectorType* pv = new(reinterpret_cast<python::converter::rvalue_from_python_storage<VectorType>*>
                             (data)->storage.bytes) VectorType();
        for(python::ssize_t i = 0; i < dim; ++i) {
            (*pv)[i] = python::extract<typename VectorType::Scalar>(PySequence_GetItem(pyobj, i));
        }
        data->convertible = pv;
    }
};

template<typename VectorType, int dim>
struct pylist_to_Vector_converter {
    pylist_to_Vector_converter(){
        python::converter::registry::push_back(
            &pylist_to_Vector_converter::convertible,
            &pylist_to_Vector_converter::construct,
            python::type_id<VectorType>());
    }
    static void* convertible(PyObject* pyobj){
        if(PySequence_Check(pyobj)){
            return pyobj;
        }
        return 0;
    }
    static void construct(PyObject* pyobj, python::converter::rvalue_from_python_stage1_data* data){
        VectorType* pv = new(reinterpret_cast<python::converter::rvalue_from_python_storage<VectorType>*>
                             (data)->storage.bytes) VectorType();
        for(python::ssize_t i = 0; i < dim; ++i) {
            (*pv)[i] = python::extract<typename VectorType::Scalar>(PySequence_GetItem(pyobj, i));
        }
        data->convertible = pv;
    }
};

template<typename MatrixType>
struct Matrix_to_ndarray_converter {
    static PyObject* convert(const MatrixType& R){
        python::list elements;
        for(int i=0; i < R.rows(); ++i){
            python::list row;
            for(int j=0; j < R.cols(); ++j){
                row.append(R(i, j));
            }
            elements.append(row);
        }
        python::object array = numpy_array(elements);
        return python::incref(array.ptr());
    }
};

template<typename MatrixType, int rows, int cols>
struct ndarray_to_Matrix_converter {
    ndarray_to_Matrix_converter(){
        python::converter::registry::push_back(
            &ndarray_to_Matrix_converter::convertible,
            &ndarray_to_Matrix_converter::construct,
            python::type_id<MatrixType>());
    }
    static void* convertible(PyObject* pyobj){
        if(PyObject_IsInstance(pyobj, numpy_ndarray.ptr()) == 1){
            return pyobj;
        }
        return 0;
    }
    static void construct(PyObject* pyobj, python::converter::rvalue_from_python_stage1_data* data){
        MatrixType* M =
            new(reinterpret_cast<python::converter::rvalue_from_python_storage<MatrixType>*>(data)->storage.bytes)
            MatrixType();
        python::object array((boost::python::handle<>(python::borrowed(pyobj))));
        python::list elements = python::extract<python::list>(numpy_ndarray_tolist(array));
        for(python::ssize_t i = 0; i < rows; ++i) {
            python::list row = python::extract<python::list>(elements[i]);
            for(python::ssize_t j = 0; j < rows; ++j) {
                (*M)(i, j) = python::extract<typename MatrixType::Scalar>(row[j]);
            }
        }
        data->convertible = M;
    }
};

template<typename MatrixType, int rows, int cols>
struct pylist_to_Matrix_converter {
    pylist_to_Matrix_converter(){
        python::converter::registry::push_back(
            &pylist_to_Matrix_converter::convertible,
            &pylist_to_Matrix_converter::construct,
            python::type_id<MatrixType>());
    }
    static void* convertible(PyObject* pyobj){
        if(PySequence_Check(pyobj)){
            return pyobj;
        }
        return 0;
    }
    static void construct(PyObject* pyobj, python::converter::rvalue_from_python_stage1_data* data){
        MatrixType* M =
            new(reinterpret_cast<python::converter::rvalue_from_python_storage<MatrixType>*>(data)->storage.bytes)
            MatrixType();
        for(python::ssize_t i = 0; i < rows; ++i) {
            python::list row = python::extract<python::list>(PySequence_GetItem(pyobj, i));
            for(python::ssize_t j = 0; j < rows; ++j) {
                (*M)(i, j) = python::extract<typename MatrixType::Scalar>(row[j]);
            }
        }
        data->convertible = M;
    }
};

template<typename TransformType>
struct Transform_to_ndarray_converter {
    static PyObject* convert(const TransformType& T){
        python::list elements;
        typename TransformType::ConstLinearPart R = T.linear();
        typename TransformType::ConstTranslationPart p = T.translation();
        python::list bottom;
        for(int i=0; i < 3; ++i){
            python::list row;
            row.append(R(i, 0));
            row.append(R(i, 1));
            row.append(R(i, 2));
            row.append(p[i]);
            elements.append(row);
            bottom.append(static_cast<typename TransformType::Scalar>(0.0));
        }
        bottom.append(static_cast<typename TransformType::Scalar>(1.0));
        elements.append(bottom);
        python::object array = numpy_array(elements);
        return python::incref(array.ptr());
    }
};

template<typename TransformType>
struct ndarray_to_Transform_converter {
    ndarray_to_Transform_converter(){
        python::converter::registry::push_back(
            &ndarray_to_Transform_converter::convertible,
            &ndarray_to_Transform_converter::construct,
            python::type_id<TransformType>());
    }
    static void* convertible(PyObject* pyobj){
        if(PyObject_IsInstance(pyobj, numpy_ndarray.ptr()) == 1){
            return pyobj;
        }
        return 0;
    }
    static void construct(PyObject* pyobj, python::converter::rvalue_from_python_stage1_data* data){
        TransformType* T =
            new(reinterpret_cast<python::converter::rvalue_from_python_storage<TransformType>*>(data)->storage.bytes)
            TransformType();
        python::object array((boost::python::handle<>(python::borrowed(pyobj))));
        python::list elements = python::extract<python::list>(numpy_ndarray_tolist(array));
        for(python::ssize_t i = 0; i < 3; ++i) {
            typename TransformType::LinearPart R = T->linear();
            typename TransformType::TranslationPart p = T->translation();
            python::list row = python::extract<python::list>(elements[i]);
            R(i, 0) = python::extract<typename TransformType::Scalar>(row[0]);
            R(i, 1) = python::extract<typename TransformType::Scalar>(row[1]);
            R(i, 2) = python::extract<typename TransformType::Scalar>(row[2]);
            p[i] = python::extract<typename TransformType::Scalar>(row[3]);
        }
        data->convertible = T;
    }
};

template<typename TransformType>
struct pylist_to_Transform_converter {
    pylist_to_Transform_converter(){
        python::converter::registry::push_back(
            &pylist_to_Transform_converter::convertible,
            &pylist_to_Transform_converter::construct,
            python::type_id<TransformType>());
    }
    static void* convertible(PyObject* pyobj){
        if(PySequence_Check(pyobj)){
            return pyobj;
        }
        return 0;
    }
    static void construct(PyObject* pyobj, python::converter::rvalue_from_python_stage1_data* data){
        TransformType* T =
            new(reinterpret_cast<python::converter::rvalue_from_python_storage<TransformType>*>(data)->storage.bytes)
            TransformType();
        for(python::ssize_t i = 0; i < 3; ++i) {
            typename TransformType::LinearPart R = T->linear();
            typename TransformType::TranslationPart p = T->translation();
            python::list row = python::extract<python::list>(PySequence_GetItem(pyobj, i));
            R(i, 0) = python::extract<typename TransformType::Scalar>(row[0]);
            R(i, 1) = python::extract<typename TransformType::Scalar>(row[1]);
            R(i, 2) = python::extract<typename TransformType::Scalar>(row[2]);
            p[i] = python::extract<typename TransformType::Scalar>(row[3]);
        }
        data->convertible = T;
    }
};


Matrix3 angleAxis(double angle, const Vector3& vec){
    return Matrix3(AngleAxis(angle, vec));
}

Affine3 angleAxis44(double angle, const Vector3& vec){
    return Affine3(AngleAxis(angle, vec));
}

Vector3 getNormalized(const Vector3& vec){
    return vec.normalized();
}

Affine3 rotFromRpy44(const Vector3& vec){
    return Affine3(rotFromRpy(vec));
}

Vector3 getUnitX(){
    return Vector3::UnitX();
}

Vector3 getUnitY(){
    return Vector3::UnitY();
}

Vector3 getUnitZ(){
    return Vector3::UnitZ();
}

}

namespace cnoid {

void exportPyEigenTypes()
{
    numpy = python::import("numpy");
    numpy_ndarray = numpy.attr("ndarray");
    numpy_array = numpy.attr("array");
    numpy_ndarray_tolist = numpy_ndarray.attr("tolist");
    
    python::to_python_converter<Vector2, Vector_to_ndarray_converter<Vector2> >();
    ndarray_to_Vector_converter<Vector2, 2>();
    pylist_to_Vector_converter<Vector2, 2>();
    
    python::to_python_converter<Vector3, Vector_to_ndarray_converter<Vector3> >();
    ndarray_to_Vector_converter<Vector3, 3>();
    pylist_to_Vector_converter<Vector3, 3>();

    python::to_python_converter<Vector3f, Vector_to_ndarray_converter<Vector3f> >();
    ndarray_to_Vector_converter<Vector3f, 3>();
    pylist_to_Vector_converter<Vector3f, 3>();
    
    python::to_python_converter<Vector4, Vector_to_ndarray_converter<Vector4> >();
    ndarray_to_Vector_converter<Vector4, 4>();
    pylist_to_Vector_converter<Vector4, 4>();
    
    python::to_python_converter<Vector6, Vector_to_ndarray_converter<Vector6> >();
    ndarray_to_Vector_converter<Vector6, 6>();
    pylist_to_Vector_converter<Vector6, 6>();

    python::to_python_converter<Matrix3, Matrix_to_ndarray_converter<Matrix3> >();
    ndarray_to_Matrix_converter<Matrix3, 3, 3>();
    pylist_to_Matrix_converter<Matrix3, 3, 3>();

    python::to_python_converter<Matrix4, Matrix_to_ndarray_converter<Matrix4> >();
    ndarray_to_Matrix_converter<Matrix4, 4, 4>();
    pylist_to_Matrix_converter<Matrix4, 4, 4>();

    python::to_python_converter<Affine3, Transform_to_ndarray_converter<Affine3> >();
    ndarray_to_Transform_converter<Affine3>();
    pylist_to_Transform_converter<Affine3>();

    python::to_python_converter<Position, Transform_to_ndarray_converter<Position> >();
    ndarray_to_Transform_converter<Position>();
    pylist_to_Transform_converter<Position>();

    python::def("rpyFromRot", cnoid::rpyFromRot);
    Matrix3 (*cnoid_rotFromRpy1)(const Vector3& rpy) = cnoid::rotFromRpy;
    python::def("rotFromRpy", cnoid_rotFromRpy1);
    python::def("rotFromRpy44", rotFromRpy44);
    python::def("omegaFromRot", cnoid::omegaFromRot);
    python::def("angleAxis", angleAxis);
    python::def("angleAxis44", angleAxis44);
    python::def("normalized", getNormalized);
    python::def("unitX", getUnitX);
    python::def("unitY", getUnitY);
    python::def("unitZ", getUnitZ);

    PySignal<void(const Affine3&)>("Affine3Signal");   
}

}
