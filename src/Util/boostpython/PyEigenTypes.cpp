/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyUtil.h"
#include "PySignal.h"
#include "../EigenTypes.h"
#include "../EigenUtil.h"

using namespace cnoid;
namespace py = boost::python;

namespace {

py::object numpy;
py::object numpy_ndarray;
py::object numpy_array;
py::object numpy_ndarray_tolist;

template<typename VectorType>
struct Vector_to_ndarray_converter {
    static PyObject* convert(const VectorType& v){
        py::list elements;
        for(int i=0; i < v.size(); ++i){
            elements.append(v[i]);
        }
        py::object array = numpy_array(elements);
        return py::incref(array.ptr());
    }
};

template<typename VectorType, int dim>
struct ndarray_to_Vector_converter {
    ndarray_to_Vector_converter(){
        py::converter::registry::push_back(
            &ndarray_to_Vector_converter::convertible,
            &ndarray_to_Vector_converter::construct,
            py::type_id<VectorType>());
    }
    static void* convertible(PyObject* pyobj){
        if(PyObject_IsInstance(pyobj, numpy_ndarray.ptr()) == 1){
            return pyobj;
        }
        return 0;
    }
    static void construct(PyObject* pyobj, py::converter::rvalue_from_python_stage1_data* data){
        VectorType* pv = new(reinterpret_cast<py::converter::rvalue_from_python_storage<VectorType>*>
                             (data)->storage.bytes) VectorType();
        for(py::ssize_t i = 0; i < dim; ++i) {
            (*pv)[i] = py::extract<typename VectorType::Scalar>(PySequence_GetItem(pyobj, i));
        }
        data->convertible = pv;
    }
};

template<typename VectorType, int dim>
struct pylist_to_Vector_converter {
    pylist_to_Vector_converter(){
        py::converter::registry::push_back(
            &pylist_to_Vector_converter::convertible,
            &pylist_to_Vector_converter::construct,
            py::type_id<VectorType>());
    }
    static void* convertible(PyObject* pyobj){
        if(PySequence_Check(pyobj)){
            return pyobj;
        }
        return 0;
    }
    static void construct(PyObject* pyobj, py::converter::rvalue_from_python_stage1_data* data){
        VectorType* pv = new(reinterpret_cast<py::converter::rvalue_from_python_storage<VectorType>*>
                             (data)->storage.bytes) VectorType();
        for(py::ssize_t i = 0; i < dim; ++i) {
            (*pv)[i] = py::extract<typename VectorType::Scalar>(PySequence_GetItem(pyobj, i));
        }
        data->convertible = pv;
    }
};

template<typename MatrixType>
struct Matrix_to_ndarray_converter {
    static PyObject* convert(const MatrixType& R){
        py::list elements;
        for(int i=0; i < R.rows(); ++i){
            py::list row;
            for(int j=0; j < R.cols(); ++j){
                row.append(R(i, j));
            }
            elements.append(row);
        }
        py::object array = numpy_array(elements);
        return py::incref(array.ptr());
    }
};

template<typename MatrixType, int rows, int cols>
struct ndarray_to_Matrix_converter {
    ndarray_to_Matrix_converter(){
        py::converter::registry::push_back(
            &ndarray_to_Matrix_converter::convertible,
            &ndarray_to_Matrix_converter::construct,
            py::type_id<MatrixType>());
    }
    static void* convertible(PyObject* pyobj){
        if(PyObject_IsInstance(pyobj, numpy_ndarray.ptr()) == 1){
            return pyobj;
        }
        return 0;
    }
    static void construct(PyObject* pyobj, py::converter::rvalue_from_python_stage1_data* data){
        MatrixType* M =
            new(reinterpret_cast<py::converter::rvalue_from_python_storage<MatrixType>*>(data)->storage.bytes)
            MatrixType();
        py::object array((py::handle<>(py::borrowed(pyobj))));
        py::list elements = py::extract<py::list>(numpy_ndarray_tolist(array));
        for(py::ssize_t i = 0; i < rows; ++i) {
            py::list row = py::extract<py::list>(elements[i]);
            for(py::ssize_t j = 0; j < rows; ++j) {
                (*M)(i, j) = py::extract<typename MatrixType::Scalar>(row[j]);
            }
        }
        data->convertible = M;
    }
};

template<typename MatrixType, int rows, int cols>
struct pylist_to_Matrix_converter {
    pylist_to_Matrix_converter(){
        py::converter::registry::push_back(
            &pylist_to_Matrix_converter::convertible,
            &pylist_to_Matrix_converter::construct,
            py::type_id<MatrixType>());
    }
    static void* convertible(PyObject* pyobj){
        if(PySequence_Check(pyobj)){
            return pyobj;
        }
        return 0;
    }
    static void construct(PyObject* pyobj, py::converter::rvalue_from_python_stage1_data* data){
        MatrixType* M =
            new(reinterpret_cast<py::converter::rvalue_from_python_storage<MatrixType>*>(data)->storage.bytes)
            MatrixType();
        for(py::ssize_t i = 0; i < rows; ++i) {
            py::list row = py::extract<py::list>(PySequence_GetItem(pyobj, i));
            for(py::ssize_t j = 0; j < rows; ++j) {
                (*M)(i, j) = py::extract<typename MatrixType::Scalar>(row[j]);
            }
        }
        data->convertible = M;
    }
};

template<typename TransformType>
struct Transform_to_ndarray_converter {
    static PyObject* convert(const TransformType& T){
        py::list elements;
        typename TransformType::ConstLinearPart R = T.linear();
        typename TransformType::ConstTranslationPart p = T.translation();
        py::list bottom;
        for(int i=0; i < 3; ++i){
            py::list row;
            row.append(R(i, 0));
            row.append(R(i, 1));
            row.append(R(i, 2));
            row.append(p[i]);
            elements.append(row);
            bottom.append(static_cast<typename TransformType::Scalar>(0.0));
        }
        bottom.append(static_cast<typename TransformType::Scalar>(1.0));
        elements.append(bottom);
        py::object array = numpy_array(elements);
        return py::incref(array.ptr());
    }
};

template<typename TransformType>
struct ndarray_to_Transform_converter {
    ndarray_to_Transform_converter(){
        py::converter::registry::push_back(
            &ndarray_to_Transform_converter::convertible,
            &ndarray_to_Transform_converter::construct,
            py::type_id<TransformType>());
    }
    static void* convertible(PyObject* pyobj){
        if(PyObject_IsInstance(pyobj, numpy_ndarray.ptr()) == 1){
            return pyobj;
        }
        return 0;
    }
    static void construct(PyObject* pyobj, py::converter::rvalue_from_python_stage1_data* data){
        TransformType* T =
            new(reinterpret_cast<py::converter::rvalue_from_python_storage<TransformType>*>(data)->storage.bytes)
            TransformType();
        py::object array((py::handle<>(py::borrowed(pyobj))));
        py::list elements = py::extract<py::list>(numpy_ndarray_tolist(array));
        for(py::ssize_t i = 0; i < 3; ++i) {
            typename TransformType::LinearPart R = T->linear();
            typename TransformType::TranslationPart p = T->translation();
            py::list row = py::extract<py::list>(elements[i]);
            R(i, 0) = py::extract<typename TransformType::Scalar>(row[0]);
            R(i, 1) = py::extract<typename TransformType::Scalar>(row[1]);
            R(i, 2) = py::extract<typename TransformType::Scalar>(row[2]);
            p[i] = py::extract<typename TransformType::Scalar>(row[3]);
        }
        data->convertible = T;
    }
};

template<typename TransformType>
struct pylist_to_Transform_converter {
    pylist_to_Transform_converter(){
        py::converter::registry::push_back(
            &pylist_to_Transform_converter::convertible,
            &pylist_to_Transform_converter::construct,
            py::type_id<TransformType>());
    }
    static void* convertible(PyObject* pyobj){
        if(PySequence_Check(pyobj)){
            return pyobj;
        }
        return 0;
    }
    static void construct(PyObject* pyobj, py::converter::rvalue_from_python_stage1_data* data){
        TransformType* T =
            new(reinterpret_cast<py::converter::rvalue_from_python_storage<TransformType>*>(data)->storage.bytes)
            TransformType();
        for(py::ssize_t i = 0; i < 3; ++i) {
            typename TransformType::LinearPart R = T->linear();
            typename TransformType::TranslationPart p = T->translation();
            py::list row = py::extract<py::list>(PySequence_GetItem(pyobj, i));
            R(i, 0) = py::extract<typename TransformType::Scalar>(row[0]);
            R(i, 1) = py::extract<typename TransformType::Scalar>(row[1]);
            R(i, 2) = py::extract<typename TransformType::Scalar>(row[2]);
            p[i] = py::extract<typename TransformType::Scalar>(row[3]);
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
    numpy = py::import("numpy");
    numpy_ndarray = numpy.attr("ndarray");
    numpy_array = numpy.attr("array");
    numpy_ndarray_tolist = numpy_ndarray.attr("tolist");
    
    py::to_python_converter<Vector2, Vector_to_ndarray_converter<Vector2> >();
    ndarray_to_Vector_converter<Vector2, 2>();
    pylist_to_Vector_converter<Vector2, 2>();
    
    py::to_python_converter<Vector3, Vector_to_ndarray_converter<Vector3> >();
    ndarray_to_Vector_converter<Vector3, 3>();
    pylist_to_Vector_converter<Vector3, 3>();

    py::to_python_converter<Vector3f, Vector_to_ndarray_converter<Vector3f> >();
    ndarray_to_Vector_converter<Vector3f, 3>();
    pylist_to_Vector_converter<Vector3f, 3>();
    
    py::to_python_converter<Vector4, Vector_to_ndarray_converter<Vector4> >();
    ndarray_to_Vector_converter<Vector4, 4>();
    pylist_to_Vector_converter<Vector4, 4>();
    
    py::to_python_converter<Vector6, Vector_to_ndarray_converter<Vector6> >();
    ndarray_to_Vector_converter<Vector6, 6>();
    pylist_to_Vector_converter<Vector6, 6>();

    py::to_python_converter<Matrix3, Matrix_to_ndarray_converter<Matrix3> >();
    ndarray_to_Matrix_converter<Matrix3, 3, 3>();
    pylist_to_Matrix_converter<Matrix3, 3, 3>();

    py::to_python_converter<Matrix4, Matrix_to_ndarray_converter<Matrix4> >();
    ndarray_to_Matrix_converter<Matrix4, 4, 4>();
    pylist_to_Matrix_converter<Matrix4, 4, 4>();

    py::to_python_converter<Affine3, Transform_to_ndarray_converter<Affine3> >();
    ndarray_to_Transform_converter<Affine3>();
    pylist_to_Transform_converter<Affine3>();

    py::to_python_converter<Position, Transform_to_ndarray_converter<Position> >();
    ndarray_to_Transform_converter<Position>();
    pylist_to_Transform_converter<Position>();

    py::def("rpyFromRot", cnoid::rpyFromRot);
    Matrix3 (*cnoid_rotFromRpy1)(const Vector3& rpy) = cnoid::rotFromRpy;
    py::def("rotFromRpy", cnoid_rotFromRpy1);
    py::def("rotFromRpy44", rotFromRpy44);
    py::def("omegaFromRot", cnoid::omegaFromRot);
    py::def("angleAxis", angleAxis);
    py::def("angleAxis44", angleAxis44);
    py::def("normalized", getNormalized);
    py::def("unitX", getUnitX);
    py::def("getUnitX", getUnitX);
    py::def("unitY", getUnitY);
    py::def("getUnitY", getUnitY);
    py::def("unitZ", getUnitZ);
    py::def("getUnitZ", getUnitZ);

    PySignal<void(const Affine3&)>("Affine3Signal");   
}

}
