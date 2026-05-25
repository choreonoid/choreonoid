#ifndef CNOID_UTIL_PYTHON_PYEIGENTYPES_H
#define CNOID_UTIL_PYTHON_PYEIGENTYPES_H

#include "../EigenTypes.h"
#include <nanobind/eigen/dense.h>

/*
   nanobind's Eigen support (nanobind/eigen/dense.h) provides type casters for
   plain matrices/vectors, expression templates, Eigen::Map and Eigen::Ref, but
   not for Eigen::Transform (cnoid::Affine3 / cnoid::Isometry3), because Transform
   does not derive from Eigen::EigenBase and is therefore not matched by any of
   them. The casters below add support for Affine3 and Isometry3 by routing the
   conversion through the 4x4 matrix caster: on the C++ -> Python direction the
   transform is exposed as its .matrix() (a 4x4 NumPy array), and on the
   Python -> C++ direction a 4x4 array is loaded into the transform's matrix.

   These casters only match the exact types cnoid::Affine3 / cnoid::Isometry3, so
   they do not interfere with the resolution of plain Matrix4 / Vector3 etc.
   arguments (those are handled by nanobind's own plain-matrix caster).

   Note: no validation is performed on the incoming 4x4 matrix (e.g. that the
   rotation part is orthonormal). A malformed matrix is accepted as-is, which is
   the same behavior as treating the value as a plain Matrix4.
*/

namespace nanobind {
namespace detail {

template<>
struct type_caster<cnoid::Affine3>
{
    using MatrixCaster = make_caster<cnoid::Matrix4>;
    NB_TYPE_CASTER(cnoid::Affine3, const_name("Affine3"))

    bool from_python(handle src, uint8_t flags, cleanup_list* cleanup) noexcept {
        MatrixCaster caster;
        if(!caster.from_python(src, flags, cleanup)){
            return false;
        }
        value.matrix() = caster.value;
        return true;
    }

    static handle from_cpp(const cnoid::Affine3& src, rv_policy policy, cleanup_list* cleanup) noexcept {
        return MatrixCaster::from_cpp(src.matrix(), policy, cleanup);
    }
};

template<>
struct type_caster<cnoid::Isometry3>
{
    using MatrixCaster = make_caster<cnoid::Matrix4>;
    NB_TYPE_CASTER(cnoid::Isometry3, const_name("Isometry3"))

    bool from_python(handle src, uint8_t flags, cleanup_list* cleanup) noexcept {
        MatrixCaster caster;
        if(!caster.from_python(src, flags, cleanup)){
            return false;
        }
        value.matrix() = caster.value;
        return true;
    }

    static handle from_cpp(const cnoid::Isometry3& src, rv_policy policy, cleanup_list* cleanup) noexcept {
        return MatrixCaster::from_cpp(src.matrix(), policy, cleanup);
    }
};

} // namespace detail
} // namespace nanobind

namespace cnoid {
namespace python {

/*
   A thin wrapper around a fixed-size Eigen vector used as a function argument
   type in the bindings. Its purpose is to accept a wider range of Python inputs
   than nanobind's own Eigen caster: in addition to NumPy arrays, a plain Python
   sequence (list / tuple) of the expected length is also accepted. This restores
   the convenience that pybind11 provided, where a list could be passed wherever
   an Eigen vector was expected.

   The wrapper holds the vector by value (rather than deriving from it) so that
   it is not itself an Eigen type; otherwise nanobind's own plain-matrix caster
   would also match it and clash with the caster defined below. Access the held
   vector through the 'value' member when passing it on to a C++ function.
*/
template<typename VectorType>
struct EigenVectorArg
{
    VectorType value;
    operator const VectorType&() const { return value; }
};

using Vector3Arg = EigenVectorArg<cnoid::Vector3>;
using Vector4Arg = EigenVectorArg<cnoid::Vector4>;
using Vector6Arg = EigenVectorArg<cnoid::Vector6>;
using Vector3fArg = EigenVectorArg<cnoid::Vector3f>;

/*
   The matrix counterpart of EigenVectorArg. In addition to NumPy arrays, it
   accepts a nested Python sequence (a sequence of rows, each a sequence of the
   row elements), e.g. [[1, 0, 0], [0, 1, 0], [0, 0, 1]] for a 3x3 matrix. The
   element assignment goes through Eigen's operator()(i, j), so the row/column
   storage order of MatrixType is handled transparently.
*/
template<typename MatrixType>
struct EigenMatrixArg
{
    MatrixType value;
    operator const MatrixType&() const { return value; }
};

using Matrix3Arg = EigenMatrixArg<cnoid::Matrix3>;
using Matrix4Arg = EigenMatrixArg<cnoid::Matrix4>;
using Matrix3fArg = EigenMatrixArg<cnoid::Matrix3f>;

// Row-major variants, used for the homogeneous transform / rotation setters that
// take an Eigen::Matrix<..., RowMajor> (the local Matrix4RM / Matrix3RM aliases
// in the binding sources).
using Matrix4RMArg = EigenMatrixArg<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>;
using Matrix3RMArg = EigenMatrixArg<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>;

} // namespace python
} // namespace cnoid

namespace nanobind {
namespace detail {

template<typename VectorType>
struct type_caster<cnoid::python::EigenVectorArg<VectorType>>
{
    using Wrapper = cnoid::python::EigenVectorArg<VectorType>;
    using Scalar = typename VectorType::Scalar;
    static constexpr int Size = VectorType::RowsAtCompileTime;
    using VectorCaster = make_caster<VectorType>;

    NB_TYPE_CASTER(Wrapper, const_name("Vector") + const_name<Size>())

    bool from_python(handle src, uint8_t flags, cleanup_list* cleanup) noexcept {
        // First, accept anything that nanobind's own Eigen caster accepts
        // (NumPy arrays etc.).
        VectorCaster caster;
        if(caster.from_python(src, flags, cleanup)){
            value.value = caster.value;
            return true;
        }

        // Otherwise, accept a Python sequence (list / tuple) of the right length.
        if(!PySequence_Check(src.ptr())){
            return false;
        }
        Py_ssize_t n = PySequence_Size(src.ptr());
        if(n != Size){
            return false;
        }
        for(Py_ssize_t i = 0; i < n; ++i){
            object item = steal(PySequence_GetItem(src.ptr(), i));
            if(!item.is_valid()){
                return false;
            }
            make_caster<Scalar> elementCaster;
            if(!elementCaster.from_python(item, flags, cleanup)){
                return false;
            }
            value.value[i] = elementCaster.value;
        }
        return true;
    }

    static handle from_cpp(const Wrapper& src, rv_policy policy, cleanup_list* cleanup) noexcept {
        return VectorCaster::from_cpp(src.value, policy, cleanup);
    }
};

template<typename MatrixType>
struct type_caster<cnoid::python::EigenMatrixArg<MatrixType>>
{
    using Wrapper = cnoid::python::EigenMatrixArg<MatrixType>;
    using Scalar = typename MatrixType::Scalar;
    static constexpr int Rows = MatrixType::RowsAtCompileTime;
    static constexpr int Cols = MatrixType::ColsAtCompileTime;
    using MatrixCaster = make_caster<MatrixType>;

    NB_TYPE_CASTER(Wrapper, const_name("Matrix") + const_name<Rows>() + const_name("x") + const_name<Cols>())

    bool from_python(handle src, uint8_t flags, cleanup_list* cleanup) noexcept {
        // First, accept anything that nanobind's own Eigen caster accepts
        // (NumPy arrays etc.).
        MatrixCaster caster;
        if(caster.from_python(src, flags, cleanup)){
            value.value = caster.value;
            return true;
        }

        // Otherwise, accept a nested Python sequence: a sequence of Rows rows,
        // each a sequence of Cols elements.
        if(!PySequence_Check(src.ptr())){
            return false;
        }
        Py_ssize_t nrows = PySequence_Size(src.ptr());
        if(nrows != Rows){
            return false;
        }
        for(Py_ssize_t i = 0; i < nrows; ++i){
            object row = steal(PySequence_GetItem(src.ptr(), i));
            if(!row.is_valid() || !PySequence_Check(row.ptr())){
                return false;
            }
            if(PySequence_Size(row.ptr()) != Cols){
                return false;
            }
            for(Py_ssize_t j = 0; j < Cols; ++j){
                object item = steal(PySequence_GetItem(row.ptr(), j));
                if(!item.is_valid()){
                    return false;
                }
                make_caster<Scalar> elementCaster;
                if(!elementCaster.from_python(item, flags, cleanup)){
                    return false;
                }
                value.value(i, j) = elementCaster.value;
            }
        }
        return true;
    }

    static handle from_cpp(const Wrapper& src, rv_policy policy, cleanup_list* cleanup) noexcept {
        return MatrixCaster::from_cpp(src.value, policy, cleanup);
    }
};

} // namespace detail
} // namespace nanobind

#endif
