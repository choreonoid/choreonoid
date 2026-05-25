#ifndef CNOID_UTIL_PYTHON_PYNUMPYHELPER_H
#define CNOID_UTIL_PYTHON_PYNUMPYHELPER_H

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <Eigen/Core>

/*
   Helpers that expose an Eigen dense expression (a plain matrix/vector, a
   Block such as Isometry3::TranslationPart, an Eigen::Map, ...) to Python as a
   writable, zero-copy NumPy view that shares memory with the C++ object.

   This mirrors C++ semantics: wherever a C++ getter returns a non-const lvalue
   reference / writable block (so that C++ code can modify the value in place),
   the corresponding Python accessor returns a NumPy array writing through to
   the same memory. nanobind's own Eigen caster only does this for plain-matrix
   lvalue references; for Eigen block expressions it falls back to a copy, and
   even its Eigen::Map caster attaches no owner (dangling risk). These helpers
   build the nb::ndarray directly with an explicit owner so the lifetime is tied
   to the C++ object that backs the memory.

   The strides passed to nb::ndarray are in element units (not bytes), matching
   Eigen's innerStride()/rowStride()/colStride(), so column- vs row-major
   storage is handled transparently.

   'owner' must be the Python wrapper that keeps the backing C++ object alive
   (typically nb::find(self) of the bound object whose memory we are viewing).
*/

namespace cnoid {
namespace python {

namespace nb = nanobind;

// 1D view (vector). Derived is any Eigen dense expression with rows()/cols()
// being a vector shape and data()/innerStride() available (plain vector, a
// column/row block, etc.).
template<typename Derived>
nb::object eigenVectorView(const Eigen::DenseBase<Derived>& expr_, nb::handle owner)
{
    auto& expr = const_cast<Eigen::DenseBase<Derived>&>(expr_);
    using Scalar = typename Derived::Scalar;
    const size_t n = (size_t)(expr.rows() * expr.cols());
    const int64_t stride = (int64_t)expr.derived().innerStride();
    size_t shape[1] = { n };
    int64_t strides[1] = { stride };
    return nb::cast(
        nb::ndarray<nb::numpy, Scalar, nb::ndim<1>>(
            (void*)expr.derived().data(), 1, shape, owner, strides),
        nb::rv_policy::reference);
}

// 2D view (matrix/block). 'expr' must provide data()/rowStride()/colStride().
template<typename Derived>
nb::object eigenMatrixView(const Eigen::DenseBase<Derived>& expr_, nb::handle owner)
{
    auto& expr = const_cast<Eigen::DenseBase<Derived>&>(expr_);
    using Scalar = typename Derived::Scalar;
    size_t shape[2] = { (size_t)expr.rows(), (size_t)expr.cols() };
    int64_t strides[2] = {
        (int64_t)expr.derived().rowStride(),
        (int64_t)expr.derived().colStride()
    };
    return nb::cast(
        nb::ndarray<nb::numpy, Scalar, nb::ndim<2>>(
            (void*)expr.derived().data(), 2, shape, owner, strides),
        nb::rv_policy::reference);
}

// 1D view over a contiguous raw buffer (e.g. std::vector<int>::data()).
template<typename Scalar>
nb::object bufferView1d(Scalar* data, size_t n, nb::handle owner)
{
    size_t shape[1] = { n };
    return nb::cast(
        nb::ndarray<nb::numpy, Scalar, nb::ndim<1>>((void*)data, 1, shape, owner),
        nb::rv_policy::reference);
}

// 2D view over a contiguous raw buffer laid out row-major as (rows, cols),
// e.g. an SgVectorArray<Vector3f> seen as (N, 3) of float.
template<typename Scalar>
nb::object bufferView2d(Scalar* data, size_t rows, size_t cols, nb::handle owner)
{
    size_t shape[2] = { rows, cols };
    return nb::cast(
        nb::ndarray<nb::numpy, Scalar, nb::ndim<2>>((void*)data, 2, shape, owner),
        nb::rv_policy::reference);
}

} // namespace python
} // namespace cnoid

#endif
