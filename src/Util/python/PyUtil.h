#ifndef CNOID_UTIL_PYTHON_PYUTIL_H
#define CNOID_UTIL_PYTHON_PYUTIL_H

#include <cnoid/Config>
#include "PyReferenced.h"

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

namespace cnoid {
namespace python {
namespace nb = nanobind;

// Release a Python-created object from nanobind's ownership so that nanobind
// does not destroy the underlying C++ object when the Python wrapper is
// collected. This is used when ownership of the object is handed over to C++
// (e.g. a Qt widget added to a parent, a tool bar mounted to the tool bar area).
// See the nanobind-qt-widget-ownership note for the rationale.
//
// nb::cnoid_relinquish_all_wrappers() (a Choreonoid-added nanobind helper, see
// thirdparty/.../CHOREONOID_PATCHES.md) is used instead of nb::inst_set_state()
// so that ownership is released from *every* wrapper that refers to the same
// C++ object. A Python subclass of a bound C++ type created with nb::new_()
// (e.g. "class MyBar(ToolBar)") has two wrappers for one C++ object: the visible
// subclass wrapper (non-owning) and a hidden base-type wrapper that owns the
// object. Releasing only the visible one would leave the hidden owner with
// destruct = true and cause a double free when C++ (Qt) also deletes the object.
inline void releaseOwnership(nb::handle obj)
{
    if(obj.is_valid() && !obj.is_none()){
        nb::cnoid_relinquish_all_wrappers(obj);
    }
}

// An argument type for binding functions that take over the ownership of a
// Python-created object. Declaring an argument as OwnershipReleased<T> makes the
// type caster release the object from nanobind's ownership during the argument
// conversion, so the binding lambda does not need to call releaseOwnership()
// explicitly. The held pointer is obtained through the implicit conversion to T*.
template<typename T>
struct OwnershipReleased
{
    T* pointer;
    operator T*() const { return pointer; }
    T* operator->() const { return pointer; }
};

}
}

namespace nanobind {
namespace detail {

template<typename T>
struct type_caster<cnoid::python::OwnershipReleased<T>>
{
    using Wrapper = cnoid::python::OwnershipReleased<T>;
    using PointerCaster = make_caster<T*>;

    NB_TYPE_CASTER(Wrapper, PointerCaster::Name)

    bool from_python(handle src, uint8_t flags, cleanup_list* cleanup) noexcept {
        PointerCaster caster;
        if(!caster.from_python(src, flags, cleanup)){
            return false;
        }
        value.pointer = caster.operator T*();
        cnoid::python::releaseOwnership(src);
        return true;
    }
};

} // namespace detail
} // namespace nanobind

#endif
