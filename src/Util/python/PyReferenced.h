#ifndef CNOID_UTIL_PYTHON_PYREFERENCED_H
#define CNOID_UTIL_PYTHON_PYREFERENCED_H

#include "../Referenced.h"
#include <nanobind/nanobind.h>

/*
   Lifetime management of cnoid::Referenced-derived objects under nanobind.

   We use nanobind's built-in intrusive ownership mechanism. The Referenced base
   class is registered with nb::intrusive_ptr in PyReferenced.cpp; that installs
   a "set_self_py" hook (calling Referenced::setSelfPython) which nanobind
   inherits to every derived type and invokes whenever an object is first bound
   to a Python wrapper - whether the object was created in C++ and returned to
   Python, or created on the Python side via its constructor binding. After that
   hook runs the object is in binding-language mode: its reference counting is
   delegated to the Python wrapper, and nanobind owns and frees the memory. This
   is what makes Python-created objects (which nanobind embeds inside the wrapper
   allocation) safe - the C++ side never operator-deletes such memory.

   Referenced's reference counting is driven through the function pointers
   installed by initReferencedPythonInterface() (see PyReferenced.cpp), whose
   decref carries the nb::is_alive() guard that prevents touching a finalized
   interpreter at process exit.

   The only caster we need here is for cnoid::ref_ptr<T>, so that C++ functions
   taking or returning ref_ptr<T> can be bound. A bare T* (T derived from
   Referenced) is already handled by nanobind's normal class caster, which - via
   the intrusive hook - routes the object through setSelfPython as well. Modeled
   on nanobind/intrusive/ref.h.
*/

namespace nanobind {
namespace detail {

template<typename T>
struct type_caster<cnoid::ref_ptr<T>>
{
    using Caster = make_caster<T>;
    static constexpr bool IsClass = true;
    NB_TYPE_CASTER(cnoid::ref_ptr<T>, Caster::Name)

    bool from_python(handle src, uint8_t flags, cleanup_list* cleanup) noexcept {
        Caster caster;
        if(!caster.from_python(src, flags, cleanup)){
            return false;
        }
        value = Value(caster.operator T*());
        return true;
    }

    static handle from_cpp(const cnoid::ref_ptr<T>& value, rv_policy policy,
                           cleanup_list* cleanup) noexcept {
        // If the object is already bound to a Python wrapper, return that same
        // wrapper (preserving identity), as long as we are not asked to copy/move.
        if(policy != rv_policy::copy && policy != rv_policy::move && value.get()){
            if(void* self = value->selfPython()){
                return handle(static_cast<PyObject*>(self)).inc_ref();
            }
        }
        return Caster::from_cpp(value.get(), policy, cleanup);
    }
};

} // namespace detail
} // namespace nanobind

#endif
