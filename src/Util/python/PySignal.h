#ifndef CNOID_UTIL_PYTHON_PYSIGNAL_H
#define CNOID_UTIL_PYTHON_PYSIGNAL_H

#include "../Signal.h"
#include <nanobind/nanobind.h>
#include <functional>
#include <utility>

/*
   The following code is a modified version of the std::function type caster
   provided by nanobind (nanobind/stl/function.h). The original converts between
   a Python callable and a C++ std::function. When such a converted function is
   invoked from C++ and an error occurs on the Python side, the resulting
   exception propagates into the C++ caller. In Choreonoid a Python slot is
   called from C++ signal emission, where an uncaught C++ exception would crash
   the process. The modified call operator below catches the Python error,
   reports it through sys.unraisablehook, and returns a default-constructed
   value so that signal emission can continue safely.

   Because this redefines type_caster<std::function<...>>, a translation unit
   that uses PySignal must not also include <nanobind/stl/function.h>.

   Original copyright:
     nanobind/stl/function.h: type caster for std::function<...>
     Copyright (c) 2022 Wenzel Jakob
     All rights reserved. Use of this source code is governed by a
     BSD-style license that can be found in the LICENSE file.
*/

NAMESPACE_BEGIN(NB_NAMESPACE)
NAMESPACE_BEGIN(detail)

struct cnoid_pyfunc_wrapper {
    PyObject *f;

    explicit cnoid_pyfunc_wrapper(PyObject *f) : f(f) {
        Py_INCREF(f);
    }

    cnoid_pyfunc_wrapper(cnoid_pyfunc_wrapper &&w) noexcept : f(w.f) {
        w.f = nullptr;
    }

    cnoid_pyfunc_wrapper(const cnoid_pyfunc_wrapper &w) : f(w.f) {
        if (f) {
            gil_scoped_acquire acq;
            Py_INCREF(f);
        }
    }

    ~cnoid_pyfunc_wrapper() {
        if (f) {
            gil_scoped_acquire acq;
            Py_DECREF(f);
        }
    }

    cnoid_pyfunc_wrapper &operator=(const cnoid_pyfunc_wrapper) = delete;
    cnoid_pyfunc_wrapper &operator=(cnoid_pyfunc_wrapper &&) = delete;
};

template <typename Return, typename... Args>
struct type_caster<std::function<Return(Args...)>> {
    using ReturnCaster = make_caster<
        std::conditional_t<std::is_void_v<Return>, void_type, Return>>;

    NB_TYPE_CASTER(std::function <Return(Args...)>,
                   const_name("collections.abc.Callable[[") +
                       concat(make_caster<Args>::Name...) + const_name("], ") +
                       ReturnCaster::Name + const_name("]"))

    struct func_wrapper_t : cnoid_pyfunc_wrapper {
        using cnoid_pyfunc_wrapper::cnoid_pyfunc_wrapper;

        Return operator()(Args... args) const {
            gil_scoped_acquire acq;
            handle func(f);
            try {
                if constexpr (std::is_void_v<Return>) {
                    func((forward_t<Args>) args...);
                } else {
                    return cast<Return>(func((forward_t<Args>) args...));
                }
            } catch (python_error &e) {
                e.discard_as_unraisable("Choreonoid signal slot");
            }
            if constexpr (!std::is_void_v<Return>) {
                return Return();
            }
        }
    };

    bool from_python(handle src, uint8_t flags, cleanup_list *) noexcept {
        if (src.is_none())
            return flags & cast_flags::convert;

        if (!PyCallable_Check(src.ptr()))
            return false;

        value = func_wrapper_t(src.ptr());

        return true;
    }

    static handle from_cpp(const Value &value, rv_policy rvp,
                           cleanup_list *) noexcept {
        const func_wrapper_t *wrapper = value.template target<func_wrapper_t>();
        if (wrapper)
            return handle(wrapper->f).inc_ref();

        if (rvp == rv_policy::none)
            return handle();

        if (!value)
            return none().release();

        return cpp_function(value).release();
    }
};

NAMESPACE_END(detail)
NAMESPACE_END(NB_NAMESPACE)


namespace cnoid {

template<
    typename Signature,
    typename Combiner = signal_private::last_value<
        typename signal_private::function_traits<Signature>::result_type>
    >
class PySignal
{
    typedef Signal<Signature, Combiner> SignalType;
    typedef SignalProxy<Signature, Combiner> SignalProxyType;

public:
    PySignal(nanobind::module_& m, const std::string& name)
    {
        namespace nb = nanobind;

        nb::class_<SignalType>(m, name.c_str())
            .def("connect",
                 [](SignalType& self, std::function<Signature> func){
                     return self.connect(func); });

        nb::class_<SignalProxyType>(m, (name + "Proxy").c_str())
            .def(nb::init<const SignalProxyType&>())
            .def("connect",
                 [](SignalProxyType& self, std::function<Signature> func){
                     return self.connect(func); });
    }
};

}

#endif
