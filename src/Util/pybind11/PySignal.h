#ifndef CNOID_UTIL_PYSIGNAL_H
#define CNOID_UTIL_PYSIGNAL_H

#include "../Signal.h"
#include <pybind11/pybind11.h>
#include <functional>

/**
   The following code modifies the type_caster code written in the function header of pybind11.
   The header supports the conversion between a python function and a C++ std::function.
   When a python function is called from C++, if there is an error on the python side,
   the resulting exception will cause the process to crash because the exception is not
   catched in the converted function. In the following code, the exeption handler is added
   to avoid the crash.
   Note that this code cannot be used with the original pybind11 function header.
*/
   
/*
   pybind11/functional.h: std::function<> support

   Copyright (c) 2016 Wenzel Jakob <wenzel.jakob@epfl.ch>

   All rights reserved. Use of this source code is governed by a
   BSD-style license that can be found in the LICENSE file.
*/

PYBIND11_NAMESPACE_BEGIN(PYBIND11_NAMESPACE)
PYBIND11_NAMESPACE_BEGIN(detail)

template <typename Return, typename... Args>
struct type_caster<std::function<Return(Args...)>> {
    using type = std::function<Return(Args...)>;
    using retval_type = conditional_t<std::is_same<Return, void>::value, void_type, Return>;
    using function_type = Return (*) (Args...);

public:
    bool load(handle src, bool convert) {
        if (src.is_none()) {
            // Defer accepting None to other overloads (if we aren't in convert mode):
            if (!convert) return false;
            return true;
        }

        if (!isinstance<function>(src))
            return false;

        auto func = reinterpret_borrow<function>(src);

        /*
           When passing a C++ function as an argument to another C++
           function via Python, every function call would normally involve
           a full C++ -> Python -> C++ roundtrip, which can be prohibitive.
           Here, we try to at least detect the case where the function is
           stateless (i.e. function pointer or lambda function without
           captured variables), in which case the roundtrip can be avoided.
         */
        if (auto cfunc = func.cpp_function()) {
            auto c = reinterpret_borrow<capsule>(PyCFunction_GET_SELF(cfunc.ptr()));
            auto rec = (function_record *) c;

            if (rec && rec->is_stateless &&
                    same_type(typeid(function_type), *reinterpret_cast<const std::type_info *>(rec->data[1]))) {
                struct capture { function_type f; };
                value = ((capture *) &rec->data)->f;
                return true;
            }
        }

        // ensure GIL is held during functor destruction
        struct func_handle {
            function f;
            func_handle(function&& f_) : f(std::move(f_)) {}
            func_handle(const func_handle& f_) {
                gil_scoped_acquire acq;
                f = f_.f;
            }
            ~func_handle() {
                gil_scoped_acquire acq;
                try {
                    function kill_f(std::move(f));
                }
                catch(const error_already_set& ex) {
                    pybind11::print(ex.what());
                }
            }
        };

        // to emulate 'move initialization capture' in C++11
        struct func_wrapper {
            func_handle hfunc;
            func_wrapper(func_handle&& hf): hfunc(std::move(hf)) {}
            Return operator()(Args... args) const {
                gil_scoped_acquire acq;
                try {
                    object retval(hfunc.f(std::forward<Args>(args)...));
                    /* Visual studio 2015 parser issue: need parentheses around this expression */
                    return (retval.template cast<Return>());
                }
                catch(const error_already_set& ex) {
                    pybind11::print(ex.what());
                }
                return Return();
            }
        };

        value = func_wrapper(func_handle(std::move(func)));
        return true;
    }

    template <typename Func>
    static handle cast(Func &&f_, return_value_policy policy, handle /* parent */) {
        if (!f_)
            return none().inc_ref();

        auto result = f_.template target<function_type>();
        if (result)
            return cpp_function(*result, policy).release();
        else
            return cpp_function(std::forward<Func>(f_), policy).release();
    }

    PYBIND11_TYPE_CASTER(type, _("Callable[[") + concat(make_caster<Args>::name...) + _("], ")
                               + make_caster<retval_type>::name + _("]"));
};

PYBIND11_NAMESPACE_END(detail)
PYBIND11_NAMESPACE_END(PYBIND11_NAMESPACE)


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
    PySignal(pybind11::module& m, const std::string& name)
    {
        pybind11::class_<SignalType>(m, name.c_str())
            .def("connect",
                 [](SignalType& self, std::function<Signature> func){
                     return self.connect(func); });

        pybind11::class_<SignalProxyType>(m, (name + "Proxy").c_str())
            .def(pybind11::init<const SignalProxyType&>())
                 .def("connect",
                      [](SignalProxyType& self, std::function<Signature> func){
                          return self.connect(func); });
    }
};

}

#endif
