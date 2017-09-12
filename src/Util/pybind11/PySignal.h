/*!
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_PYSIGNAL_H
#define CNOID_UTIL_PYSIGNAL_H

#include "../Signal.h"
#include <pybind11/pybind11.h>
#include <boost/type_traits.hpp>

namespace cnoid {

namespace signal_private {

template<typename T> struct python_function_caller0 {
    pybind11::object func;
    python_function_caller0(pybind11::object func) : func(func) { }
    T operator()() {
        pybind11::gil_scoped_acquire lock;
        T result;
        try {
            pybind11::object result0 = func();
            result = result0.cast<T>();
        } catch(const pybind11::error_already_set& ex) {
            pybind11::print(ex.what());
        }
        return result;
    }
};

template<> struct python_function_caller0<void> {
    pybind11::object func;
    python_function_caller0(pybind11::object func) : func(func) { }
    void operator()() {
        pybind11::gil_scoped_acquire lock;
        try {
            func();
        } catch(const pybind11::error_already_set& ex) {
            pybind11::print(ex.what());
        }
    }
};

template<typename T, typename ARG1> struct python_function_caller1 {
    pybind11::object func;
    python_function_caller1(pybind11::object func) : func(func) { }
    T operator()(ARG1 arg1) {
        pybind11::gil_scoped_acquire lock;
        T result;
        try {
            pybind11::object result0 = func(pybind11::cast(arg1));
            result = result0.cast<T>();
        } catch(const pybind11::error_already_set& ex) {
            pybind11::print(ex.what());
        }
        return result;
    }
};

template<typename ARG1> struct python_function_caller1<void, ARG1> {
    pybind11::object func;
    python_function_caller1(pybind11::object func) : func(func) { }
    void operator()(ARG1 arg1) {
        pybind11::gil_scoped_acquire lock;
        try {
            func(pybind11::cast(arg1));
        } catch(pybind11::error_already_set const& ex) {
            pybind11::print(ex.what());
        }
    }
};

template<typename T, typename ARG1, typename ARG2> struct python_function_caller2 {
    pybind11::object func;
    python_function_caller2(pybind11::object func) : func(func) { }
    T operator()(ARG1 arg1, ARG2 arg2) {
        pybind11::gil_scoped_acquire lock;
        T result;
        try {
            pybind11::object result0 = func(pybind11::cast(arg1), pybind11::cast(arg2));
            result = result0.cast<T>();
        } catch(const pybind11::error_already_set& ex) {
            pybind11::print(ex.what());
        }
        return result;
    }
};

template<typename ARG1, typename ARG2> struct python_function_caller2<void, ARG1, ARG2> {
    pybind11::object func;
    python_function_caller2(pybind11::object func) : func(func) { }
    void operator()(ARG1 arg1, ARG2 arg2) {
        pybind11::gil_scoped_acquire lock;
        try {
            func(pybind11::cast(arg1), pybind11::cast(arg2));
        } catch(const pybind11::error_already_set& ex) {
            pybind11::print(ex.what());
        }
    }
};


template<int Arity, typename Signature, typename Combiner>
class py_signal_impl;

template<typename Signature, typename Combiner>
class py_signal_impl<0, Signature, Combiner>
{
    typedef boost::function_traits<Signature> traits;
public:
    typedef python_function_caller0<typename traits::result_type> caller;
};

template<typename Signature, typename Combiner>
class py_signal_impl<1, Signature, Combiner>
{
    typedef boost::function_traits<Signature> traits;
public:
    typedef python_function_caller1<typename traits::result_type,
                                    typename traits::arg1_type> caller;
};

template<typename Signature, typename Combiner>
class py_signal_impl<2, Signature, Combiner>
{
    typedef boost::function_traits<Signature> traits;
public:
    typedef python_function_caller2<typename traits::result_type,
                                    typename traits::arg1_type,
                                    typename traits::arg2_type> caller;
};

} // namespace signal_private

template<
    typename Signature, 
    typename Combiner = signal_private::last_value<typename boost::function_traits<Signature>::result_type>
    >
class PySignal : public signal_private::py_signal_impl<
    (boost::function_traits<Signature>::arity), Signature, Combiner>
{
    typedef Signal<Signature, Combiner> SignalType;
    typedef SignalProxy<Signature, Combiner> SignalProxyType;
    typedef signal_private::py_signal_impl<(boost::function_traits<Signature>::arity), Signature, Combiner> base_type;
    
public:
    PySignal(pybind11::module& m, const std::string& name)
    {
        pybind11::class_<SignalType>(m, name.c_str())
            .def("connect", [](SignalType& self, pybind11::object func){
                    return self.connect(typename base_type::caller(func)); });

        pybind11::class_<SignalProxyType>(m, (name + "Proxy").c_str())
            .def(pybind11::init<const SignalProxyType&>())
            .def("connect", [](SignalProxyType& self, pybind11::object func){
                    return self.connect(typename base_type::caller(func)); });
    }
};

}

#endif
