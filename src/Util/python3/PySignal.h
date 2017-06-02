/*!
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_PYSIGNAL_H
#define CNOID_UTIL_PYSIGNAL_H

#include "../Signal.h"
#include "PyUtil.h"
#include <cnoid/PythonUtil>
#include <boost/type_traits.hpp>

namespace cnoid {

template<typename T> pybind11::object pyGetSignalArgObject(T& value){
    return pybind11::cast(value);
}

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
        } catch(pybind11::error_already_set const& ex) {
            cnoid::handlePythonException();
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
        } catch(pybind11::error_already_set const& ex) {
            cnoid::handlePythonException();
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
            pybind11::object result0 = func(pyGetSignalArgObject(arg1));
            result = result0.cast<T>();
        } catch(pybind11::error_already_set const& ex) {
            handlePythonException();
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
            func(pyGetSignalArgObject(arg1));
        } catch(pybind11::error_already_set const& ex) {
            handlePythonException();
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
            pybind11::object result0 = func(pyGetSignalArgObject(arg1), pyGetSignalArgObject(arg2));
            result = result0.cast<T>();
        } catch(pybind11::error_already_set const& ex) {
            handlePythonException();
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
            func(pyGetSignalArgObject(arg1), pyGetSignalArgObject(arg2));
        } catch(pybind11::error_already_set const& ex) {
            handlePythonException();
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
class PySignalProxy : public signal_private::py_signal_impl<
    (boost::function_traits<Signature>::arity), Signature, Combiner>
{
    typedef signal_private::py_signal_impl<(boost::function_traits<Signature>::arity), Signature, Combiner> base_type;
    
    static Connection connect(SignalProxy<Signature, Combiner>& self, pybind11::object func){
        return self.connect(typename base_type::caller(func));
    }
public:
    PySignalProxy(const char* name) {
        pybind11::class_< SignalProxy<Signature, Combiner> >(name)
            .def("connect", &PySignalProxy::connect);
    }
};


template<
    typename Signature, 
    typename Combiner = signal_private::last_value<typename boost::function_traits<Signature>::result_type>
    >
class PySignal : public signal_private::py_signal_impl<
    (boost::function_traits<Signature>::arity), Signature, Combiner>
{
    typedef signal_private::py_signal_impl<(boost::function_traits<Signature>::arity), Signature, Combiner> base_type;
    
    static Connection connect(Signal<Signature, Combiner>& self, pybind11::object func){
        return self.connect(typename base_type::caller(func));
    }
    static Connection connectProxy(SignalProxy<Signature, Combiner>& self, pybind11::object func){
        return self.connect(typename base_type::caller(func));
    }
public:
    PySignal(pybind11::module& m, const char* name) {

        pybind11::class_< Signal<Signature, Combiner> >(m, name)
            .def("connect", &PySignal::connect);

        pybind11::class_< SignalProxy<Signature, Combiner> >(m, (std::string(name) + "Proxy").c_str())
            .def("connect", &PySignal::connectProxy);
    }
};

}

#endif
