/*!
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_PYSIGNAL_H
#define CNOID_UTIL_PYSIGNAL_H

#include "../Signal.h"
#include "PyUtil.h"

namespace cnoid {

template<typename T> boost::python::object pyGetSignalArgObject(T& value){
    return boost::python::object(value);
}

namespace signal_private {

template<typename T> struct python_function_caller0 {
    boost::python::object func;
    python_function_caller0(boost::python::object func) : func(func) { }
    T operator()() {
        python::gil_scoped_acquire lock;
        T result;
        try {
            result = func();
        } catch(boost::python::error_already_set const& ex) {
            python::handleException();
        }
        return result;
    }
};

template<> struct python_function_caller0<void> {
    boost::python::object func;
    python_function_caller0(boost::python::object func) : func(func) { }
    void operator()() {
        python::gil_scoped_acquire lock;
        try {
            func();
        } catch(boost::python::error_already_set const& ex) {
            python::handleException();
        }
    }
};

template<typename T, typename ARG1> struct python_function_caller1 {
    boost::python::object func;
    python_function_caller1(boost::python::object func) : func(func) { }
    T operator()(ARG1 arg1) {
        python::gil_scoped_acquire lock;
        T result;
        try {
            result = func(pyGetSignalArgObject(arg1));
        } catch(boost::python::error_already_set const& ex) {
            python::handleException();
        }
        return result;
    }
};

template<typename ARG1> struct python_function_caller1<void, ARG1> {
    boost::python::object func;
    python_function_caller1(boost::python::object func) : func(func) { }
    void operator()(ARG1 arg1) {
        python::gil_scoped_acquire lock;
        try {
            func(pyGetSignalArgObject(arg1));
        } catch(boost::python::error_already_set const& ex) {
            python::handleException();
        }
    }
};

template<typename T, typename ARG1, typename ARG2> struct python_function_caller2 {
    boost::python::object func;
    python_function_caller2(boost::python::object func) : func(func) { }
    T operator()(ARG1 arg1, ARG2 arg2) {
        python::gil_scoped_acquire lock;
        T result;
        try {
            result = func(pyGetSignalArgObject(arg1), pyGetSignalArgObject(arg2));
        } catch(boost::python::error_already_set const& ex) {
            python::handleException();
        }
        return result;
    }
};

template<typename ARG1, typename ARG2> struct python_function_caller2<void, ARG1, ARG2> {
    boost::python::object func;
    python_function_caller2(boost::python::object func) : func(func) { }
    void operator()(ARG1 arg1, ARG2 arg2) {
        python::gil_scoped_acquire lock;
        try {
            func(pyGetSignalArgObject(arg1), pyGetSignalArgObject(arg2));
        } catch(boost::python::error_already_set const& ex) {
            python::handleException();
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
    
    static Connection connect(SignalProxy<Signature, Combiner>& self, boost::python::object func){
        return self.connect(typename base_type::caller(func));
    }
public:
    PySignalProxy(const char* name) {
        boost::python::class_< SignalProxy<Signature, Combiner> >(name)
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
    
    static Connection connect(Signal<Signature, Combiner>& self, boost::python::object func){
        return self.connect(typename base_type::caller(func));
    }
    static Connection connectProxy(SignalProxy<Signature, Combiner>& self, boost::python::object func){
        return self.connect(typename base_type::caller(func));
    }
public:
    PySignal(const char* name) {

        boost::python::class_< Signal<Signature, Combiner>, boost::noncopyable >(name)
            .def("connect", &PySignal::connect);

        boost::python::class_< SignalProxy<Signature, Combiner> >((std::string(name) + "Proxy").c_str())
            .def("connect", &PySignal::connectProxy);
    }
};

}

#endif
