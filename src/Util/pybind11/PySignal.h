/*!
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_PYSIGNAL_H
#define CNOID_UTIL_PYSIGNAL_H

#include "../Signal.h"
#include <pybind11/functional.h>

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
