/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_PYTHON_PLUGIN_PYTHON_PLUGIN_H
#define CNOID_PYTHON_PLUGIN_PYTHON_PLUGIN_H

#include <cnoid/Config>

#ifdef CNOID_USE_PYBIND11
#include <pybind11/pybind11.h>
#else
#include <boost/python.hpp>
#endif

#include "exportdecl.h"

namespace cnoid {

#ifdef CNOID_USE_PYBIND11

typedef pybind11::gil_scoped_acquire PyGILock;

#else

namespace pybind11 {

using namespace boost::python;

class module : public object
{
public:
    module() { }
    module(const object& obj) : object(obj) { }
    
    static object import(str name){
        return boost::python::import(name);
    }
};

class gil_scoped_acquire
{
    PyGILState_STATE gstate;
public:
    gil_scoped_acquire(){
        gstate = PyGILState_Ensure();
    }
    ~gil_scoped_acquire() {
        PyGILState_Release(gstate);
    }
};

}
    
#endif

CNOID_EXPORT pybind11::object pythonMainModule();
CNOID_EXPORT pybind11::object pythonMainNamespace();
CNOID_EXPORT pybind11::object pythonSysModule();
CNOID_EXPORT bool execPythonCode(const std::string& code);
}

#endif
