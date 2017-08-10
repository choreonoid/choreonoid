/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_PYTHON_PYTHON_UTIL_H
#define CNOID_PYTHON_PYTHON_UTIL_H

#include <cnoid/Config>

#ifdef CNOID_USE_PYBIND11
#include <pybind11/pybind11.h>
#else
#include <boost/python.hpp>
#endif

#include "exportdecl.h"

namespace cnoid {

#ifdef CNOID_USE_PYBIND11
namespace py = pybind11;

#else
namespace py {

using namespace boost::python;

class module : public object
{
public:
    module();
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
    
typedef py::gil_scoped_acquire PyGILock;

#endif

CNOID_EXPORT void handlePythonException();

}

#endif
