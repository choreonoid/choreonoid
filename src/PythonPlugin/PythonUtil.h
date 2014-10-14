/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_PYTHON_PLUGIN_PYTHON_UTIL_H_INCLUDED
#define CNOID_PYTHON_PLUGIN_PYTHON_UTIL_H_INCLUDED

#include <boost/python.hpp>
#include "exportdecl.h"

namespace cnoid {

class PyGILock
{
    PyGILState_STATE gstate;
public:
    PyGILock(){
        gstate = PyGILState_Ensure();
    }
    ~PyGILock() {
        PyGILState_Release(gstate);
    }
};

CNOID_EXPORT boost::python::object pythonMainModule();
CNOID_EXPORT boost::python::object pythonMainNamespace();
CNOID_EXPORT boost::python::object pythonSysModule();

CNOID_EXPORT bool execPythonCode(const std::string& code);

}

#endif
