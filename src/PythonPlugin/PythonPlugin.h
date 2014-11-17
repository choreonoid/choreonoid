/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_PYTHON_PLUGIN_PYTHON_UTIL_H
#define CNOID_PYTHON_PLUGIN_PYTHON_UTIL_H

#include <boost/python.hpp>
#include "exportdecl.h"

namespace cnoid {

CNOID_EXPORT boost::python::object pythonMainModule();
CNOID_EXPORT boost::python::object pythonMainNamespace();
CNOID_EXPORT boost::python::object pythonSysModule();

CNOID_EXPORT bool execPythonCode(const std::string& code);

}

#endif
