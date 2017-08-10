/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_PYTHON3_PLUGIN_PYTHON3_PLUGIN_H
#define CNOID_PYTHON3_PLUGIN_PYTHON3_PLUGIN_H

#include <pybind11/pybind11.h>
#include "exportdecl.h"

namespace cnoid {

CNOID_EXPORT pybind11::object pythonMainModule();
CNOID_EXPORT pybind11::object pythonMainNamespace();
CNOID_EXPORT pybind11::object pythonSysModule();

CNOID_EXPORT bool execPythonCode(const std::string& code);

}

#endif
