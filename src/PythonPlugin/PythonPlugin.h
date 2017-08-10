/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_PYTHON_PLUGIN_PYTHON_PLUGIN_H
#define CNOID_PYTHON_PLUGIN_PYTHON_PLUGIN_H

#include <cnoid/PythonUtil>
#include "exportdecl.h"

namespace cnoid {

CNOID_EXPORT py::object pythonMainModule();
CNOID_EXPORT py::object pythonMainNamespace();
CNOID_EXPORT py::object pythonSysModule();

CNOID_EXPORT bool execPythonCode(const std::string& code);

}

#endif
