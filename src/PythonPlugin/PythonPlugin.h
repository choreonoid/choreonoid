/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_PYTHON_PLUGIN_PYTHON_PLUGIN_H
#define CNOID_PYTHON_PLUGIN_PYTHON_PLUGIN_H

#include <cnoid/PyUtil>
#include "exportdecl.h"

namespace cnoid {

CNOID_EXPORT python::object pythonMainModule();
CNOID_EXPORT python::object pythonMainNamespace();
CNOID_EXPORT python::object pythonSysModule();
CNOID_EXPORT python::object pythonExitExceptionType();
CNOID_EXPORT python::object pythonRollBackImporterModule();
CNOID_EXPORT bool execPythonCode(const std::string& code);

#ifdef CNOID_USE_BOOST_PYTHON
CNOID_EXPORT python::object pythonStringOutBufClass();
#endif

}

#endif
