/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_PYTHON_PYTHON_UTIL_H
#define CNOID_PYTHON_PYTHON_UTIL_H

#include <cnoid/Config>

#ifdef CNOID_USE_BOOST_PYTHON
#include "exportdecl.h"
namespace cnoid {
CNOID_EXPORT void handlePythonException();
}
#endif

#endif
