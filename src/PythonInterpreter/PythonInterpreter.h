#ifndef CNOID_PYTHON_PYTHON_INTERPRETER_H
#define CNOID_PYTHON_PYTHON_INTERPRETER_H

#include <Python.h>
#include "exportdecl.h"

namespace cnoid {

/**
   Returns whether the embedded CPython interpreter has been successfully
   initialized by this library. The PythonPlugin and the Python binding modules
   can check this to know that a usable interpreter is available.
*/
CNOID_EXPORT bool isPythonAvailable();

/**
   The following accessors return borrowed references to the basic objects of
   the embedded interpreter. The GIL must be held while the returned objects are
   used. They return nullptr when the interpreter is not available.
*/
CNOID_EXPORT PyObject* pythonMainModule();
CNOID_EXPORT PyObject* pythonGlobalNamespace();
CNOID_EXPORT PyObject* pythonSysModule();

}

/**
   Entry points loaded dynamically by the Base App with dlsym / QLibrary::resolve.
   They are declared with C linkage so that their symbol names are stable. The
   initialization function returns a non-zero value on success.

   cnoid_initializePythonInterpreter() boots a bare CPython interpreter (no
   Choreonoid-specific integration such as the MessageView redirection, which is
   set up later by the PythonPlugin). cnoid_finalizePythonInterpreter() must be
   called as the very last step of the App shutdown, after all plugins have been
   finalized and the item tree has been released, so that finalizing the
   interpreter (which forcibly destroys the Python wrappers that still own
   Referenced-derived C++ objects under the nanobind backend) does not run any
   C++ destructor while other shutdown code is still executing.
*/
extern "C" CNOID_PYTHON_INTERPRETER_DLLEXPORT int cnoid_initializePythonInterpreter();
extern "C" CNOID_PYTHON_INTERPRETER_DLLEXPORT void cnoid_finalizePythonInterpreter();

#endif
