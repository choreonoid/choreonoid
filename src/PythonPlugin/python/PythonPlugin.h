#ifndef CNOID_PYTHON_PLUGIN_PYTHON_PLUGIN_H
#define CNOID_PYTHON_PLUGIN_PYTHON_PLUGIN_H

#include <cnoid/Plugin>
#include <Python.h>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT PythonPlugin : public Plugin
{
public:
    static PythonPlugin* instance();

    PythonPlugin();
    ~PythonPlugin();

    /**
       The following accessors return borrowed references. The GIL must be held
       while the returned objects are used.
    */
    PyObject* mainModule();
    PyObject* globalNamespace();
    PyObject* sysModule();
    PyObject* exitException();
    PyObject* rollbackImporterModule();

    virtual bool initialize() override;
    virtual bool finalize() override;

private:
    class Impl;
    Impl* impl;
};

}

#endif
