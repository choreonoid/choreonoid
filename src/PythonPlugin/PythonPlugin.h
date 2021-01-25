#ifndef CNOID_PYTHON_PLUGIN_PYTHON_PLUGIN_H
#define CNOID_PYTHON_PLUGIN_PYTHON_PLUGIN_H

#include <cnoid/Plugin>
#include <cnoid/PyUtil>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT PythonPlugin : public Plugin
{
public:
    static PythonPlugin* instance();

    PythonPlugin();

    python::module mainModule();
    python::object globalNamespace();
    python::module sysModule();
    python::object exitException();
    python::module rollbackImporterModule();

    virtual bool initialize() override;
    virtual bool finalize() override;

private:
    class Impl;
    Impl* impl;
};

}

#endif
