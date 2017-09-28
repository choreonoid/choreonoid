/*! @file
  @author Shin'ichiro Nakaoka
*/

#include "PythonSimScriptItem.h"
#include <cnoid/Plugin>

using namespace cnoid;

namespace {

class PythonSimScriptPlugin : public Plugin
{
public:
    PythonSimScriptPlugin() : Plugin("PythonSimScript") {
        require("Body");
        require("Python");
    }

    virtual bool initialize() {
        PythonSimScriptItem::initialize(this);
        return true;
    }
};

}

CNOID_IMPLEMENT_PLUGIN_ENTRY(PythonSimScriptPlugin);
