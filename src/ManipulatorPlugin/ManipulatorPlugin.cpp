#include "MprProgramItemBase.h"
#include "MprVariableListItemBase.h"
#include "MprControllerItemBase.h"
#include <cnoid/Plugin>

using namespace cnoid;

namespace {
  
class ManipulatorPlugin : public Plugin
{
public:
    ManipulatorPlugin() : Plugin("Manipulator")
    {
        require("Body");
    }

    virtual bool initialize()
    {
        MprProgramItemBase::initializeClass(this);
        MprVariableListItemBase::initializeClass(this);
        MprControllerItemBase::initializeClass(this);
        
        return true;
    }
};

}

CNOID_IMPLEMENT_PLUGIN_ENTRY(ManipulatorPlugin);
