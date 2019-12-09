#include "ManipulatorProgramItemBase.h"
#include "ManipulatorVariableListItemBase.h"
#include "ManipulatorControllerItemBase.h"
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
        ManipulatorProgramItemBase::initializeClass(this);
        ManipulatorVariableListItemBase::initializeClass(this);
        ManipulatorControllerItemBase::initializeClass(this);
        
        return true;
    }
};

}

CNOID_IMPLEMENT_PLUGIN_ENTRY(ManipulatorPlugin);
