#include <cnoid/Plugin>

using namespace cnoid;

namespace {
  
class ManipulatorPlugin : public Plugin
{
public:
    ManipulatorPlugin() : Plugin("Manipulator")
    {

    }

    virtual bool initialize()
    {
        return true;
    }
};

}

CNOID_IMPLEMENT_PLUGIN_ENTRY(ManipulatorPlugin);
