/**
   \file
   \author Ikumi Susa
*/

#include <cnoid/Plugin>

namespace cnoid{

class AGXBodyExtensionPlugin : public Plugin
{
public:

    AGXBodyExtensionPlugin() : Plugin("AGXBodyExtension") {
        require("AGXDynamics");
    }

    virtual bool initialize() {
        return true;
    }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(AGXBodyExtensionPlugin)

}