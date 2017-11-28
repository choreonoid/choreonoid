#include "AGXSubSimulatorItem.h"
#include <cnoid/Plugin>

using namespace cnoid;

class AGXSubSimulatorPlugin : public Plugin
{
public:

    AGXSubSimulatorPlugin() : Plugin("AGXSubSimulator") {
        require("AGXDynamics");
    }

    virtual bool initialize() {
        AGXSubSimulatorItem::initializeClass(this);
        return true;
    }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(AGXSubSimulatorPlugin);