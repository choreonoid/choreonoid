#include "AGXSimulatorItem.h"
#include <agxSDK/Simulation.h>
#include <cnoid/Plugin>

using namespace std;
using namespace cnoid;

namespace {

class AGXDynamicsPlugin : public Plugin
{
public:
    agx::AutoInit agxInit;

    AGXDynamicsPlugin() : Plugin("AGXDynamics")
    { 
        require("Body");
    }
        
    virtual ~AGXDynamicsPlugin()
    {

    }

    virtual bool initialize()
    {
        AGXSimulatorItem::initializeClass(this);

        return true;
    }
        
    virtual bool finalize()
    {
        return true;
    }

};

}

CNOID_IMPLEMENT_PLUGIN_ENTRY(AGXDynamicsPlugin);
