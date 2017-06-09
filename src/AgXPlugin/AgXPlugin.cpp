/*!
  @file
  @author Shizuko Hattori
*/

#include "AgXSimulatorItem.h"
#include <agxSDK/Simulation.h>
#include <cnoid/Plugin>

using namespace std;
using namespace cnoid;

namespace {

agx::AutoInit agxInit;

class AgXPlugin : public Plugin
{
public:
    agx::AutoInit agxInit;

    AgXPlugin() : Plugin("AgX")
        { 
            require("Body");
        }
        
    virtual ~AgXPlugin()
        {

        }

    virtual bool initialize()
        {
            AgXSimulatorItem::initializeClass(this);

            return true;
        }
        
    virtual bool finalize()
        {
            return true;
        }

};
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(AgXPlugin);
