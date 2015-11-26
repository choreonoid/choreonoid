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
       //     string pathToAgx = "/opt/Algoryx/AgX-2.13.2.2";
      //      agxIO::Environment::instance()->getFilePath(agxIO::Environment::RESOURCE_PATH).pushbackPath(pathToAgx);
       //     agxIO::Environment::instance()->getFilePath(agxIO::Environment::RESOURCE_PATH).addFilePath(pathToAgx);
      //      agxIO::Environment::instance()->getFilePath(agxIO::Environment::RUNTIME_PATH).addFilePath(pathToAgx+"/bin/plugins");
       //     agxIO::Environment::instance()->getFilePath(agxIO::Environment::RESOURCE_PATH).addFilePath(pathToAgx+"/data");

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
