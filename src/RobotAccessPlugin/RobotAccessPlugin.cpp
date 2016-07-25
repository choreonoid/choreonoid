/**
   @author Shin'ichiro Nakaoka
*/

#include "RobotAccessItem.h"
#include "RobotAccessBar.h"
#include <cnoid/Plugin>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {
    
class RobotAccessPlugin : public Plugin
{
public:
    RobotAccessPlugin();
    virtual bool initialize();
    virtual bool finalize();
};

};


RobotAccessPlugin::RobotAccessPlugin()
    : Plugin("RobotAccess")
{
    require("Body");
}


bool RobotAccessPlugin::initialize()
{
    addToolBar(new RobotAccessBar());

    return true;
}


bool RobotAccessPlugin::finalize()
{
    return true;
}
    

CNOID_IMPLEMENT_PLUGIN_ENTRY(RobotAccessPlugin)
