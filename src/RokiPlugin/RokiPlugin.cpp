/*! @file
  @author Shizuko Hattori
*/

#include "RokiSimulatorItem.h"
#include <cnoid/Plugin>

using namespace cnoid;

namespace {
  
class RokiPlugin : public Plugin
{
public:
    RokiPlugin() : Plugin("Roki") {
        require("Body");
    }

    virtual bool initialize(){
        RokiSimulatorItem::initializeClass(this);
        return true;
    }

    virtual bool finalize()
    {
        return true;
    }
};
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(RokiPlugin);
