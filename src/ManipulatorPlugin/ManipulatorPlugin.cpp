#include "ManipulatorCoordinateFrameItem.h"
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
        ManipulatorCoordinateFrameItem::initializeClass(this);
        return true;
    }
};

}

CNOID_IMPLEMENT_PLUGIN_ENTRY(ManipulatorPlugin);
