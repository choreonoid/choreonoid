#include "GLVisionSimulatorItem.h"
#include <cnoid/Plugin>

using namespace cnoid;

namespace {

class GLVisionSimulatorPlugin : public Plugin
{
public:
    GLVisionSimulatorPlugin();
    virtual bool initialize() override;
};

}


GLVisionSimulatorPlugin::GLVisionSimulatorPlugin()
    : Plugin("GLVisionSimulator")
{
    require("Body");
}


bool GLVisionSimulatorPlugin::initialize()
{
    GLVisionSimulatorItem::initializeClass(this);
    return true;
}


CNOID_IMPLEMENT_PLUGIN_ENTRY(GLVisionSimulatorPlugin)
