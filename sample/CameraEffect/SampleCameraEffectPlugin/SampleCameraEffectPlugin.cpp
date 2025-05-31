#include <cnoid/Plugin>
#include "SampleCameraEffectSimulatorItem.h"

using namespace cnoid;

class SampleCameraEffectPlugin : public Plugin
{
public:
    SampleCameraEffectPlugin() : Plugin("SampleCameraEffect")
    {
        require("Body");
        require("GLVisionSimulator");
    }

    virtual bool initialize() override
    {
        SampleCameraEffectSimulatorItem::initializeClass(this);
        return true;
    }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(SampleCameraEffectPlugin)