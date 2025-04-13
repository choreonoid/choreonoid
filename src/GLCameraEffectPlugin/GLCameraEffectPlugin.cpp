#include "GLCameraEffectSimulator.h"
#include "GLRangeCameraEffectSimulator.h"
#include <cnoid/Plugin>

using namespace cnoid;

namespace {

class GLCameraEffectPlugin : public Plugin
{
public:
    GLCameraEffectPlugin();
    virtual bool initialize() override;
};

}  // namespace


GLCameraEffectPlugin::GLCameraEffectPlugin()
    : Plugin("GLCameraEffect")
{
    require("Body");
    require("GLVisionSimulator");
}


bool GLCameraEffectPlugin::initialize()
{
    GLVisionSensorSimulator::registerSimulator<Camera>(
        [](Camera* camera) { return new GLCameraEffectSimulator(camera); });

    GLVisionSensorSimulator::registerSimulator<RangeCamera>(
        [](RangeCamera* camera) {
            return new GLRangeCameraEffectSimulator(camera);
        });

    return true;
}


CNOID_IMPLEMENT_PLUGIN_ENTRY(GLCameraEffectPlugin)