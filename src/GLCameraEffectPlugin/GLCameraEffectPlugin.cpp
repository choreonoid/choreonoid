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
    virtual bool finalize() override;

private:
    int cameraHandle;
    int rangeCameraHandle;
};

}  // namespace


GLCameraEffectPlugin::GLCameraEffectPlugin()
    : Plugin("GLCameraEffect"),
      cameraHandle(-1),
      rangeCameraHandle(-1)
{
    require("Body");
    require("GLVisionSimulator");
}


bool GLCameraEffectPlugin::initialize()
{
    cameraHandle = GLVisionSensorSimulator::registerSimulator<Camera>(
        [](Camera* camera) -> GLVisionSensorSimulator* {
            if(!camera->info()->get("apply_camera_effect", false)) {
                return nullptr;
            }
            return new GLCameraEffectSimulator(camera);
        });

    rangeCameraHandle = GLVisionSensorSimulator::registerSimulator<RangeCamera>(
        [](RangeCamera* camera) -> GLVisionSensorSimulator* {
            if(!camera->info()->get("apply_camera_effect", false)) {
                return nullptr;
            }
            return new GLRangeCameraEffectSimulator(camera);
        });

    return true;
}


bool GLCameraEffectPlugin::finalize()
{
    GLVisionSensorSimulator::unregisterSimulator<Camera>(cameraHandle);
    GLVisionSensorSimulator::unregisterSimulator<RangeCamera>(rangeCameraHandle);
    return true;
}


CNOID_IMPLEMENT_PLUGIN_ENTRY(GLCameraEffectPlugin)