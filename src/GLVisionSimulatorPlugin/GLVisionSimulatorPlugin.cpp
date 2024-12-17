#include "GLVisionSimulatorItem.h"
#include "GLCameraSimulator.h"
#include "GLFisheyeCameraSimulator.h"
#include "GLRangeCameraSimulator.h"
#include "GLRangeSensorSimulator.h"
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

    GLVisionSensorSimulator::registerSimulator<Camera>(
        [](Camera* camera) -> GLVisionSensorSimulator* {
            auto lensType = camera->lensType();
            if(lensType == Camera::FISHEYE_LENS || lensType == Camera::DUAL_FISHEYE_LENS){
                return new GLFisheyeCameraSimulator(camera);
            } else {
                return new GLCameraSimulator(camera);
            }
        });

    GLVisionSensorSimulator::registerSimulator<RangeCamera>(
        [](RangeCamera* camera){ return new GLRangeCameraSimulator(camera); });

    GLVisionSensorSimulator::registerSimulator<RangeSensor>(
        [](RangeSensor* sensor){ return new GLRangeSensorSimulator(sensor); });

    return true;
}


CNOID_IMPLEMENT_PLUGIN_ENTRY(GLVisionSimulatorPlugin)
