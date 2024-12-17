#ifndef CNOID_GL_VISION_SENSOR_PLUGIN_GL_CAMERA_SIMULATOR_H
#define CNOID_GL_VISION_SENSOR_PLUGIN_GL_CAMERA_SIMULATOR_H

#include "GLVisionSensorSimulator.h"
#include <cnoid/Camera>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT GLCameraSimulator : public GLVisionSensorSimulator
{
public:
    GLCameraSimulator(Camera* camera);
    virtual bool doInitialize(GLVisionSimulatorItem* visionSimulatorItem) override;
    virtual bool doInitializeScreenCamera(GLVisionSensorRenderingScreen* screen) override;
    virtual void doStoreScreenImage(GLVisionSensorRenderingScreen* screen) override;
    virtual void doClearVisionSensorData() override;
    virtual void doUpdateVisionSensorData() override;

protected:
    CameraPtr camera;
    std::shared_ptr<Image> image;
};

}

#endif
