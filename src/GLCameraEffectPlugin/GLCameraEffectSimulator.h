#ifndef CNOID_GL_CAMERA_EFFECT_PLUGIN_GL_CAMERA_EFFECT_SIMULATOR_H
#define CNOID_GL_CAMERA_EFFECT_PLUGIN_GL_CAMERA_EFFECT_SIMULATOR_H

#include "ImageFilter.h"
#include <cnoid/Camera>
#include <cnoid/GLCameraSimulator>
#include <cnoid/GLVisionSimulatorItem>
#include <cnoid/ConnectionSet>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT GLCameraEffectSimulator : public GLCameraSimulator
{
public:
    GLCameraEffectSimulator(Camera* camera);
    virtual bool doInitialize(
        GLVisionSimulatorItem* visionSimulatorItem) override;
    virtual bool doInitializeScreenCamera(
        GLVisionSensorRenderingScreen* screen) override;
    virtual void doStoreScreenImage(
        GLVisionSensorRenderingScreen* screen) override;
    virtual void doClearVisionSensorData() override;
    virtual void doUpdateVisionSensorData() override;

protected:
    CameraPtr camera;
    std::shared_ptr<Image> image;
    ImageFilter filter;
    bool applyCameraEffect;
    ScopedConnection connection;
};

}  // namespace cnoid

#endif  // CNOID_GL_CAMERA_EFFECT_PLUGIN_GL_CAMERA_EFFECT_SIMULATOR_H
