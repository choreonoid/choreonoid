#ifndef CNOID_GL_VISION_SENSOR_PLUGIN_GL_FISHEYE_CAMERA_SIMULATOR_H
#define CNOID_GL_VISION_SENSOR_PLUGIN_GL_FISHEYE_CAMERA_SIMULATOR_H

#include "GLVisionSensorSimulator.h"
#include "FisheyeLensConverter.h"
#include <cnoid/Camera>

namespace cnoid {

class GLFisheyeCameraSimulator : public GLVisionSensorSimulator
{
public:
    GLFisheyeCameraSimulator(Camera* camera);
    virtual bool doInitialize(GLVisionSimulatorItem* visionSimulatorItem) override;
    virtual bool doInitializeScreenCamera(GLVisionSensorRenderingScreen* screen) override;
    virtual void doStoreScreenImage(GLVisionSensorRenderingScreen* screen) override;
    virtual void doClearVisionSensorData() override;
    virtual void doUpdateVisionSensorData() override;

private:
    CameraPtr camera;
    FisheyeLensConverter fisheyeLensConverter;

    struct ScreenInfo
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        CameraPtr cameraForRendering;
        std::shared_ptr<Image> image;
        Matrix3 R;
    };

    std::vector<ScreenInfo> screenInfos;
};

}

#endif
