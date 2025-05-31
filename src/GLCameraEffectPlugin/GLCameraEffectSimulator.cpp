#include "GLCameraEffectSimulator.h"
#include <cnoid/ValueTree>

using namespace std;
using namespace cnoid;

GLCameraEffectSimulator::GLCameraEffectSimulator(Camera* camera)
    : GLCameraSimulator(camera)
    , camera(camera)
    , applyCameraEffect(false)
{}


bool GLCameraEffectSimulator::doInitialize(
    GLVisionSimulatorItem* visionSimulatorItem)
{
    // if(!GLCameraSimulator::doInitialize(visionSimulatorItem)) {
    //     return false;
    // }

    connection.disconnect();
    applyCameraEffect = camera->info()->get("apply_camera_effect", false);
    if (applyCameraEffect) {
        filter.readCameraInfo(camera->info());
        connection = camera->sigInfoChanged().connect(
            [&](){ filter.readCameraInfo(camera->info()); });
    }

    double frameRate = std::max(0.1,
                                std::min(camera->frameRate(),
                                         visionSimulatorItem->maxFrameRate()));
    setCycleTime(1.0 / frameRate);
    if (visionSimulatorItem->isVisionDataRecordingEnabled()) {
        camera->setImageStateClonable(true);
    }

    return true;
}


bool GLCameraEffectSimulator::doInitializeScreenCamera(
    GLVisionSensorRenderingScreen* screen)
{
    return screen->useCameraDeviceAsScreenCamera();
}


void GLCameraEffectSimulator::doStoreScreenImage(
    GLVisionSensorRenderingScreen* screen)
{
    if (!image) {
        image = std::make_shared<Image>();
    }
    screen->readImageBuffer(*image);
    if (applyCameraEffect) {
        filter.updateImage(image.get());
    }
}


void GLCameraEffectSimulator::doClearVisionSensorData()
{
    // GLCameraSimulator::doClearVisionSensorData();
    camera->clearImage();
}


void GLCameraEffectSimulator::doUpdateVisionSensorData()
{
    // GLCameraSimulator::doUpdateVisionSensorData();
    camera->setImage(image);
}