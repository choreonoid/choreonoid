#include "GLRangeCameraEffectSimulator.h"
#include <cnoid/ValueTree>

using namespace std;
using namespace cnoid;

GLRangeCameraEffectSimulator::GLRangeCameraEffectSimulator(
    RangeCamera* rangeCamera)
    : GLRangeCameraSimulator(rangeCamera)
    , rangeCamera(rangeCamera)
    , applyCameraEffect(false)
{}


bool GLRangeCameraEffectSimulator::doInitialize(
    GLVisionSimulatorItem* visionSimulatorItem)
{
    // if(!GLRangeCameraSimulator::doInitialize(visionSimulatorItem)) {
    //     return false;
    // }

    connection.disconnect();
    applyCameraEffect = camera->info()->get("apply_camera_effect", false);
    if (applyCameraEffect) {
        filter.readCameraInfo(rangeCamera->info());
        connection = rangeCamera->sigInfoChanged().connect(
            [&](){ filter.readCameraInfo(rangeCamera->info()); });
    }

    double frameRate = std::max(0.1,
                                std::min(rangeCamera->frameRate(),
                                         visionSimulatorItem->maxFrameRate()));
    setCycleTime(1.0 / frameRate);
    if (visionSimulatorItem->isVisionDataRecordingEnabled()) {
        rangeCamera->setImageStateClonable(true);
    }

    return true;
}


bool GLRangeCameraEffectSimulator::doInitializeScreenCamera(
    GLVisionSensorRenderingScreen* screen)
{
    return screen->useCameraDeviceAsScreenCamera();
}


void GLRangeCameraEffectSimulator::doStoreScreenImage(
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


void GLRangeCameraEffectSimulator::doClearVisionSensorData()
{
    // GLRangeCameraSimulator::doClearVisionSensorData();
    rangeCamera->clearImage();
}


void GLRangeCameraEffectSimulator::doUpdateVisionSensorData()
{
    // GLRangeCameraSimulator::doUpdateVisionSensorData();
    rangeCamera->setImage(image);
}