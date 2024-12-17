#include "GLCameraSimulator.h"
#include "GLVisionSimulatorItem.h"

using namespace std;
using namespace cnoid;


GLCameraSimulator::GLCameraSimulator(Camera* camera)
    : GLVisionSensorSimulator(camera),
      camera(camera)
{

}


bool GLCameraSimulator::doInitialize(GLVisionSimulatorItem* visionSimulatorItem)
{
    if(camera->lensType() != Camera::NORMAL_LENS){
        return false;
    }
    if(camera->imageType() != Camera::COLOR_IMAGE){
        return false;
    }
    
    double frameRate = std::max(0.1, std::min(camera->frameRate(), visionSimulatorItem->maxFrameRate()));
    setCycleTime(1.0 / frameRate);
    if(visionSimulatorItem->isVisionDataRecordingEnabled()){
        camera->setImageStateClonable(true);
    }

    return true;
}


bool GLCameraSimulator::doInitializeScreenCamera(GLVisionSensorRenderingScreen* screen)
{
    return screen->useCameraDeviceAsScreenCamera();
}

        
void GLCameraSimulator::doStoreScreenImage(GLVisionSensorRenderingScreen* screen)
{
    if(!image){
        image = std::make_shared<Image>();
    }
    screen->readImageBuffer(*image);
}


void GLCameraSimulator::doClearVisionSensorData()
{
    camera->clearImage();
}


void GLCameraSimulator::doUpdateVisionSensorData()
{
    camera->setImage(image);
}
