#include "GLFisheyeCameraSimulator.h"
#include "GLVisionSimulatorItem.h"
#include <cnoid/SceneCameras>
#include <cnoid/MathUtil>

using namespace std;
using namespace cnoid;

namespace {

enum ScreenId {
    NO_SCREEN = FisheyeLensConverter::NO_SCREEN,
    FRONT_SCREEN = FisheyeLensConverter::FRONT_SCREEN,
    LEFT_SCREEN = FisheyeLensConverter::LEFT_SCREEN,
    RIGHT_SCREEN = FisheyeLensConverter::RIGHT_SCREEN,
    TOP_SCREEN = FisheyeLensConverter::TOP_SCREEN,
    BOTTOM_SCREEN = FisheyeLensConverter::BOTTOM_SCREEN,
    BACK_SCREEN = FisheyeLensConverter::BACK_SCREEN
};

}


GLFisheyeCameraSimulator::GLFisheyeCameraSimulator(Camera* camera)
    : GLVisionSensorSimulator(camera),
      camera(camera)
{

}


bool GLFisheyeCameraSimulator::doInitialize(GLVisionSimulatorItem* visionSimulatorItem)
{
    auto lensType = camera->lensType();

    if(lensType != Camera::FISHEYE_LENS && lensType != Camera::DUAL_FISHEYE_LENS){
        return false;
    }

    int numScreens = 5;
    double fov = camera->fieldOfView();
    if(fov <= radian(90.0)){
        numScreens = 1;
    }
    int resolution = camera->resolutionX();
    int width = resolution;
    int height = resolution;
    if(camera->lensType()==Camera::DUAL_FISHEYE_LENS){
        numScreens = 6;
        resolution /= 2;
        height /=2;
        fov = radian(180.0);
    }
    if(fov > radian(180.0)){
        fov = radian(180.0);
    }
    resolution /= 2;   //screen resolution

    screenInfos.resize(numScreens);

    auto& R_front = screenInfos[FRONT_SCREEN].R;
    R_front = camera->R_local() * camera->opticalFrameRotation();
    if(numScreens >= 2){
        screenInfos[RIGHT_SCREEN].R = R_front * AngleAxis(radian(-90.0), Vector3::UnitY());
        screenInfos[LEFT_SCREEN].R = R_front * AngleAxis(radian(90.0),  Vector3::UnitY());
        screenInfos[TOP_SCREEN].R = R_front * AngleAxis(radian(90.0),  Vector3::UnitX());
        screenInfos[BOTTOM_SCREEN].R = R_front * AngleAxis(radian(-90.0), Vector3::UnitX());
        if(numScreens == 6){
            screenInfos[BACK_SCREEN].R = R_front * AngleAxis(radian(180.0), Vector3::UnitY());
        }
    }

    double frameRate = std::max(0.1, std::min(camera->frameRate(), visionSimulatorItem->maxFrameRate()));
    setCycleTime(1.0 / frameRate);
    if(visionSimulatorItem->isVisionDataRecordingEnabled()){
        camera->setImageStateClonable(true);
    }

    fisheyeLensConverter.initialize(width, height, fov, resolution);
    fisheyeLensConverter.setImageRotationEnabled(camera->lensType() == Camera::DUAL_FISHEYE_LENS);
    fisheyeLensConverter.setAntiAliasingEnabled(visionSimulatorItem->isAntiAliasingEnabled());

    for(int i=0; i < numScreens; ++i){
        auto& info = screenInfos[i];
        info.cameraForRendering = new Camera(*camera);
        info.cameraForRendering->setLocalRotation(info.R);
        info.cameraForRendering->setResolution(resolution, resolution);
        info.image = make_shared<Image>();
        fisheyeLensConverter.addScreenImage(info.image);
        addScreen();
    }

    return true;
}


void GLFisheyeCameraSimulator::doStoreScreenImage(GLVisionSensorRenderingScreen* screen)
{
    screen->readImageBuffer(*screenInfos[screen->index()].image);
}


bool GLFisheyeCameraSimulator::doInitializeScreenCamera(GLVisionSensorRenderingScreen* screen)
{
    auto screenCamera = new SgPerspectiveCamera;
    screenCamera->setNearClipDistance(camera->nearClipDistance());
    screenCamera->setFarClipDistance(camera->farClipDistance());
    screenCamera->setFieldOfView(radian(90.0));
    auto& cr = screenInfos[screen->index()].cameraForRendering;
    screen->setScreenCamera(screenCamera, cr->T_local(), cr->resolutionX(), cr->resolutionY());
    return true;
}


void GLFisheyeCameraSimulator::doClearVisionSensorData()
{
    camera->clearImage();    
}
    
    
void GLFisheyeCameraSimulator::doUpdateVisionSensorData()
{
    std::shared_ptr<Image> image = std::make_shared<Image>();
    fisheyeLensConverter.convertImage(image.get());
    camera->setImage(image);
}
