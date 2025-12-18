#include "GLLivoxMid360Simulator.h"
#include <cnoid/GLVisionSimulatorItem>
#include <cnoid/SceneCameras>
#include <cnoid/GLSceneRenderer>
#include <cnoid/MathUtil>
#include <cnoid/EigenUtil>
#include <cnoid/MessageOut>
#include <cnoid/Format>

using namespace std;
using namespace cnoid;

namespace {

constexpr int NumScreens = 4;
constexpr double ScreenYawRange = (2.0 * M_PI) / NumScreens;
constexpr double ScreenYawEnd = ScreenYawRange / 2.0;
constexpr double ScreenTanYawEnd = tan(ScreenYawEnd);

}


GLLivoxMid360Simulator::GLLivoxMid360Simulator(LivoxMid360* sensor)
    : GLVisionSensorSimulator(sensor),
      sensor(sensor)
{

}


bool GLLivoxMid360Simulator::doInitialize(GLVisionSimulatorItem* visionSimulatorItem)
{
    screenInfos.resize(NumScreens);
    
    for(int i=0; i < NumScreens; ++i){
        auto screen = addScreen();
        screen->setLightingEnabled(false);
    }

    if(sensor->yawRange() <= 0.0 || sensor->pitchRange() <= 0.0){
        return false;
    }
    if(sensor->angularPrecision() <= 0.0){
        return false;
    }
    if(sensor->sphericalAngleSeq().empty()){
        MessageOut::master()->putErrorln(
            formatC("Spherical angle data of {0} is not specified.", sensor->name()));
        return false;
    }

    double frameRate = std::max(0.1, std::min(sensor->scanRate(), visionSimulatorItem->maxFrameRate()));
    setCycleTime(1.0 / frameRate);
    if(visionSimulatorItem->isVisionDataRecordingEnabled()){
        sensor->setRangeDataStateClonable(true);
    }

    screenSyncCounter = 0;
    currentAngleSeqIndex = 0;
    
    return true;
}


bool GLLivoxMid360Simulator::doInitializeScreenCamera(GLVisionSensorRenderingScreen* screen)
{
    int screenIndex = screen->index();
    auto& info = screenInfos[screenIndex];
    
    auto screenCamera = new SgPerspectiveCamera;
    screenCamera->setNearClipDistance(sensor->minDistance());
    screenCamera->setFarClipDistance(sensor->maxDistance());

    const double screenPitchEnd = sensor->pitchRange() / 2.0;
    info.tanPitchEndAtYawEnd = tan(screenPitchEnd) / cos(ScreenYawEnd);

    info.resolutionX = ScreenYawRange / sensor->angularPrecision();
    info.pixelRatioX = info.resolutionX - 1;
    
    info.resolutionY = info.resolutionX * info.tanPitchEndAtYawEnd / ScreenTanYawEnd;
    info.pixelRatioY = info.resolutionY - 1;

    if(ScreenTanYawEnd > info.tanPitchEndAtYawEnd){
        screenCamera->setFieldOfView(2.0 * atan(info.tanPitchEndAtYawEnd));
    } else {
        screenCamera->setFieldOfView(ScreenYawRange);
    }

    double centerYaw = screenIndex * (M_PI / 2.0);
    AngleAxis yawRotation(centerYaw, Vector3::UnitY());
    const double pitchOffset = sensor->maxPitchAngle() - screenPitchEnd;
    AngleAxis pitchRotation(pitchOffset, Vector3::UnitX());
    info.R_camera = Matrix3(yawRotation * pitchRotation);
    
    screen->setScreenCamera(screenCamera, info.R_camera, info.resolutionX, info.resolutionY);

    return true;
}


void GLLivoxMid360Simulator::doStoreScreenImage(GLVisionSensorRenderingScreen* screen)
{
    screen->readDepthBuffer(screenInfos[screen->index()].depthBuf);

    // TODO: set the following values in initialization
    auto& info = screenInfos[screen->index()];
    Matrix4 P_inv = screen->renderer()->projectionMatrix().inverse();
    info.P_inv_32 = P_inv(3, 2);
    info.P_inv_33 = P_inv(3, 3);

    bool doStoreRangeData = false;
    {
        std::lock_guard<std::mutex> lock(screenSyncMutex);
        if(++screenSyncCounter == NumScreens){
            doStoreRangeData = true;
            screenSyncCounter = 0;
        }
    }

    if(doStoreRangeData){
        storeRangeData();
    }
}


void GLLivoxMid360Simulator::storeRangeData()
{
    const auto& angleSeq = sensor->sphericalAngleSeq();
    rangeData = std::make_shared<vector<double>>();
    rangeData->reserve(sensor->numSamples());
    angleData = std::make_shared<std::vector<Vector2f>>();
    angleData->reserve(sensor->numSamples());

    for(int i=0; i < sensor->numSamples(); ++i){
        if(currentAngleSeqIndex == angleSeq.size()){
            currentAngleSeqIndex = 0;
        }
        const Vector2f& angles = angleSeq[currentAngleSeqIndex++];
        double yaw = angles[0];
        double pitch = angles[1];
        if(auto distance = getDistance(yaw, pitch)){
            rangeData->push_back(*distance);
            angleData->emplace_back(yaw, pitch);
        }
    }
}


std::optional<double> GLLivoxMid360Simulator::getDistance(double yawAngle, double pitchAngle)
{
    double shiftedYaw = yawAngle + ScreenYawEnd;
    if(shiftedYaw < 0.0){
        shiftedYaw += 2.0 * M_PI;
    }
    int screenIndex = static_cast<int>(shiftedYaw / ScreenYawRange) % NumScreens;
    auto& info = screenInfos[screenIndex];

    Matrix3 R(AngleAxis(yawAngle, Vector3::UnitY()) * AngleAxis(pitchAngle, Vector3::UnitX()));
    Vector3 az = R.col(2);
    Vector3 az_local = info.R_camera.inverse() * az;
    double localYaw = atan2(az_local.x(), az_local.z());
    Vector3 az_pitch = AngleAxis(-localYaw, Vector3::UnitY()) * az_local;
    double localPitch = atan2(-az_pitch.y(), az_pitch.z());

    double rx = (ScreenTanYawEnd - tan(localYaw)) / (2.0 * ScreenTanYawEnd);
    int px = nearbyint(rx * info.pixelRatioX);
    px = std::min(px, info.resolutionX - 1);
    double ry = (tan(localPitch) / cos(localYaw) + info.tanPitchEndAtYawEnd) / (2.0 * info.tanPitchEndAtYawEnd);
    int py = nearbyint(ry * info.pixelRatioY);
    py = std::min(py, info.resolutionY - 1);

    float depth = info.depthBuf[py * info.resolutionX + px];
    if(depth <= 0.0f || depth >= 1.0f){
        return std::nullopt;
    } else {
        double z0 = 2.0 * depth - 1.0;
        double w = info.P_inv_32 * z0 + info.P_inv_33;
        double z = -1.0 / w;
        return fabs((z / cos(localPitch)) / cos(localYaw));
    }
}


void GLLivoxMid360Simulator::doClearVisionSensorData()
{
    sensor->clearRangeData();
}


void GLLivoxMid360Simulator::doUpdateVisionSensorData()
{
    sensor->setRangeData(rangeData);
    sensor->setSphericalAngleData(angleData);
}
