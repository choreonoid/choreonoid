#include "GLRangeSensorSimulator.h"
#include "GLVisionSimulatorItem.h"
#include <cnoid/GLSceneRenderer>
#include <cnoid/SceneCameras>
#include <cnoid/MathUtil>

using namespace std;
using namespace cnoid;


GLRangeSensorSimulator::GLRangeSensorSimulator(RangeSensor* rangeSensor)
    : GLVisionSensorSimulator(rangeSensor),
      rangeSensor(rangeSensor)
{
    numPitchSamples = 0;
}


bool GLRangeSensorSimulator::doInitialize(GLVisionSimulatorItem* visionSimulatorItem)
{
    int numScreens = 1;
    double yawRangePerScreen = rangeSensor->yawRange();
    if(yawRangePerScreen > radian(120.0)){
        numScreens = static_cast<int>(yawRangePerScreen / radian(91.0)) + 1;
        yawRangePerScreen = yawRangePerScreen / numScreens;
    }
    RangeSensorPtr tmpSensor = new RangeSensor(*rangeSensor);
    double yawRangeOffset = 0;
    double yawOffset = -rangeSensor->yawRange() / 2.0;
    pitchRange = std::min(rangeSensor->pitchRange(), radian(170.0));
    tmpSensor->setPitchRange(pitchRange);
    pitchStep = tmpSensor->pitchStep();
    numPitchSamples = tmpSensor->numPitchSamples();
    yawStep = rangeSensor->yawStep();
    detectionRate = rangeSensor->detectionRate();
    errorDeviation = rangeSensor->errorDeviation();
    screenInfos.resize(numScreens);
    
    for(int i=0; i < numScreens; ++i){
        auto& screenInfo = screenInfos[i];
        auto screen = addScreen();
        screen->setLightingEnabled(false);

        // Adjust to be a multiple of yawStep
        int n = 1;
        if(yawStep > 0.0){
            n = static_cast<int>(round((yawRangePerScreen + yawRangeOffset) / yawStep)) + 1;
        }
        double adjustedYawRange = (n - 1) * yawStep;
        screenInfo.yawRange = adjustedYawRange;
        yawRangeOffset = yawRangePerScreen - adjustedYawRange;
        screenInfo.numYawSamples = n;
        if(i < numScreens - 1){
            screenInfo.numUniqueYawSamples = n - 1;
        } else {
            screenInfo.numUniqueYawSamples = n; // Last screen
        }

        double centerAngle = yawOffset + adjustedYawRange / 2.0;
        screenInfo.R_camera = AngleAxis(centerAngle, Vector3::UnitY());

        yawOffset += adjustedYawRange;
    }

    double frameRate = std::max(0.1, std::min(rangeSensor->scanRate(), visionSimulatorItem->maxFrameRate()));
    setCycleTime(1.0 / frameRate);
    if(visionSimulatorItem->isVisionDataRecordingEnabled()){
        rangeSensor->setRangeDataStateClonable(true);
    }

    precisionRatio = visionSimulatorItem->rangeSensorPrecisionRatio();

    if(errorDeviation > 0.0){
        distanceErrorDistribution.param(
            std::normal_distribution<>::param_type(0.0, errorDeviation));
    }
    depthError = visionSimulatorItem->depthError();
    randomNumber.seed(0);
    detectionProbability.reset();
    distanceErrorDistribution.reset();
    
    return true;
}


bool GLRangeSensorSimulator::doInitializeScreenCamera(GLVisionSensorRenderingScreen* screen)
{
    // Enable depth buffer update for range data acquisition
    screen->setDepthBufferUpdateEnabled(true);

    auto& screenInfo = screenInfos[screen->index()];
    auto screenCamera = new SgPerspectiveCamera;
    screenCamera->setNearClipDistance(rangeSensor->minDistance());
    screenCamera->setFarClipDistance(rangeSensor->maxDistance());

    const double halfYawRange = screenInfo.yawRange / 2.0;
    const double halfPitchRange = pitchRange / 2.0;
    const double maxTanPitch = tan(halfPitchRange) / cos(halfYawRange);
    const double maxTanYaw = tan(halfYawRange);
    const double maxPitchRange2 = atan(maxTanPitch);

    int resolutionX;
    int resolutionY;

    if(screenInfo.numYawSamples >= numPitchSamples){
        resolutionX = screenInfo.numYawSamples * precisionRatio;
        resolutionY = resolutionX * maxTanPitch / maxTanYaw;
    } else {
        resolutionY = numPitchSamples * precisionRatio;
        resolutionX = resolutionY * maxTanYaw / maxTanPitch;
        double r = screenInfo.numYawSamples * precisionRatio;
        if(halfYawRange != 0.0 && resolutionX < r){
            resolutionX = r;
            resolutionY = resolutionX * maxTanPitch / maxTanYaw;
        }
    }

    if(maxTanYaw > maxTanPitch){
        if(halfPitchRange == 0.0){
            resolutionY = 1;
            double r = tan(halfYawRange) * 2.0 / resolutionX;
            screenCamera->setFieldOfView(atan2(r / 2.0, 1.0) * 2.0);
        } else {
            screenCamera->setFieldOfView(maxPitchRange2 * 2.0);
        }
    } else {
        if(halfYawRange == 0.0){
            resolutionX = 1;
            double r = tan(halfPitchRange) * 2.0 / resolutionY;
            screenCamera->setFieldOfView(atan2(r / 2.0, 1.0) * 2.0);
        } else {
            screenCamera->setFieldOfView(halfYawRange * 2.0);
        }
    }

    screen->setScreenCamera(screenCamera, screenInfo.R_camera, resolutionX, resolutionY);

    return true;
}


void GLRangeSensorSimulator::doStoreScreenImage(GLVisionSensorRenderingScreen* screen)
{
    auto& screenInfo = screenInfos[screen->index()];
    
    const double yawRange = screenInfo.yawRange;
    const double maxTanYawAngle = tan(yawRange / 2.0);
    const double maxTanPitchAngle = tan(pitchRange / 2.0) / cos(yawRange / 2.0);

    const Matrix4 Pinv = screen->renderer()->projectionMatrix().inverse();
    const double Pinv_32 = Pinv(3, 2);
    const double Pinv_33 = Pinv(3, 3);
    const int resolutionX = screen->resolutionX();
    const int resolutionY = screen->resolutionY();
    const double fw = resolutionX;
    const double fh = resolutionY;

    auto& depthBuf = screenInfo.depthBuf;
    screen->readDepthBuffer(depthBuf);
    auto& distances = screenInfo.distances;
    if(!distances){
        distances = make_shared<std::vector<double>>();
    } else {
        distances->clear();
    }
    distances->reserve(screenInfo.numUniqueYawSamples * numPitchSamples);

    for(int pitch = 0; pitch < numPitchSamples; ++pitch){
        const double pitchAngle = pitch * pitchStep - pitchRange / 2.0;
        const double cosPitchAngle = cos(pitchAngle);

        for(int yaw = 0; yaw < screenInfo.numUniqueYawSamples; ++yaw){

            if(detectionRate < 1.0){
                if(detectionProbability(randomNumber) > detectionRate){
                    distances->push_back(std::numeric_limits<double>::infinity());
                    continue;
                }
            }
            
            const double yawAngle = yaw * yawStep - yawRange / 2.0;

            int py;
            if(pitchRange == 0.0){
                py = 0;
            } else {
                const double r = (tan(pitchAngle) / cos(yawAngle) + maxTanPitchAngle) / (maxTanPitchAngle * 2.0);
                py = nearbyint(r * (fh - 1.0));
            }
            const int srcpos = py * resolutionX;

            int px;
            if(yawRange == 0.0){
                px = 0;
            } else {
                const double r = (maxTanYawAngle - tan(yawAngle)) / (maxTanYawAngle * 2.0);
                px = nearbyint(r * (fw - 1.0));
            }
            //! \todo add the option to do the interpolation between the adjacent two pixel depths
            const float depth = depthBuf[srcpos + px];
            if(depth <= 0.0f || depth >= 1.0f){
                distances->push_back(std::numeric_limits<double>::infinity());
            } else {                
                const double z0 = 2.0 * depth - 1.0;
                const double w = Pinv_32 * z0 + Pinv_33;
                const double z = -1.0 / w + depthError;
                double distance = fabs((z / cosPitchAngle) / cos(yawAngle));

                if(errorDeviation > 0.0){
                    distance += distanceErrorDistribution(randomNumber);
                }
                
                distances->push_back(distance);
            }
        }
    }
}


void GLRangeSensorSimulator::doClearVisionSensorData()
{
    rangeSensor->clearRangeData();
}


void GLRangeSensorSimulator::doUpdateVisionSensorData()
{
    int numScreens = screenInfos.size();

    if(numScreens == 0){
        rangeData->clear();
        
    } else if(numScreens == 1){
        rangeData = screenInfos[0].distances;

    } else {
        vector<double>::iterator src[4];
        int size = 0;
        for(int i=0; i < numScreens; ++i){
            auto& distances = screenInfos[i].distances;
            size += distances->size();
            src[i] = distances->begin();
        }
        rangeData = std::make_shared<vector<double>>(size);

        auto dest = rangeData->begin();

        int numPitchSamples = rangeSensor->numPitchSamples();
        for(int i=0; i < numPitchSamples; ++i){
            for(size_t j=0; j < numScreens; ++j){
                int n = screenInfos[j].numUniqueYawSamples;
                copy(src[j], src[j] + n, dest);
                advance(src[j], n);
                advance(dest, n);
            }
        }
    }

    rangeSensor->setRangeData(rangeData);
}
