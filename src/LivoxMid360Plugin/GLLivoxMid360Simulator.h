#ifndef CNOID_GL_LIVOX_MID360_PLUGIN_LIVOX_MID360_SIMULATOR_H
#define CNOID_GL_LIVOX_MID360_PLUGIN_LIVOX_MID360_SIMULATOR_H

#include "LivoxMid360.h"
#include <cnoid/GLVisionSensorSimulator>
#include <optional>
#include <mutex>

namespace cnoid {

class GLLivoxMid360Simulator : public GLVisionSensorSimulator
{
public:
    GLLivoxMid360Simulator(LivoxMid360* sensor);

protected:
    virtual bool doInitialize(GLVisionSimulatorItem* visionSimulatorItem) override;
    virtual bool doInitializeScreenCamera(GLVisionSensorRenderingScreen* screen) override;
    virtual void doStoreScreenImage(GLVisionSensorRenderingScreen* screen) override;
    virtual void doClearVisionSensorData() override;
    virtual void doUpdateVisionSensorData() override;

private:
    LivoxMid360Ptr sensor;

    struct ScreenInfo {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Matrix3 R_camera;
        double tanPitchEndAtYawEnd;
        int resolutionX;
        int resolutionY;
        double pixelRatioX;
        double pixelRatioY;
        std::vector<float> depthBuf;
        double P_inv_32;
        double P_inv_33;
    };
    std::vector<ScreenInfo> screenInfos;
        
    std::mutex screenSyncMutex;
    int screenSyncCounter;
    int currentAngleSeqIndex;

    std::shared_ptr<RangeSensor::RangeData> rangeData;
    std::shared_ptr<std::vector<Vector2f>> angleData;

    void storeRangeData();
    std::optional<double> getDistance(double yawAngle, double pitchAngle);
};

}

#endif
