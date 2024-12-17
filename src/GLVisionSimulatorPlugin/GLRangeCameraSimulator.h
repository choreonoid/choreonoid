#ifndef CNOID_GL_VISION_SENSOR_PLUGIN_GL_RANGE_CAMERA_SIMULATOR_H
#define CNOID_GL_VISION_SENSOR_PLUGIN_GL_RANGE_CAMERA_SIMULATOR_H

#include "GLCameraSimulator.h"
#include <cnoid/RangeCamera>
#include <random>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT GLRangeCameraSimulator : public GLCameraSimulator
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    GLRangeCameraSimulator(RangeCamera* rangeCamera);
    virtual bool doInitialize(GLVisionSimulatorItem* visionSimulatorItem) override;
    virtual bool doInitializeScreenCamera(GLVisionSensorRenderingScreen* screen) override;
    virtual void doStoreScreenImage(GLVisionSensorRenderingScreen* screen) override;
    virtual void doClearVisionSensorData() override;
    virtual void doUpdateVisionSensorData() override;

private:
    RangeCameraPtr rangeCamera;
    Camera::ImageType imageType;
    Matrix3f Ro;
    bool hasRo;
    double detectionRate;
    double errorDeviation;
    bool isOrganized;
    bool isDense;
    std::vector<unsigned char> colorBuf;
    std::vector<float> depthBuf;
    std::shared_ptr<RangeCamera::PointData> points;
    double depthError;
    std::mt19937 randomNumber;
    std::uniform_real_distribution<> detectionProbability;
    std::normal_distribution<> distanceErrorDistribution;
};

}

#endif
