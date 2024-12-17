#ifndef CNOID_GL_VISION_SENSOR_PLUGIN_GL_RANGE_SENSOR_SIMULATOR_H
#define CNOID_GL_VISION_SENSOR_PLUGIN_GL_RANGE_SENSOR_SIMULATOR_H

#include "GLVisionSensorSimulator.h"
#include <cnoid/RangeSensor>
#include <random>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT GLRangeSensorSimulator : public GLVisionSensorSimulator
{
public:
    GLRangeSensorSimulator(RangeSensor* rangeSensor);
    virtual bool doInitialize(GLVisionSimulatorItem* visionSimulatorItem) override;
    virtual bool doInitializeScreenCamera(GLVisionSensorRenderingScreen* screen) override;
    virtual void doStoreScreenImage(GLVisionSensorRenderingScreen* screen) override;
    virtual void doClearVisionSensorData() override;
    virtual void doUpdateVisionSensorData() override;

private:
    RangeSensorPtr rangeSensor;

    double pitchRange;
    double pitchStep;
    int numPitchSamples;
    double yawStep;
    double detectionRate;
    double errorDeviation;

    struct ScreenInfo
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        Matrix3 R_camera;
        double yawRange;
        int numYawSamples;
        int numUniqueYawSamples;
        std::vector<float> depthBuf;
        std::shared_ptr<std::vector<double>> distances;
    };
    std::vector<ScreenInfo> screenInfos;
        
    std::shared_ptr<RangeSensor::RangeData> rangeData;
    
    double precisionRatio;
    double depthError;
    std::mt19937 randomNumber;
    std::uniform_real_distribution<> detectionProbability;
    std::normal_distribution<> distanceErrorDistribution;
};

}

#endif
