#ifndef CNOID_GL_CAMERA_EFFECT_PLUGIN_GL_RANGE_CAMERA_EFFECT_SIMULATOR_H
#define CNOID_GL_CAMERA_EFFECT_PLUGIN_GL_RANGE_CAMERA_EFFECT_SIMULATOR_H

#include "ImageFilter.h"
#include <cnoid/GLRangeCameraSimulator>
#include <cnoid/GLVisionSimulatorItem>
#include <cnoid/RangeCamera>
#include <cnoid/ConnectionSet>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT GLRangeCameraEffectSimulator : public GLRangeCameraSimulator
{
public:
    GLRangeCameraEffectSimulator(RangeCamera* rangeCamera);
    virtual bool doInitialize(
        GLVisionSimulatorItem* visionSimulatorItem) override;
    virtual bool doInitializeScreenCamera(
        GLVisionSensorRenderingScreen* screen) override;
    virtual void doStoreScreenImage(
        GLVisionSensorRenderingScreen* screen) override;
    virtual void doClearVisionSensorData() override;
    virtual void doUpdateVisionSensorData() override;

protected:
    RangeCameraPtr rangeCamera;
    std::shared_ptr<Image> image;
    ImageFilter filter;
    bool applyCameraEffect;
    ScopedConnection connection;
};

}  // namespace cnoid

#endif  // CNOID_GL_CAMERA_EFFECT_PLUGIN_GL_RANGE_CAMERA_EFFECT_SIMULATOR_H
