#ifndef CNOID_SAMPLE_CAMERA_EFFECT_PLUGIN_SAMPLE_CAMERA_EFFECT_SIMULATOR_TEIM_H
#define CNOID_SAMPLE_CAMERA_EFFECT_PLUGIN_SAMPLE_CAMERA_EFFECT_SIMULATOR_TEIM_H

#include <cnoid/SubSimulatorItem>
#include <cnoid/DeviceList>
#include <cnoid/Camera>

namespace cnoid {

class ExtensionManager;
class Camera;

class SampleCameraEffectSimulatorItem : public SubSimulatorItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    SampleCameraEffectSimulatorItem();
    SampleCameraEffectSimulatorItem(const SampleCameraEffectSimulatorItem& org);
    ~SampleCameraEffectSimulatorItem();

    virtual bool initializeSimulation(SimulatorItem* simulatorItem) override;

protected:
    virtual Item* doDuplicate() const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    DeviceList<Camera> cameras;

    void onPreDynamicsFunction();
};

}

#endif