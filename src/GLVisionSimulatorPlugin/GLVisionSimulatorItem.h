#ifndef CNOID_GL_VISION_SENSOR_PLUGIN_GL_VISION_SIMULATOR_ITEM_H
#define CNOID_GL_VISION_SENSOR_PLUGIN_GL_VISION_SIMULATOR_ITEM_H

#include <cnoid/SubSimulatorItem>
#include <cnoid/EigenTypes>
#include <condition_variable>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT GLVisionSimulatorItem : public SubSimulatorItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    GLVisionSimulatorItem();
    GLVisionSimulatorItem(const GLVisionSimulatorItem& org);
    ~GLVisionSimulatorItem();

    enum ThreadMode { SINGLE_THREAD_MODE, SENSOR_THREAD_MODE, SCREEN_THREAD_MODE, N_THREAD_MODES };

    void setTargetBodies(const std::string& bodyNames);
    void setTargetSensors(const std::string& sensorNames);
    void setMaxFrameRate(double rate);
    void setMaxLatency(double latency);
    void setVisionDataRecordingEnabled(bool on);
    void setThreadMode(int mode);
    void setBestEffortMode(bool on);
    void setRangeSensorPrecisionRatio(double r);
    void setBackgroundColor(const Vector3f& c);
    void setEveryRenderableItemEnabled(bool on);
    void setHeadLightEnabled(bool on);
    void setWorldLightEnabled(bool on);
    void setAdditionalLightSetEnabled(bool on);

    double maxFrameRate() const;
    double maxLatency() const;
    bool isVisionDataRecordingEnabled() const;
    bool isSensorThreadEnabled() const;
    bool isScreenThreadEnabled() const;
    bool isBestEffortMode() const;
    double rangeSensorPrecisionRatio() const;
    double depthError() const;
    const Vector3f& backgroundColor() const;
    bool isEveryRenderableItemEnabled() const;
    bool isHeadLightEnabled() const;
    bool isWorldLightEnabled() const;
    bool isAdditionalLightSetEnabled() const;
    bool isAntiAliasingEnabled() const;
    int msaaLevel() const;
    void setMsaaLevel(int level);

    CloneMap& cloneMap();
    double currentTime() const;
    std::condition_variable& queueCondition();

    virtual bool initializeSimulation(SimulatorItem* simulatorItem) override;
    virtual void finalizeSimulation() override;

    [[deprecated("Use setThreadMode(SENSOR_THREAD_MODE)")]]
    void setDedicatedSensorThreadsEnabled(bool on);
    [[deprecated("Use setAdditionalLightSetEnabled")]]
    void setAdditionalLightsEnabled(bool on);
    [[deprecated("Use setEveryRenderableItemEnabled")]]
    void setAllSceneObjectsEnabled(bool on);

    class Impl;

protected:
    virtual Item* doCloneItem(CloneMap* cloneMap) const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    Impl* impl;
};

typedef ref_ptr<GLVisionSimulatorItem> GLVisionSimulatorItemPtr;

}

#endif
