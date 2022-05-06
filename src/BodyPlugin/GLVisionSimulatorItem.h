/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_GL_VISION_SIMULATOR_ITEM_H
#define CNOID_BODY_PLUGIN_GL_VISION_SIMULATOR_ITEM_H

#include "SubSimulatorItem.h"
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
    void setAllSceneObjectsEnabled(bool on);
    void setHeadLightEnabled(bool on);
    void setAdditionalLightsEnabled(bool on);

    virtual bool initializeSimulation(SimulatorItem* simulatorItem);
    virtual void finalizeSimulation();

    [[deprecated("Use setThreadMode(SENSOR_THREAD_MODE)")]]
    void setDedicatedSensorThreadsEnabled(bool on);

    class Impl;

protected:
    virtual Item* doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

private:
    Impl* impl;
};

typedef ref_ptr<GLVisionSimulatorItem> GLVisionSimulatorItemPtr;

}

#endif
