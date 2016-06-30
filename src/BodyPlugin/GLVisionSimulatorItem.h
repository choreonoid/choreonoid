/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODYPLUGIN_GL_VISION_SIMULATOR_ITEM_H
#define CNOID_BODYPLUGIN_GL_VISION_SIMULATOR_ITEM_H

#include "SubSimulatorItem.h"
#include "exportdecl.h"

namespace cnoid {

class GLVisionSimulatorItemImpl;

class CNOID_EXPORT GLVisionSimulatorItem : public SubSimulatorItem
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    GLVisionSimulatorItem();
    GLVisionSimulatorItem(const GLVisionSimulatorItem& org);
    ~GLVisionSimulatorItem();
        
    void setTargetBodies(const std::string& bodyNames);
    void setTargetSensors(const std::string& sensorNames);
    void setMaxFrameRate(double rate);
    void setMaxLatency(double latency);
    void setVisionDataRecordingEnabled(bool on);
    void setDedicatedSensorThreadsEnabled(bool on);
    void setBestEffortMode(bool on);
    void setRangeSensorPrecisionRatio(double r);
    void setAllSceneObjectsEnabled(bool on);
    void setHeadLightEnabled(bool on);
    void setAdditionalLightsEnabled(bool on);

    virtual bool initializeSimulation(SimulatorItem* simulatorItem);
    virtual void finalizeSimulation();

protected:
    virtual Item* doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

private:
    GLVisionSimulatorItemImpl* impl;
};

typedef ref_ptr<GLVisionSimulatorItem> GLVisionSimulatorItemPtr;

}

#endif
