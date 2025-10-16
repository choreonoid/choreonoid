#ifndef CNOID_GL_VISION_SENSOR_PLUGIN_GL_VISION_SENSOR_SIMULATOR_H
#define CNOID_GL_VISION_SENSOR_PLUGIN_GL_VISION_SENSOR_SIMULATOR_H

#include "GLVisionSensorRenderingScreen.h"
#include "GLVisionSensorScene.h"
#include <cnoid/VisionSensor>
#include <vector>
#include <typeinfo>
#include "exportdecl.h"

namespace cnoid {

class GLVisionSimulatorItem;
class SimulationBody;

class CNOID_EXPORT GLVisionSensorSimulator : public Referenced
{
public:
    template<class SensorType>
    static int registerSimulator(std::function<GLVisionSensorSimulator*(SensorType* sensor)> factory){
        return registerSimulator_(
            typeid(SensorType),
            [factory](VisionSensor* sensor){ return factory(static_cast<SensorType*>(sensor)); });
    }

    template<class SensorType>
    static void unregisterSimulator(int handle){
        unregisterSimulator_(typeid(SensorType), handle);
    }
    
    static GLVisionSensorSimulator* createSimulator(VisionSensor* sensor);
    
    ~GLVisionSensorSimulator();

    void setVisionSimulator(GLVisionSimulatorItem* visionSimulatorItem, SimulationBody* simBody, int bodyIndex);
    bool initialize(const std::vector<SimulationBody*>& simBodies);
    GLVisionSensorScenePtr createSensorScene(const std::vector<SimulationBody*>& simBodies);
    void startSharedRenderingThread();
    void moveRenderingBufferToMainThread();
    void startConcurrentRendering();
    void updateSensorScene();
    void render(GLVisionSensorRenderingScreen*& currentGLContextScreen, bool doDoneGLContextCurrent);
    void finalizeRendering();
    bool waitForRenderingToFinish();
    bool waitForRenderingToFinish(std::unique_lock<std::mutex>& lock);
    void clearVisionData();
    void copyVisionData();
    void finalize();

    virtual bool doInitializeScreenCamera(GLVisionSensorRenderingScreen* screen) = 0;
    virtual void doStoreScreenImage(GLVisionSensorRenderingScreen* screen) = 0;
    
    VisionSensor* visionSensor() { return visionSensor_; }
    GLVisionSimulatorItem* visionSimulatorItem() { return visionSimulatorItem_; }
    double elapsedTime() const { return elapsedTime_; }
    void initializeElapsedTime() { elapsedTime_ = cycleTime_; }
    void addToElapsedTime(double timeStep) { elapsedTime_ += timeStep; }
    double cycleTime() const { return cycleTime_; }
    double latency() const { return latency_; }
    void setOnsetTime(double time) { onsetTime_ = time; }
    GLVisionSensorScene* sharedScene() { return sharedScene_; }
    const std::vector<GLVisionSensorRenderingScreenPtr>& screens() const { return screens_; }
    bool wasDeviceOn() const { return wasDeviceOn_; }
    void wasDeviceOn(bool on) { wasDeviceOn_ = on; }
    bool isRendering() const { return isRendering_; }
    void isRendering(bool on) { isRendering_ = on; }
    bool needToClearVisionDataByTurningOff() { return needToClearVisionDataByTurningOff_; }
    void needToClearVisionDataByTurningOff(bool on) { needToClearVisionDataByTurningOff_ = on; }

protected:
    GLVisionSensorSimulator(VisionSensor* sensor);
    GLVisionSensorRenderingScreen* addScreen();
    int numScreens() const { return screens_.size(); }
    GLVisionSensorRenderingScreen* screen(int index) { return screens_[index]; }
    void setCycleTime(double t) { cycleTime_ = t; }

    virtual bool doInitialize(GLVisionSimulatorItem* visionSimulatorItem) = 0;
    virtual void doClearVisionSensorData() = 0;
    virtual void doUpdateVisionSensorData() = 0;

private:
    GLVisionSimulatorItem* visionSimulatorItem_;
    SimulationBody* simBody;
    int bodyIndex;
    VisionSensorPtr visionSensor_;
    double elapsedTime_;
    double cycleTime_;
    double latency_;
    double onsetTime_;
    GLVisionSensorScenePtr sharedScene_;
    std::vector<GLVisionSensorScenePtr> scenes;
    std::vector<GLVisionSensorRenderingScreenPtr> screens_;
    bool wasDeviceOn_;
    bool isRendering_;  // only updated and referred to in the simulation thread
    bool needToClearVisionDataByTurningOff_;
    bool isBestEffortMode;

    static int registerSimulator_(
        const std::type_info& sensorType, const std::function<GLVisionSensorSimulator*(VisionSensor* sensor)>& factory);
    static void unregisterSimulator_(const std::type_info& sensorType, int handle);
};

typedef ref_ptr<GLVisionSensorSimulator> GLVisionSensorSimulatorPtr;

}

#endif
