/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_SIMULATOR_ITEM_H
#define CNOID_BODY_PLUGIN_SIMULATOR_ITEM_H

#include <cnoid/Item>
#include "exportdecl.h"

namespace cnoid {

class Body;
typedef ref_ptr<Body> BodyPtr;
class Device;
class CollisionDetector;
typedef boost::shared_ptr<CollisionDetector> CollisionDetectorPtr;
class BodyItem;
class BodyMotionItem;
class ControllerItem;
class SimulationBodyImpl;
class SimulatorItemImpl;
class SgCloneMap;

class CNOID_EXPORT SimulationBody : public Referenced
{
public:
    SimulationBody(BodyPtr body);
    virtual ~SimulationBody();

    BodyItem* bodyItem() const;
    Body* body() const;
    ControllerItem* controller() const;

    /**
       Call this in the initilization when the shapes are accessed after the initialization
    */
    void cloneShapesOnce();

    /**
       Use this instead of Device::notiryStateChange when the state part which
       is not recoreded is changed
    */
    void notifyUnrecordedDeviceStateChange(Device* device);

    /**
       Called from the simulation loop thread.
    */
    virtual void storeResult();
    virtual bool flushResult();

private:
    SimulationBodyImpl* impl;
    friend class SimulatorItemImpl;
};
    
typedef ref_ptr<SimulationBody> SimulationBodyPtr;


class CNOID_EXPORT SimulatorItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);

    SimulatorItem();
    SimulatorItem(const SimulatorItem& org);
    virtual ~SimulatorItem();

    virtual double worldTimeStep();
            
    bool startSimulation(bool doReset = true);
    void stopSimulation();
    bool isRunning() const;
    int currentFrame() const;
    double currentTime() const;

    SignalProxy<void()> sigSimulationFinished();

    enum RecordingMode { RECORD_FULL, RECORD_TAIL, RECORD_NONE, N_RECORDING_MODES };
    enum TimeRangeMode { TIMEBAR_RANGE, SPECIFIED_PERIOD, UNLIMITED, N_TIME_RANGE_MODES };

    void setRecordingMode(int selection);
    Selection recordingMode() const;
    void setTimeRangeMode(int selection);
    void setRealtimeSyncMode(bool on);
    void setDeviceStateOutputEnabled(bool on);
    void setActiveControlPeriodOnlyMode(bool on);

    bool isRecordingEnabled() const;
    bool isDeviceStateOutputEnabled() const;
        
    bool isAllLinkPositionOutputMode();
    virtual void setAllLinkPositionOutputMode(bool on);
        
    virtual void selectMotionItems();

    /**
       For sub simulators
    */
    const std::vector<SimulationBody*>& simulationBodies();

    /**
       The following functions can be called from the initializeSimulation function of SubSimulatorItem.
    */
    void addPreDynamicsFunction(boost::function<void()> func);
    void addMidDynamicsFunction(boost::function<void()> func);
    void addPostDynamicsFunction(boost::function<void()> func);
        
    //void addRecordFunction(boost::function<void()> func);

    SgCloneMap& sgCloneMap();

    /**
       emitted from the simulation thread
    */
    SignalProxy<void(const std::vector<SimulationBodyPtr>& simulationBodies)>
        sigSimulationBodyListUpdated();

protected:

    virtual void onDisconnectedFromRoot();

    /**
       @note orgBody should not owned by the SimulationBody instance.
       Instead of it, a clone instance which may be a sub Body class should be created and owned.
    */
    virtual SimulationBodyPtr createSimulationBody(BodyPtr orgBody) = 0;

    virtual ControllerItem* createBodyMotionController(BodyItem* bodyItem, BodyMotionItem* bodyMotionItem);

    CollisionDetectorPtr collisionDetector();

    /**
       @param simBodies SimulatorBody objects which have a valid body
       @note This function is called from the main thread.
    */
    virtual bool initializeSimulation(const std::vector<SimulationBody*>& simBodies) = 0;

    virtual void initializeSimulationThread();

    /**
       This function is called from the simulation loop thread.
       @param activeSimBodies SimulatorBody objects which are non-static ones.
    */
    virtual bool stepSimulation(const std::vector<SimulationBody*>& activeSimBodies) = 0;

    /**
       \note This function is called from the main thread.
    */
    virtual void finalizeSimulation();

    void doPutProperties(PutPropertyFunction& putProperty);
    bool store(Archive& archive);
    bool restore(const Archive& archive);
            
private:
            
    SimulatorItemImpl* impl;
    friend class SimulatorItemImpl;
};
        
typedef ref_ptr<SimulatorItem> SimulatorItemPtr;
}

#endif
