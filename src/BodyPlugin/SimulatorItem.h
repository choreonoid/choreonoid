/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_SIMULATOR_ITEM_H
#define CNOID_BODY_PLUGIN_SIMULATOR_ITEM_H

#include "CollisionSeq.h"
#include <cnoid/Item>
#include <cnoid/EigenTypes>
#include <vector>
#include <memory>
#include "exportdecl.h"

namespace cnoid {

class Body;
class Link;
class Device;
class CollisionDetector;
class WorldItem;
class BodyItem;
class ControllerItem;
class SimulatorItem;
class SimulatedMotionEngineManager;
class CloneMap;

class CNOID_EXPORT SimulationBody : public Referenced
{
public:
    SimulationBody(Body* body);
    virtual ~SimulationBody();

    BodyItem* bodyItem() const;
    Body* body() const;

    int numControllers() const;
    ControllerItem* controller(int index = 0) const;

    /**
       Call this in the initilization when the shapes are accessed after the initialization
    */
    void cloneShapesOnce();

    virtual bool initialize(SimulatorItem* simulatorItem, BodyItem* bodyItem);

    const std::string& recordItemPrefix() const;
    virtual void initializeRecordBuffers();
    virtual void initializeRecordItems();

    /**
       Called from the simulation loop thread
    */
    bool isActive() const;
    void setActive(bool on);

    /**
       Use this instead of Device::notifyStateChange when the state part which
       is not recoreded is changed
    */
    void notifyUnrecordedDeviceStateChange(Device* device);
    
    /**
       Called from the simulation loop thread.
    */
    virtual void bufferRecords();
    virtual void flushRecords();

    class Impl;

private:
    Impl* impl;

    friend class SimulatorItem;
};
    
typedef ref_ptr<SimulationBody> SimulationBodyPtr;


class CNOID_EXPORT SimulatorItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);

    static SimulatorItem* findActiveSimulatorItemFor(Item* item);

    virtual ~SimulatorItem();

    WorldItem* worldItem();
    virtual double worldTimeStep();
    void setTimeStep(double step);

    bool startSimulation(bool doReset = true);
    void stopSimulation(bool isForced = false);
    void pauseSimulation();
    void restartSimulation();
    bool isRunning() const;
    bool isPausing() const;
    bool isActive() const; ///< isRunning() && !isPausing()

    //! This can only be called from the simulation thread
    int currentFrame() const;
    
    //! This can only be called from the simulation thread
    double currentTime() const;

    //! This can be called from non simulation threads
    int simulationFrame() const;

    //! This can be called from non simulation threads
    double simulationTime() const;
    
    SignalProxy<void()> sigSimulationAboutToBeStarted();
    SignalProxy<void()> sigSimulationStarted();
    SignalProxy<void()> sigSimulationPaused();
    SignalProxy<void()> sigSimulationResumed();
    SignalProxy<void(bool isForced)> sigSimulationFinished();

    enum RecordingMode {
        FullRecording,
        TailRecording,
        NoRecording,
        NumRecordingModes,

        // Deprecated
        REC_FULL = FullRecording,
        REC_TAIL = TailRecording,
        REC_NONE = NoRecording,
        N_RECORDING_MODES = NumRecordingModes
    };
    
    void setRecordingMode(int selection);
    int recordingMode() const;

    enum TimeRangeMode {
        UnlimitedTime,
        SpecifiedTime,
        TimeBarTime,
        NumTimeRangeModes,

        // Deprecated. Use setActiveControlTimeRangeMode.
        ActiveControlTime, 

        // Deprecated
        TR_UNLIMITED = UnlimitedTime,
        TR_ACTIVE_CONTROL = ActiveControlTime,
        TR_SPECIFIED = SpecifiedTime,
        TR_TIMEBAR = TimeBarTime,
        N_TIME_RANGE_MODES = NumTimeRangeModes
    };
    
    void setTimeRangeMode(int selection);

    void setTimeLength(double length);

    [[deprecated("Use setTimeLength")]]
    void setSpecifiedRecordingTimeLength(double length){
        setTimeLength(length);
    }

    void setActiveControlTimeRangeMode(bool on);
    bool isActiveControlTimeRangeMode() const;

    void setRealtimeSyncMode(bool on);
    
    void setDeviceStateOutputEnabled(bool on);

    bool isRecordingEnabled() const;
    bool isDeviceStateOutputEnabled() const;

    bool isAllLinkPositionOutputMode();
    virtual void setAllLinkPositionOutputMode(bool on);

    const std::string& controllerOptionString() const;

    void setSceneViewEditModeBlockedDuringSimulation(bool on);    
    
    /**
       For sub simulators
    */
    const std::vector<SimulationBody*>& simulationBodies();

    SimulationBody* findSimulationBody(BodyItem* bodyItem);
    SimulationBody* findSimulationBody(const std::string& name);

    /**
       \return The registration id of the function. The id can be used for removing the function.
    */
    int addPreDynamicsFunction(std::function<void()> func);
    int addMidDynamicsFunction(std::function<void()> func);
    int addPostDynamicsFunction(std::function<void()> func);

    void removePreDynamicsFunction(int id);
    void removeMidDynamicsFunction(int id);
    void removePostDynamicsFunction(int id);
        
    //void addRecordFunction(std::function<void()> func);

    CloneMap& cloneMap();

    /**
       \note This signal is emitted in the simulation thread
    */
    SignalProxy<void(const std::vector<SimulationBodyPtr>& simulationBodies)>
        sigSimulationBodyListUpdated();

    /**
       \note This function should be a pure virtual function
    */
    virtual Vector3 getGravity() const;
    
    /**
       @param point link local position to apply the force
       @param f linear force to apply in global coordinate
    */
    virtual void setExternalForce(BodyItem* bodyItem, Link* link, const Vector3& point, const Vector3& f, double time = 0.0);
    virtual void clearExternalForces();
    
    /**
       @param attachmentPoint link local position
    */
    virtual void setVirtualElasticString(
        BodyItem* bodyItem, Link* link, const Vector3& attachmentPoint, const Vector3& endPoint);
    virtual void clearVirtualElasticStrings();

    virtual void setForcedPosition(BodyItem* bodyItem, const Isometry3& T);
    virtual bool isForcedPositionActiveFor(BodyItem* bodyItem) const;
    virtual void clearForcedPositions();

    class Impl;

protected:
    SimulatorItem();
    SimulatorItem(const std::string& name);
    SimulatorItem(const SimulatorItem& org);

    virtual void onTreePathChanged() override;
    virtual void onDisconnectedFromRoot() override;

    virtual void clearSimulation();

    /**
       @note orgBody should not owned by the SimulationBody instance.
       Instead it must clone a body instance using cloneMap and use it for the simulation.
       @note This function should be a pure virtual function when the old function is removed.
    */
    virtual SimulationBody* createSimulationBody(Body* orgBody, CloneMap& cloneMap);

    //! \deprecated. This is an old interface. Use createSimulationBody(Body* orgBody, CloneMap& cloneMap).
    virtual SimulationBody* createSimulationBody(Body* orgBody);

    CollisionDetector* getOrCreateCollisionDetector();

    //! \deprecated. Use getOrCreateCollisionDetector().
    CollisionDetector* collisionDetector();

    /**
       @param simBodies SimulatorBody objects which have a valid body
       @note This function is called from the main thread.
    */
    virtual bool initializeSimulation(const std::vector<SimulationBody*>& simBodies) = 0;

    /**
       This function is called after all the initializetion processes including those of
       controllers and sub simulators to complete the initialization of simulation.
       @note This function is called from the main thread.
    */
    virtual bool completeInitializationOfSimulation();

    virtual void initializeSimulationThread();

    virtual void finalizeSimulationThread();

    /**
       This function is called from the simulation loop thread.
       @param activeSimBodies SimulatorBody objects which are non-static ones.
    */
    virtual bool stepSimulation(const std::vector<SimulationBody*>& activeSimBodies) = 0;

    /**
       \note This function is called from the main thread.
    */
    virtual void finalizeSimulation();

    virtual std::shared_ptr<CollisionLinkPairList> getCollisions();

    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    Impl* impl;

    friend class SimulationBody;
    friend class SimulatedMotionEngineManager;
};
        
typedef ref_ptr<SimulatorItem> SimulatorItemPtr;

}

#endif
