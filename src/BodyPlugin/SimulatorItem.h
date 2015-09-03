/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_SIMULATOR_ITEM_H
#define CNOID_BODY_PLUGIN_SIMULATOR_ITEM_H

#include "CollisionSeq.h"
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
class SimulatedMotionEngineManager;
class SgCloneMap;

class CNOID_EXPORT SimulationBody : public Referenced
{
public:
    SimulationBody(BodyPtr body);
    virtual ~SimulationBody();

    BodyItem* bodyItem() const;
    Body* body() const;

    int numControllers() const;
    ControllerItem* controller(int index = 0) const;

    /**
       Call this in the initilization when the shapes are accessed after the initialization
    */
    void cloneShapesOnce();

    /**
       Called from the simulation loop thread
    */
    bool isActive() const;
    void setActive(bool on);

    /**
       Use this instead of Device::notiryStateChange when the state part which
       is not recoreded is changed
    */
    void notifyUnrecordedDeviceStateChange(Device* device);

    const std::string& resultItemPrefix() const;
    
    virtual void initializeResultBuffers();
    virtual void initializeResultItems();

    /**
       Called from the simulation loop thread.
    */
    virtual void bufferResults();
    virtual void flushResults();

private:
    SimulationBodyImpl* impl;
    friend class SimulatorItemImpl;
};
    
typedef ref_ptr<SimulationBody> SimulationBodyPtr;


class CNOID_EXPORT SimulatorItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);

    static SimulatorItem* findActiveSimulatorItemFor(Item* item);

    SimulatorItem();
    virtual ~SimulatorItem();

    virtual double worldTimeStep();
            
    bool startSimulation(bool doReset = true);
    void stopSimulation();
    void pauseSimulation();
    void restartSimulation();
    bool isRunning() const;
    bool isPausing() const;
    bool isActive() const; ///< isRunning() && !isPausing()
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
        
    /**
       For sub simulators
    */
    const std::vector<SimulationBody*>& simulationBodies();

    SimulationBody* findSimulationBody(BodyItem* bodyItem);
    SimulationBody* findSimulationBody(const std::string& name);

    /**
       \return The registration id of the function. The id can be used for removing the function.
    */
    int addPreDynamicsFunction(boost::function<void()> func);
    int addMidDynamicsFunction(boost::function<void()> func);
    int addPostDynamicsFunction(boost::function<void()> func);

    void removePreDynamicsFunction(int id);
    void removeMidDynamicsFunction(int id);
    void removePostDynamicsFunction(int id);
        
    //void addRecordFunction(boost::function<void()> func);

    SgCloneMap& sgCloneMap();

    /**
       \note This signal is emitted in the simulation thread
    */
    SignalProxy<void(const std::vector<SimulationBodyPtr>& simulationBodies)>
        sigSimulationBodyListUpdated();

    /*
    virtual void setExternalForce(BodyItem* bodyItem, Link* link, const Vector6& f);
    */

    /**
       @param point link local position to apply the force
       @param f linear force to apply in global coordinate
    */
    virtual void setExternalForce(BodyItem* bodyItem, Link* link, const Vector3& point, const Vector3& f, double time = 0.0);
    virtual void clearExternalForces();
    
    /**
       @param attachmentPoint link local position
       @param goal global goal position
    */
    virtual void setVirtualElasticString(
        BodyItem* bodyItem, Link* link, const Vector3& attachmentPoint, const Vector3& endPoint);
    virtual void clearVirtualElasticStrings();

    virtual void setForcedBodyPosition(BodyItem* bodyItem, const Position& T);
    virtual void clearForcedBodyPositions();

protected:
    SimulatorItem(const SimulatorItem& org);

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

    virtual CollisionLinkPairListPtr getCollisions()
    {
        return boost::make_shared<CollisionLinkPairList>();
    }

    void doPutProperties(PutPropertyFunction& putProperty);
    bool store(Archive& archive);
    bool restore(const Archive& archive);
            
private:
            
    SimulatorItemImpl* impl;
    friend class SimulatorItemImpl;
    friend class SimulatedMotionEngineManager;
};
        
typedef ref_ptr<SimulatorItem> SimulatorItemPtr;
}

#endif
