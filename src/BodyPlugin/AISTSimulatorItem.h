/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODYPLUGIN_AIST_SIMULATOR_ITEM_H
#define CNOID_BODYPLUGIN_AIST_SIMULATOR_ITEM_H

#include "SimulatorItem.h"
#include <cnoid/Collision>
#include "exportdecl.h"

namespace cnoid {

class ContactAttribute;
class AISTSimulatorItemImpl;
        
class CNOID_EXPORT AISTSimulatorItem : public SimulatorItem
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    AISTSimulatorItem();
    AISTSimulatorItem(const AISTSimulatorItem& org);
    virtual ~AISTSimulatorItem();

    virtual bool startSimulation(bool doReset = true);

    enum DynamicsMode { FORWARD_DYNAMICS = 0, HG_DYNAMICS, KINEMATICS, N_DYNAMICS_MODES };
    enum IntegrationMode { EULER_INTEGRATION = 0, RUNGE_KUTTA_INTEGRATION, N_INTEGRATION_MODES };

    void setDynamicsMode(int mode);
    void setIntegrationMode(int mode);
    void setGravity(const Vector3& gravity);
    const Vector3& gravity() const;
    void setFriction(double staticFriction, double slipFriction);
    void setContactCullingDistance(double value);        
    void setContactCullingDepth(double value);        
    void setErrorCriterion(double value);        
    void setMaxNumIterations(int value);
    void setContactCorrectionDepth(double value);
    void setContactCorrectionVelocityRatio(double value);
    void setEpsilon(double epsilon);
    void set2Dmode(bool on);
    void setKinematicWalkingEnabled(bool on);
    void setConstraintForceOutputEnabled(bool on);

    virtual void setForcedPosition(BodyItem* bodyItem, const Position& T);
    virtual bool isForcedPositionActiveFor(BodyItem* bodyItem) const;
    virtual void clearForcedPositions();

    // experimental functions
    void setFriction(Link* link1, Link* link2, double staticFriction, double slipFriction);

    typedef boost::function<bool(Link* link1, Link* link2, const CollisionArray& collisions, const ContactAttribute& attribute)>
        CollisionHandler;
    int registerCollisionHandler(const std::string& name, CollisionHandler handler);
    void unregisterCollisionHandler(int handlerId);
    int collisionHandlerId(const std::string& name) const;
    void setCollisionHandler(Link* link1, Link* link2, int handlerId);

protected:
    virtual SimulationBody* createSimulationBody(Body* orgBody);
    virtual bool initializeSimulation(const std::vector<SimulationBody*>& simBodies);
    virtual bool stepSimulation(const std::vector<SimulationBody*>& activeSimBodies);
    virtual void finalizeSimulation();
    virtual CollisionLinkPairListPtr getCollisions();
        
    virtual Item* doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);
#ifdef ENABLE_SIMULATION_PROFILING
    virtual void getProfilingNames(std::vector<std::string>& profilingNames);
    virtual void getProfilingTimes(std::vector<double>& profilingTimes);
#endif

private:
    AISTSimulatorItemImpl* impl;
    friend class AISTSimulatorItemImpl;
};

typedef ref_ptr<AISTSimulatorItem> AISTSimulatorItemPtr;
}

#endif
