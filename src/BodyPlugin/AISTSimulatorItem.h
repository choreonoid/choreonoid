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

class AISTSimulatorItemImpl;
class ContactMaterial;
        
class CNOID_EXPORT AISTSimulatorItem : public SimulatorItem
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    AISTSimulatorItem();
    AISTSimulatorItem(const AISTSimulatorItem& org);
    virtual ~AISTSimulatorItem();

    virtual bool startSimulation(bool doReset = true) override;

    enum DynamicsMode { FORWARD_DYNAMICS = 0, KINEMATICS, N_DYNAMICS_MODES };
    enum IntegrationMode { EULER_INTEGRATION = 0, RUNGE_KUTTA_INTEGRATION, N_INTEGRATION_MODES };

    void setDynamicsMode(int mode);
    void setIntegrationMode(int mode);
    void setGravity(const Vector3& gravity);
    const Vector3& gravity() const;
    void setFriction(double staticFriction, double dynamicFriction);
    double staticFriction() const;
    double dynamicFriction() const;
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

    void addExtraJoint(ExtraJoint& extrajoint);
    void clearExtraJoint();

    virtual Vector3 getGravity() const override;
    virtual void setForcedPosition(BodyItem* bodyItem, const Position& T) override;
    virtual bool isForcedPositionActiveFor(BodyItem* bodyItem) const override;
    virtual void clearForcedPositions() override;

    typedef std::function<bool(Link* link1, Link* link2,
                               const CollisionArray& collisions,
                               ContactMaterial* contactMaterial)> CollisionHandler;
    
    void registerCollisionHandler(const std::string& name, CollisionHandler handler);
    bool unregisterCollisionHandler(const std::string& name);

    //! \deprecated
    void setFriction(Link* link1, Link* link2, double staticFriction, double dynamicFriction);

protected:
    virtual SimulationBody* createSimulationBody(Body* orgBody, CloneMap& cloneMap) override;
    virtual bool initializeSimulation(const std::vector<SimulationBody*>& simBodies) override;
    virtual bool stepSimulation(const std::vector<SimulationBody*>& activeSimBodies) override;
    virtual void finalizeSimulation() override;
    virtual std::shared_ptr<CollisionLinkPairList> getCollisions() override;
        
    virtual Item* doDuplicate() const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;
#ifdef ENABLE_SIMULATION_PROFILING
    virtual void getProfilingNames(std::vector<std::string>& profilingNames) override;
    virtual void getProfilingTimes(std::vector<double>& profilingTimes) override;
#endif

private:
    AISTSimulatorItemImpl* impl;
    friend class AISTSimulatorItemImpl;
};

typedef ref_ptr<AISTSimulatorItem> AISTSimulatorItemPtr;

}

#endif
