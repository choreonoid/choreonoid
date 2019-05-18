/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_DYWORLD_H
#define CNOID_BODY_DYWORLD_H

#include "ForwardDynamics.h"
#include "ExtraJoint.h"
#include <cnoid/TimeMeasure>
#include <map>
#include "exportdecl.h"

namespace cnoid {

class DyLink;
class DyBody;
typedef ref_ptr<DyBody> DyBodyPtr;

#ifdef ENABLE_SIMULATION_PROFILING
const bool BODY_SIMULATION_PROFILING = true;
#else
const bool BODY_SIMULATION_PROFILING = false;
#endif

class CNOID_EXPORT WorldBase
{
public:
    WorldBase();
    virtual ~WorldBase();

    /**
       @brief get the number of bodies in this world
       @return the number of bodies
    */ 
    int numBodies() const { return bodyInfoArray.size(); }

    /**
       @brief get body by index
       @param index of the body
       @return body
    */
    DyBody* body(int index) const;

    /**
       @brief get body by name
       @param name of the body
       @return body
    */
    DyBody* body(const std::string& name) const;

    /**
       @brief get forward dynamics computation method for body
       @param index index of the body
       @return forward dynamics computation method
    */
    std::shared_ptr<ForwardDynamics> forwardDynamics(int index) {
        return bodyInfoArray[index].forwardDynamics;
    }

    /**
       @brief get index of body by name
       @param name of the body
       @return index of the body
    */
    int bodyIndex(const std::string& name) const;

    /**
       @brief add body to this world
       @param body
       @return index of the body
       @note This must be called before initialize() is called.
    */
    int addBody(DyBody* body);

    /**
       Use this method instead of addBody(const DyBodyPtr& body) when you want to specify
       a forward dynamics calculater.
    */
    int addBody(DyBody* body, std::shared_ptr<ForwardDynamics> forwardDynamics);
        
    /**
       @brief clear bodies in this world
    */
    void clearBodies();

    /**
       @brief clear collision pairs
    */
    void clearCollisionPairs();

    /**
       @brief set time step
       @param dt time step[s]
    */
    void setTimeStep(double dt);

    /**
       @brief get time step
       @return time step[s]
    */
    double timeStep(void) const { return timeStep_; }
	
    /**
       @brief set current time
       @param tm current time[s]
    */
    void setCurrentTime(double tm);

    /**
       @brief get current time
       @return current time[s]
    */
    double currentTime(void) const { return currentTime_; }
	
    /**
       @brief set gravity acceleration
       @param g gravity acceleration[m/s^2]
    */
    void setGravityAcceleration(const Vector3& g);

    /**
       @brief get gravity acceleration
       @return gravity accleration
    */
    inline const Vector3& gravityAcceleration() const { return g; }

        
    /**
       @brief enable/disable sensor simulation
       @param on true to enable, false to disable
       @note This must be called before initialize() is called.
    */
    void enableSensors(bool on);

    void setOldAccelSensorCalcMode(bool on);

    /**
       @brief choose euler method for integration
    */
    void setEulerMethod();

    /**
       @brief choose runge-kutta method for integration
    */
    void setRungeKuttaMethod();

    /**
       @brief initialize this world. This must be called after all bodies are registered.
    */
    virtual void initialize();

    void setVirtualJointForces();
        
    /**
       @brief compute forward dynamics and update current state
    */
    virtual void calcNextState();

    /**
       @brief get index of link pairs
       @param link1 link1
       @param link2 link2
       @return pair of index and flag. The flag is true if the pair was already registered, false othewise.
    */
    std::pair<int,bool> getIndexOfLinkPairs(DyLink* link1, DyLink* link2);

    std::vector<ExtraJoint> extrajoints;

protected:

    double currentTime_;
    double timeStep_;

    struct BodyInfo {
        DyBodyPtr body;
        std::shared_ptr<ForwardDynamics> forwardDynamics;
        bool hasVirtualJointForces;
    };
    std::vector<BodyInfo> bodyInfoArray;

    bool sensorsAreEnabled;
    bool isOldAccelSensorCalcMode;

private:
    typedef std::map<std::string, int> NameToIndexMap;
    NameToIndexMap nameToBodyIndexMap;

    typedef std::map<DyBodyPtr, int> BodyToIndexMap;
    BodyToIndexMap bodyToIndexMap;

    Vector3 g;

    bool isEulerMethod; // Euler or Runge Kutta ?

    struct LinkPairKey {
        DyLink* link1;
        DyLink* link2;
        bool operator<(const LinkPairKey& pair2) const;
    };
    typedef std::map<LinkPairKey, int> LinkPairKeyToIndexMap;
    LinkPairKeyToIndexMap linkPairKeyToIndexMap;

    int numRegisteredLinkPairs;
		
};

template <class TConstraintForceSolver> class World : public WorldBase
{
public:
    TConstraintForceSolver constraintForceSolver;

#ifdef ENABLE_SIMULATION_PROFILING
    double forceSolveTime;
    double forwardDynamicsTime;
    double customizerTime;
    TimeMeasure timer;
#endif

    World() : constraintForceSolver(*this) { }

    virtual void initialize() {
        WorldBase::initialize();
        constraintForceSolver.initialize();
    }

    virtual void calcNextState(){
#ifdef ENABLE_SIMULATION_PROFILING
            timer.begin();
#endif
        WorldBase::setVirtualJointForces();
#ifdef ENABLE_SIMULATION_PROFILING
        customizerTime = timer.measure();
        timer.begin();
#endif
        constraintForceSolver.solve();
#ifdef ENABLE_SIMULATION_PROFILING
        forceSolveTime = timer.measure();
        timer.begin();
#endif
        WorldBase::calcNextState();
#ifdef ENABLE_SIMULATION_PROFILING
        forwardDynamicsTime = timer.measure();
#endif
    }
};

};

#endif
