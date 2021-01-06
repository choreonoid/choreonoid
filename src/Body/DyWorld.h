/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_DYWORLD_H
#define CNOID_BODY_DYWORLD_H

#include "DyBody.h"
#include "ExtraJoint.h"
#include <string>
#include <map>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT DyWorldBase
{
public:
    DyWorldBase();
    virtual ~DyWorldBase();

    int numBodies() const { return bodies_.size(); }
    DyBody* body(int index) { return bodies_[index]; }
    DyBody* body(const std::string& name) const;
    const std::vector<DyBodyPtr>& bodies() { return bodies_; }

    /**
       \brief This function returns the number of bodies used in the internal calculation in this world
       \note The body that has non-root free-joints are decomposed into internal sub bodies
    */ 
    int numSubBodies() const { return subBodies_.size(); }
    DySubBody* subBody(int index) { return subBodies_[index]; }
    const std::vector<DySubBodyPtr>& subBodies() { return subBodies_; }

    /**
       \brief This function adds a body to the world
       \return Index of the body in the world
       \note This must be called before initialize() is called.
    */
    int addBody(DyBody* body);

    bool hasHighGainDynamics() const { return hasHighGainDynamics_; }
    void clearBodies();
    void clearCollisionPairs();
    void setTimeStep(double dt);
    double timeStep(void) const { return timeStep_; }
    void setCurrentTime(double tm);
    double currentTime(void) const { return currentTime_; }
	
    /**
       \param g gravity acceleration[m/s^2]
    */
    void setGravityAcceleration(const Vector3& g);

    inline const Vector3& gravityAcceleration() const { return g; }

    /**
       \brief enable/disable sensor simulation
       \param on true to enable, false to disable
       \note This must be called before initialize() is called.
    */
    void enableSensors(bool on);

    void setOldAccelSensorCalcMode(bool on);

    /**
       \brief Use the euler method for integration
    */
    void setEulerMethod();

    /**
       \brief Use the runge-kutta method for integration
    */
    void setRungeKuttaMethod();

    /**
       \brief initialize this world. This must be called after all bodies are registered.
    */
    virtual void initialize();

    void setVirtualJointForces();
        
    /**
       \brief compute forward dynamics and update current state
    */
    virtual void calcNextState();

    void refreshState();
        
    /**
       \brief get index of link pairs
       \param link1 link1
       \param link2 link2
       \return pair of index and flag. The flag is true if the pair was already registered, false othewise.
    */
    std::pair<int,bool> getIndexOfLinkPairs(DyLink* link1, DyLink* link2);

    std::vector<ExtraJoint>& extraJoints() { return extraJoints_; }
    void clearExtraJoints() { extraJoints_.clear(); }
    void addExtraJoint(ExtraJoint& extraJoint){ extraJoints_.push_back(extraJoint); }

private:
    double currentTime_;
    double timeStep_;

    std::vector<DyBodyPtr> bodies_;
    std::vector<DySubBodyPtr> subBodies_;
    std::vector<DyBodyPtr> bodiesWithVirtualJointForces_;
    std::map<std::string, DyBodyPtr> nameToBodyMap;

    Vector3 g;
    bool sensorsAreEnabled;
    bool isOldAccelSensorCalcMode;
    bool isEulerMethod; // Euler or Runge Kutta ?
    bool hasHighGainDynamics_;

    struct LinkPairKey {
        DyLink* link1;
        DyLink* link2;
        bool operator<(const LinkPairKey& pair2) const;
    };
    std::map<LinkPairKey, int> linkPairKeyToIndexMap;

    int numRegisteredLinkPairs;

    std::vector<ExtraJoint> extraJoints_;

    void extractInternalBodies(Link* link);    
};

template <class TConstraintForceSolver> class DyWorld : public DyWorldBase
{
public:
    TConstraintForceSolver constraintForceSolver;

    DyWorld() : constraintForceSolver(*this) { }

    virtual void initialize() {
        DyWorldBase::initialize();
        constraintForceSolver.initialize();
    }

    virtual void calcNextState(){
        DyWorldBase::setVirtualJointForces();
        constraintForceSolver.solve();
        DyWorldBase::calcNextState();
    }
};

};

#endif
