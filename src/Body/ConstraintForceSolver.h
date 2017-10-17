/** 
    \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_CONSTRAINT_FORCE_SOLVER_H
#define CNOID_BODY_CONSTRAINT_FORCE_SOLVER_H

#include <cnoid/CollisionDetector>
#include <cnoid/CollisionSeq>
#include "exportdecl.h"

namespace cnoid {

class Link;
class ConstraintForceSolverImpl;
class WorldBase;
class ContactMaterial;
	
class CNOID_EXPORT ConstraintForceSolver
{
    ConstraintForceSolverImpl* impl;
		
public:
    ConstraintForceSolver(WorldBase& world);
    ~ConstraintForceSolver();
		
    void setCollisionDetector(CollisionDetectorPtr detector);
    CollisionDetectorPtr collisionDetector();

    void setFriction(double staticFriction, double slipFliction);
    double staticFriction() const;
    double slipFriction() const;

    void setSelfCollisionEnabled(bool on);
    bool isSelfCollisionEnabled() const;

    void setContactCullingDistance(double thresh);
    double contactCullingDistance() const;
    
    void setContactCullingDepth(double depth);
    double contactCullingDepth();
    
    void setCoefficientOfRestitution(double epsilon);
    double coefficientOfRestitution() const;

    void setGaussSeidelErrorCriterion(double e);
    double gaussSeidelErrorCriterion();

    void setGaussSeidelMaxNumIterations(int n);
    int gaussSeidelMaxNumIterations();

    void setContactDepthCorrection(double depth, double velocityRatio);
    double contactCorrectionDepth();
    double contactCorrectionVelocityRatio();

    void set2Dmode(bool on);
    void enableConstraintForceOutput(bool on);

    void initialize(void);
    void solve();
    void clearExternalForces();

    CollisionLinkPairListPtr getCollisions();

#ifdef ENABLE_SIMULATION_PROFILING
    double getCollisionTime();
#endif

    // experimental functions
    void setFriction(Link* link1, Link* link2, double staticFriction, double slipFriction);
    typedef std::function<bool(Link* link1, Link* link2, const CollisionArray& collisions, ContactMaterial* contactMaterial)>
        CollisionHandler;
    int registerCollisionHandler(const std::string& name, CollisionHandler handler);
    void unregisterCollisionHandler(int handlerId);
    int collisionHandlerId(const std::string& name) const;
    void setCollisionHandler(Link* link1, Link* link2, int handlerId);
};

};

#endif
