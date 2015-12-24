/** 
    \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_CONSTRAINT_FORCE_SOLVER_H
#define CNOID_BODY_CONSTRAINT_FORCE_SOLVER_H

#include <cnoid/CollisionSeq>
#include "exportdecl.h"

namespace cnoid {

class Link;
class ConstraintForceSolverImpl;
class WorldBase;
class CollisionDetector;
typedef boost::shared_ptr<CollisionDetector> CollisionDetectorPtr;
	
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

    void setFriction(Link* link1, Link* link2, double staticFriction, double slipFriction);

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
};

};

#endif
