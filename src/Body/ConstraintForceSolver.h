/** \file
    \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_CONSTRAINT_FORCE_SOLVER_H_INCLUDED
#define CNOID_BODY_CONSTRAINT_FORCE_SOLVER_H_INCLUDED

#include "exportdecl.h"

namespace cnoid
{
class Link;
class CFSImpl;
class WorldBase;
class CollisionDetector;
typedef boost::shared_ptr<CollisionDetector> CollisionDetectorPtr;
	
class CNOID_EXPORT ConstraintForceSolver
{
    CFSImpl* impl;
		
public:
    ConstraintForceSolver(WorldBase& world);
    ~ConstraintForceSolver();
		
    void setCollisionDetector(CollisionDetectorPtr detector);
    CollisionDetectorPtr collisionDetector();

    void setDefaultFriction(double staticFriction, double slipFliction);
    void setDefaultCullingThresh(double thresh);
    void setDefaultCoefficientOfRestitution(double epsilon);

    double defaultStaticFriction() const;
    double defaultSlipFriction() const;
    double defaultCullingThresh() const;
    double defaultCoefficientOfRestitution() const;

    void setGaussSeidelErrorCriterion(double e);
    void setGaussSeidelMaxNumIterations(int n);
    void setContactDepthCorrection(double depth, double velocityRatio);
    void setNegativeVelocityRatioForPenetration(double ratio);
    void set2Dmode(bool on);
    void enableConstraintForceOutput(bool on);

    static double defaultGaussSeidelErrorCriterion();
    static int defaultGaussSeidelMaxNumIterations();
    static double defaultContactCorrectionDepth();
    static double defaultContactCorrectionVelocityRatio();

    void initialize(void);
    void solve();
    void clearExternalForces();
};
};


#endif
