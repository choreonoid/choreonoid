/** 
    \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_CONSTRAINT_FORCE_SOLVER_H
#define CNOID_BODY_CONSTRAINT_FORCE_SOLVER_H

#include <cnoid/CollisionSeq>
#include "exportdecl.h"

namespace cnoid {

class Link;
class DyWorldBase;
class CollisionDetector;
class ContactMaterial;
class MaterialTable;
	
class CNOID_EXPORT ConstraintForceSolver
{
public:
    ConstraintForceSolver(DyWorldBase& world);
    ~ConstraintForceSolver();
		
    void setCollisionDetector(CollisionDetector* detector);
    CollisionDetector* collisionDetector();

    void setBodyCollisionDetectionMode(
        int bodyIndex, bool isBodyToBodyCollisionEnabled, bool isSelfCollisionDetectionEnabled);

    void setMaterialTable(MaterialTable* table);

    void setFrictionCoefficientRange(double minFriction, double maxFriction);
    double minFrictionCoefficient() const;
    double maxFrictionCoefficient() const;

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

    [[deprecated("This function does nothing. Set Link::LinkContactState to Link::sensingMode from a controller.")]]
    void enableConstraintForceOutput(bool on);

    void initialize(void);
    void solve();
    void clearExternalForces();

    std::shared_ptr<CollisionLinkPairList> getCollisions();

    // experimental functions
    typedef std::function<bool(Link* link1, Link* link2,
                               const CollisionArray& collisions,
                               ContactMaterial* contactMaterial)>  CollisionHandler;
    
    void registerCollisionHandler(const std::string& name, CollisionHandler handler);
    bool unregisterCollisionHandler(const std::string& name);

private:
    class Impl;
    Impl* impl;
};

};

#endif
