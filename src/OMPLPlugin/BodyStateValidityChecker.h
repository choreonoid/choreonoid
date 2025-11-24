#ifndef CNOID_OMPL_PLUGIN_BODY_STATE_VALIDITY_CHECKER_H
#define CNOID_OMPL_PLUGIN_BODY_STATE_VALIDITY_CHECKER_H

#include <ompl/base/StateValidityChecker.h>

namespace cnoid {

class Body;
class KinematicBodySet;

class BodyStateValidityChecker : public ompl::base::StateValidityChecker
{
public:
    BodyStateValidityChecker(ompl::base::SpaceInformation* si);
    BodyStateValidityChecker(const ompl::base::SpaceInformationPtr& si);
    ~BodyStateValidityChecker();

    void clearBodies();
    void setTargetBodySet(KinematicBodySet* bodySet);
    void setTargetBody(Body* body);
    void addAttachedObject(Body* body);
    void addEnvironmentalObject(Body* body);
    void setWorkspaceBoundsBody(Body* body);
    void setJointSpaceConfigurationHandlerCheckEnabled(bool on);
    bool makeReady();
    
    enum class InvalidReason {
        Valid = 0,
        NearSingularPoint,
        SelfCollision,
        CollisionWithEnvironment,
        OutsideWorkspaceBounds
    };
    InvalidReason getLastInvalidReason() const;

    virtual bool isValid(const ompl::base::State* state) const override;

private:
    class Impl;
    Impl* impl;
};

typedef std::shared_ptr<BodyStateValidityChecker> BodyStateValidityCheckerPtr;

}

#endif
