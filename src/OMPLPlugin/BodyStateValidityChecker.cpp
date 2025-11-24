#include "BodyStateValidityChecker.h"
#include <cnoid/Body>
#include <cnoid/KinematicBodySet>
#include <cnoid/BodyKinematicsKit>
#include <cnoid/BodyCollisionDetector>
#include <cnoid/AISTCollisionDetector>
#include <cnoid/JointPath>
#include <cnoid/JointSpaceConfigurationHandler>
#include <cnoid/LinkedJointHandler>
#include <cnoid/MathUtil>
#include <ompl/base/spaces/RealVectorStateSpace.h>

using namespace std;
using namespace cnoid;

namespace cnoid {

class BodyStateValidityChecker::Impl
{
public:
    KinematicBodySetPtr targetBodySet;
    vector<BodyPtr> attachedObjects;
    vector<BodyPtr> environmentalObjects;
    BodyCollisionDetector bodyCollisionDetector;
    std::shared_ptr<JointSpaceConfigurationHandler> jointSpaceConfigurationHandler;
    bool isJointSpaceConfigurationHandlerCheckEnabled;
    BodyPtr workspaceBoundsBody;
    Link* lastCollidingLinkPair[2];
    mutable InvalidReason lastInvalidReason;

    Impl();
    bool makeReady();
    bool isValid(const ompl::base::State* state);
    void updateTargetBodyPosition(const ompl::base::State* state);
    bool detectCollision();
    BodyStateValidityChecker::InvalidReason getLastInvalidReason() const;
    bool checkPartOfTargetBody(Body* body) const;
};

}


BodyStateValidityChecker::BodyStateValidityChecker(ompl::base::SpaceInformation* si)
    : ompl::base::StateValidityChecker(si)
{
    impl = new Impl;
}


BodyStateValidityChecker::BodyStateValidityChecker(const ompl::base::SpaceInformationPtr& si)
    : ompl::base::StateValidityChecker(si)
{
    impl = new Impl;
}


BodyStateValidityChecker::Impl::Impl()
{
    bodyCollisionDetector.setCollisionDetector(new AISTCollisionDetector);
    bodyCollisionDetector.setGeometryHandleMapEnabled(true);
    isJointSpaceConfigurationHandlerCheckEnabled = false;
}


BodyStateValidityChecker::~BodyStateValidityChecker()
{
    delete impl;
}


void BodyStateValidityChecker::clearBodies()
{
    impl->bodyCollisionDetector.clearBodies();
    // Disable collision detection bwtween environmental objects
    impl->bodyCollisionDetector.collisionDetector()->setGroupPairEnabled(1, 1, false);

    impl->targetBodySet.reset();
    impl->attachedObjects.clear();
    impl->environmentalObjects.clear();
    impl->workspaceBoundsBody.reset();

    impl->jointSpaceConfigurationHandler.reset();

    impl->lastCollidingLinkPair[0] = nullptr;
    impl->lastCollidingLinkPair[1] = nullptr;
}


void BodyStateValidityChecker::setTargetBodySet(KinematicBodySet* bodySet)
{
    impl->targetBodySet = bodySet;
}


void BodyStateValidityChecker::setTargetBody(Body* body)
{
    // Create a KinematicBodySet with a single body for backward compatibility
    auto bodySet = new KinematicBodySet;
    auto kinematicsKit = new BodyKinematicsKit;
    
    // Use setJointPath to ensure inverse kinematics is available
    auto baseLink = body->rootLink();
    auto endLink = body->guessMainEndLink();
    if(endLink){
        kinematicsKit->setJointPath(baseLink, endLink);
    } else {
        // Fallback to setJointTraverse if no end link can be guessed
        kinematicsKit->setJointTraverse(body);
    }
    
    bodySet->setBodyPart(0, kinematicsKit);
    bodySet->setMainBodyPartIndex(0);
    setTargetBodySet(bodySet);
}


void BodyStateValidityChecker::addAttachedObject(Body* body)
{
    impl->attachedObjects.push_back(body);
}


void BodyStateValidityChecker::addEnvironmentalObject(Body* body)
{
    impl->environmentalObjects.push_back(body);
}


void BodyStateValidityChecker::setWorkspaceBoundsBody(Body* body)
{
    impl->workspaceBoundsBody = body;
}


void BodyStateValidityChecker::setJointSpaceConfigurationHandlerCheckEnabled(bool on)
{
    impl->isJointSpaceConfigurationHandlerCheckEnabled = on;
}


bool BodyStateValidityChecker::makeReady()
{
    return impl->makeReady();
}


bool BodyStateValidityChecker::Impl::makeReady()
{
    if(!targetBodySet || !targetBodySet->mainBodyPart()){
        return false;
    }

    bodyCollisionDetector.clearBodies();
    
    auto targetBody = targetBodySet->mainBodyPart()->body();
    
    bodyCollisionDetector.addBody(targetBody, true, 0);

    auto collisionDetector = bodyCollisionDetector.collisionDetector();
    for(auto& body : attachedObjects){
        bodyCollisionDetector.addBody(body, false, 0);
        bodyCollisionDetector.setLinksInAttachmentIgnored(
            body->rootLink(), body->parentBodyLink(), true);
    }

    for(auto& body : environmentalObjects){
        bodyCollisionDetector.addBody(body, false, 1);
    }

    if(workspaceBoundsBody){
        bodyCollisionDetector.addBody(workspaceBoundsBody, false, 1);
    }

    bodyCollisionDetector.makeReady();

    jointSpaceConfigurationHandler.reset();
    if(isJointSpaceConfigurationHandlerCheckEnabled){
        jointSpaceConfigurationHandler =
            targetBodySet->mainBodyPart()->configurationHandler();
    }

    return true;
}


// \note This function must be thread safe
bool BodyStateValidityChecker::isValid(const ompl::base::State* state) const
{
    return impl->isValid(state);
}


bool BodyStateValidityChecker::Impl::isValid(const ompl::base::State* state)
{
    lastInvalidReason = BodyStateValidityChecker::InvalidReason::Valid;

    updateTargetBodyPosition(state);

    if(jointSpaceConfigurationHandler){
        if(jointSpaceConfigurationHandler->getCurrentNearSingularPointState()){
            lastInvalidReason = BodyStateValidityChecker::InvalidReason::NearSingularPoint;
            return false;
        }
    }

    bool hasCollision = detectCollision();
    
    return !hasCollision;
}


void BodyStateValidityChecker::Impl::updateTargetBodyPosition(const ompl::base::State* state)
{
    auto kinematicsKit = targetBodySet->mainBodyPart();
    auto targetBody = kinematicsKit->body();
    const auto* jointState = state->as<ompl::base::RealVectorStateSpace::StateType>();
    int numJoints = targetBody->numJoints();
    for(int i=0; i < numJoints; ++i){
        targetBody->joint(i)->q() = jointState->values[i];
    }
    kinematicsKit->updateLinkedJointDisplacementsAndCalcFowardKinematics();
}


bool BodyStateValidityChecker::Impl::detectCollision()
{
    auto targetBody = targetBodySet->mainBodyPart()->body();
    
    for(auto& body : attachedObjects){
        body->syncPositionWithParentBody(true);
    }
    
    bodyCollisionDetector.updatePositions();

    lastCollidingLinkPair[0] = nullptr;
    lastCollidingLinkPair[1] = nullptr;

    auto detectCollidingLinkPair = [this, targetBody](const CollisionPair& collisionPair){
        lastCollidingLinkPair[0] = static_cast<Link*>(collisionPair.object(0));
        lastCollidingLinkPair[1] = static_cast<Link*>(collisionPair.object(1));
        return true; // Early termination on first collision
    };

    bool detected = false;
    for(auto& link : targetBody->links()){
        if(bodyCollisionDetector.detectCollisions(link, detectCollidingLinkPair)){
            detected = true;
            break;
        }
    }

    if(!detected){
        for(auto& body : attachedObjects){
            for(auto& link : body->links()){
                if(bodyCollisionDetector.detectCollisions(link, detectCollidingLinkPair)){
                    detected = true;
                    goto exit;
                }
            }
        }
    }
exit:

    return detected;
}


BodyStateValidityChecker::InvalidReason BodyStateValidityChecker::getLastInvalidReason() const
{
    return impl->getLastInvalidReason();
}


BodyStateValidityChecker::InvalidReason BodyStateValidityChecker::Impl::getLastInvalidReason() const
{
    if(lastInvalidReason == InvalidReason::Valid){
        if(lastCollidingLinkPair[0] && lastCollidingLinkPair[1]){
            auto body0 = lastCollidingLinkPair[0]->body();
            auto body1 = lastCollidingLinkPair[1]->body();
            if(body0 == workspaceBoundsBody || body1 == workspaceBoundsBody){
                lastInvalidReason = InvalidReason::OutsideWorkspaceBounds;
            } else if(checkPartOfTargetBody(body0) && checkPartOfTargetBody(body1)){
                lastInvalidReason = InvalidReason::SelfCollision;
            } else {
                lastInvalidReason = InvalidReason::CollisionWithEnvironment;
            }
        }
    }
    return lastInvalidReason;
}


bool BodyStateValidityChecker::Impl::checkPartOfTargetBody(Body* body) const
{
    if(body == targetBodySet->mainBodyPart()->body()){
        return true;
    }
    for(auto& attachedObject : attachedObjects){
        if(body == attachedObject){
            return true;
        }
    }
    return false;
}
