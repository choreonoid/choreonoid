#include "BodyStateValidityChecker.h"
#include <cnoid/Body>
#include <cnoid/KinematicBodySet>
#include <cnoid/BodyKinematicsKit>
#include <cnoid/BodyCollisionDetector>
#include <cnoid/AISTCollisionDetector>
#include <cnoid/JointPath>
#include <cnoid/JointSpaceConfigurationHandler>
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
    mutable BodyStateValidityChecker::InvalidReason lastInvalidReason;

    Impl();
    bool makeReady();
    bool isValid(const ompl::base::State* state);
    void updateTargetBodyPosition(const ompl::base::State* state);
    bool detectCollision();
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

    impl->attachedObjects.clear();
    impl->environmentalObjects.clear();
    impl->workspaceBoundsBody.reset();

    impl->jointSpaceConfigurationHandler.reset();
}


void BodyStateValidityChecker::setTargetBodySet(KinematicBodySet* bodySet)
{
    impl->targetBodySet = bodySet;
}


void BodyStateValidityChecker::setTargetBody(Body* body)
{
    // Create a KinematicBodySet with a single body for backward compatibility
    impl->targetBodySet = new KinematicBodySet;
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
    
    impl->targetBodySet->setBodyPart(0, kinematicsKit);
    impl->targetBodySet->setMainBodyPartIndex(0);
}


void BodyStateValidityChecker::addAttachedObject(Body* body)
{
    impl->attachedObjects.push_back(body);
}


void BodyStateValidityChecker::addEnvironmentalObject(Body* body)
{
    impl->environmentalObjects.push_back(body);
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
        if(auto parentLink = body->parentBodyLink()){
            if(auto parentHandle = bodyCollisionDetector.findGeometryHandle(parentLink)){
                for(auto& link : body->links()){
                    collisionDetector->setGeometryPairEnabled(
                        *parentHandle, *bodyCollisionDetector.findGeometryHandle(link), false);
                }
            }
        }
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
        if(auto kinematicsKit = targetBodySet->mainBodyPart()){
            jointSpaceConfigurationHandler = kinematicsKit->configurationHandler();
        }
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
    // Reset to valid before checks
    lastInvalidReason = BodyStateValidityChecker::InvalidReason::Valid;

    updateTargetBodyPosition(state);

    if(jointSpaceConfigurationHandler){
        if(jointSpaceConfigurationHandler->getCurrentNearSingularPointState()){
            lastInvalidReason = BodyStateValidityChecker::InvalidReason::NearSingularPoint;
            return false;
        }
    }

    bool hasCollision = detectCollision();
    // Note: lastInvalidReason is set inside detectCollision() if collision detected
    return !hasCollision;
}


void BodyStateValidityChecker::Impl::updateTargetBodyPosition(const ompl::base::State* state)
{
    auto targetBody = targetBodySet->mainBodyPart()->body();
    
    const auto* jointState = state->as<ompl::base::RealVectorStateSpace::StateType>();
    int numJoints = targetBody->numJoints();
    for(int i=0; i < numJoints; ++i){
        targetBody->joint(i)->q() = jointState->values[i];
    }
    targetBody->calcForwardKinematics();
}


bool BodyStateValidityChecker::Impl::detectCollision()
{
    auto targetBody = targetBodySet->mainBodyPart()->body();
    
    // Update attached objects
    for(auto& body : attachedObjects){
        if(auto parentLink = body->parentBodyLink()){
            auto rootLink = body->rootLink();
            rootLink->setPosition(parentLink->T() * rootLink->Tb());
            body->calcForwardKinematics();
        }
    }
    
    bodyCollisionDetector.updatePositions();

    // Collision callback to classify collision type
    auto checkCollision = [this](const CollisionPair& collisionPair){
        Link* link0 = static_cast<Link*>(collisionPair.object(0));
        Link* link1 = static_cast<Link*>(collisionPair.object(1));

        // Check if either body is the workspace bounds body
        if(link0->body() == workspaceBoundsBody || link1->body() == workspaceBoundsBody){
            lastInvalidReason = BodyStateValidityChecker::InvalidReason::OutsideWorkspaceBounds;
        } else {
            lastInvalidReason = BodyStateValidityChecker::InvalidReason::CollisionWithEnvironment;
        }

        return true; // Early termination on first collision
    };

    // Check collisions
    bool detected = false;
    for(auto& link : targetBody->links()){
        detected = bodyCollisionDetector.detectCollisions(link, checkCollision);
        if(detected){
            break;
        }
    }

    if(!detected){
        for(auto& body : attachedObjects){
            for(auto& link : body->links()){
                detected = bodyCollisionDetector.detectCollisions(link, checkCollision);
                if(detected){
                    break;
                }
            }
        }
    }
    
    return detected;
}


void BodyStateValidityChecker::setWorkspaceBoundsBody(Body* body)
{
    impl->workspaceBoundsBody = body;
}


BodyStateValidityChecker::InvalidReason BodyStateValidityChecker::getLastInvalidReason() const
{
    return impl->lastInvalidReason;
}
