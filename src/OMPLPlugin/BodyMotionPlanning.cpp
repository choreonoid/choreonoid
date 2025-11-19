#include "BodyMotionPlanning.h"
#include "BodyStateValidityChecker.h"
#include "OMPLPlugin.h"
#include <cnoid/Body>
#include <cnoid/BodyState>
#include <cnoid/KinematicBodySet>
#include <cnoid/BodyKinematicsKit>
#include <cnoid/SceneDrawables>
#include <cnoid/MeshGenerator>
#include <cnoid/MessageOut>
#include <cnoid/Format>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/util/Console.h>
#include <ompl/config.h>
#include <thread>
#include <atomic>
#include <mutex>
#include <sstream>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

enum PlannerType { RRTConnect, PRM, NumPlannerTypes };

}

namespace cnoid {

class BodyMotionPlanning::Impl
{
public:
    using ScopedRealVectorState = ompl::base::ScopedState<ompl::base::RealVectorStateSpace>;

    MessageOut* extMessageOut;
    MessageOut* mout;
    bool outputEnabled;
    std::string errorMessages;

    KinematicBodySetPtr targetBodySet;
    vector<BodyPtr> attachedObjects;
    vector<BodyPtr> environmentalObjects;
    vector<BodyState> waypoints;
    bool isReady;

    PlannerType plannerType;
    bool isPathSimplificationEnabled;
    bool isPathShortcuttingEnabled;
    Vector3 workspaceLowerBound;
    Vector3 workspaceUpperBound;
    SgLineSetPtr boundLineSet;
    bool isWorkspaceBoundsEnabled;

    PathVisualizationMode pathVisualizationMode;
    int pathVisualizationInterpolationSteps;

    std::shared_ptr<ompl::base::StateSpace> space;
    ompl::base::SpaceInformationPtr spaceInformation;
    BodyStateValidityCheckerPtr stateValidityChecker;
    double stateValidityCheckingResolutionRatio;
    bool isJointSpaceConfigurationHandlerCheckEnabled;
    double timeoutSeconds;
    
    std::shared_ptr<ompl::geometric::PathGeometric> solutionPath;
    std::vector<std::shared_ptr<ompl::geometric::PathGeometric>> segmentPaths;
    SgGroupPtr pathVisualizationGroup;

    std::thread solveThread;
    std::atomic<bool> terminationRequested;
    std::mutex threadMutex;

    Impl();
    SgNode* getWorkspaceBoundVisualizationNode();
    void setupWorkspaceBounds();
    bool checkWorkspaceBounds();
    void clear();
    bool makeReady(bool doCloneBodies);
    Result solve();
    Result solveAsynchronously(const std::function<void(Result result)>& onFinished);
    Result solveAllSegments(bool useThread = false);
    Result solveSegment(std::shared_ptr<ompl::base::Planner> planner, int waypointIndex);
    bool checkAndReportStateValidity(const ompl::base::State* state, const char* stateType);
    std::string getWaypointName(int index);
    void setWaypointState(int index, ompl::base::ScopedState<>& out_state);
    bool restoreBodyStateOfSolutionPathState(int index);
    SgNode* getPathVisualizationNode();
    bool createDirectLineVisualization(SgLineSet* lineSet, std::vector<Vector3f>& waypoints, BodyKinematicsKit* kinematicsKit, int numStates);
    bool createJointSpaceInterpolationVisualization(SgLineSet* lineSet, std::vector<Vector3f>& waypoints, BodyKinematicsKit* kinematicsKit, int numStates);
    void createPointVisualization(SgPointSet* pointSet, const std::vector<Vector3f>& waypoints);
};

}


int BodyMotionPlanning::numPlannerTypes()
{
    return NumPlannerTypes;
}


const char* BodyMotionPlanning::plannerTypeName(int index)
{
    switch(index){
    case RRTConnect: return "RRT Connect";
    case PRM: return "PRM";
    default:
        return "None";
    }
}


BodyMotionPlanning::BodyMotionPlanning()
{
    impl = new Impl;
}


BodyMotionPlanning::Impl::Impl()
{
    extMessageOut = MessageOut::master();
    outputEnabled = true;

    mout = new MessageOut(
        [this](const std::string& message, int type) {
            if (type == MessageOut::Error || type == MessageOut::Warning) {
                errorMessages += message;
            }
            if (outputEnabled) {
                extMessageOut->put(message, type);
            }
        },
        [](const std::string&, int) {},
        [this]() {
            if (outputEnabled) {
                extMessageOut->flush();
            }
        }
    );

    isReady = false;

    plannerType = RRTConnect;
    isPathSimplificationEnabled = true;
    isPathShortcuttingEnabled = false;  // Default: OFF for safety
    workspaceLowerBound = Vector3(-10.0, -10.0, -10.0);
    workspaceUpperBound = Vector3(10.0, 10.0, 10.0);
    isWorkspaceBoundsEnabled = false;

    pathVisualizationMode = JointSpaceInterpolation;
    pathVisualizationInterpolationSteps = 10;

    stateValidityCheckingResolutionRatio = 0.001;
    isJointSpaceConfigurationHandlerCheckEnabled = false;
    timeoutSeconds = 20.0;  // Default timeout value
    terminationRequested = false;
}


BodyMotionPlanning::~BodyMotionPlanning()
{
    impl->terminationRequested = true;
    if(impl->solveThread.joinable()){
        impl->solveThread.join();
    }
    delete impl;
}


void BodyMotionPlanning::setMessageOut(MessageOut* mout)
{
    impl->extMessageOut = mout;
}


MessageOut* BodyMotionPlanning::messageOut()
{
    return impl->mout;
}


void BodyMotionPlanning::setMessageOutputEnabled(bool on)
{
    impl->outputEnabled = on;
}


std::string BodyMotionPlanning::getLastErrorMessages() const
{
    return impl->errorMessages;
}


void BodyMotionPlanning::setPlannerType(int index)
{
    impl->plannerType = static_cast<PlannerType>(index);
}


void BodyMotionPlanning::setStateValidityCheckingResolutionRatio(double ratio)
{
    if(ratio > 0.0 && ratio < 1.0){
        impl->stateValidityCheckingResolutionRatio = ratio;
    }
}


void BodyMotionPlanning::setJointSpaceConfigurationHandlerCheckEnabled(bool on)
{
    impl->isJointSpaceConfigurationHandlerCheckEnabled = on;
}


void BodyMotionPlanning::setTimeoutSeconds(double seconds)
{
    if(seconds > 0.0){
        impl->timeoutSeconds = seconds;
    }
}


double BodyMotionPlanning::timeoutSeconds() const
{
    return impl->timeoutSeconds;
}


void BodyMotionPlanning::setPathSimplificationEnabled(bool on)
{
    impl->isPathSimplificationEnabled = on;
}


bool BodyMotionPlanning::isPathSimplificationEnabled() const
{
    return impl->isPathSimplificationEnabled;
}


void BodyMotionPlanning::setPathShortcuttingEnabled(bool on)
{
    impl->isPathShortcuttingEnabled = on;
}


bool BodyMotionPlanning::isPathShortcuttingEnabled() const
{
    return impl->isPathShortcuttingEnabled;
}


void BodyMotionPlanning::setPathVisualizationMode(PathVisualizationMode mode)
{
    impl->pathVisualizationMode = mode;
}


BodyMotionPlanning::PathVisualizationMode BodyMotionPlanning::pathVisualizationMode() const
{
    return impl->pathVisualizationMode;
}


void BodyMotionPlanning::setPathVisualizationInterpolationSteps(int steps)
{
    if(steps > 0){
        impl->pathVisualizationInterpolationSteps = steps;
    }
}


int BodyMotionPlanning::pathVisualizationInterpolationSteps() const
{
    return impl->pathVisualizationInterpolationSteps;
}


void BodyMotionPlanning::setWorkspaceBounds(const Vector3& lower, const Vector3& upper)
{
    impl->workspaceLowerBound = lower;
    impl->workspaceUpperBound = upper;
    impl->isWorkspaceBoundsEnabled = true;
    impl->isReady = false;
}


void BodyMotionPlanning::clearWorkspaceBounds()
{
    impl->isWorkspaceBoundsEnabled = false;
    impl->isReady = false;
}


const Vector3& BodyMotionPlanning::workspaceLowerBound() const
{
    return impl->workspaceLowerBound;
}


const Vector3& BodyMotionPlanning::workspaceUpperBound() const
{
    return impl->workspaceUpperBound;
}


SgNode* BodyMotionPlanning::getWorkspaceBoundVisualizationNode()
{
    return impl->getWorkspaceBoundVisualizationNode();
}


SgNode* BodyMotionPlanning::Impl::getWorkspaceBoundVisualizationNode()
{
    if(!isWorkspaceBoundsEnabled){
        return nullptr;
    }

    if(!boundLineSet){
        boundLineSet = new SgLineSet;
        boundLineSet->setLineWidth(2.0f);
        boundLineSet->getOrCreateMaterial()->setDiffuseColor(Vector3f(1.0f, 0.0f, 0.0f));
        boundLineSet->addLine(0, 1);
        boundLineSet->addLine(1, 2);
        boundLineSet->addLine(2, 3);
        boundLineSet->addLine(3, 0);
        boundLineSet->addLine(0, 4);
        boundLineSet->addLine(1, 5);
        boundLineSet->addLine(2, 6);
        boundLineSet->addLine(3, 7);
        boundLineSet->addLine(4, 5);
        boundLineSet->addLine(5, 6);
        boundLineSet->addLine(6, 7);
        boundLineSet->addLine(7, 4);
    }

    auto& vertices = *boundLineSet->getOrCreateVertices();
    vertices.resize(8);
    const auto& l = workspaceLowerBound;
    const auto& u = workspaceUpperBound;
    vertices[0] << l.x(), l.y(), l.z();
    vertices[1] << l.x(), u.y(), l.z();
    vertices[2] << u.x(), u.y(), l.z();
    vertices[3] << u.x(), l.y(), l.z();
    vertices[4] << l.x(), l.y(), u.z();
    vertices[5] << l.x(), u.y(), u.z();
    vertices[6] << u.x(), u.y(), u.z();
    vertices[7] << u.x(), l.y(), u.z();

    return boundLineSet;
}


void BodyMotionPlanning::clear()
{
    impl->clear();
}


void BodyMotionPlanning::Impl::clear()
{
    if(stateValidityChecker){
        stateValidityChecker->clearBodies();
    }
    targetBodySet.reset();
    attachedObjects.clear();
    environmentalObjects.clear();
    waypoints.clear();
    solutionPath.reset();
    segmentPaths.clear();
    isReady = false;
    pathVisualizationGroup.reset();
    terminationRequested = false;
}


void BodyMotionPlanning::setTargetBodySet(KinematicBodySet* bodySet)
{
    impl->targetBodySet = bodySet;
    impl->isReady = false;
}


KinematicBodySet* BodyMotionPlanning::targetBodySet()
{
    return impl->targetBodySet;
}


void BodyMotionPlanning::setTargetBody(Body* body)
{
    impl->targetBodySet = new KinematicBodySet;
    auto kinematicsKit = new BodyKinematicsKit;
    kinematicsKit->setJointTraverse(body);
    impl->targetBodySet->setBodyPart(0, kinematicsKit);
    impl->targetBodySet->setMainBodyPartIndex(0);
    impl->isReady = false;
}


void BodyMotionPlanning::addAttachedObject(Body* body)
{
    impl->attachedObjects.push_back(body);
    impl->isReady = false;
}


void BodyMotionPlanning::addEnvironmentalObject(Body* body)
{
    impl->environmentalObjects.push_back(body);
    impl->isReady = false;
}


void BodyMotionPlanning::addCurrentBodyStateAsWaypoint()
{
    if(impl->targetBodySet){
        BodyState state;
        auto body = impl->targetBodySet->mainBodyPart()->body();
        state.storePositionOfBody(body);
        impl->waypoints.push_back(std::move(state));
    }
    impl->isReady = false;
}


bool BodyMotionPlanning::Impl::makeReady(bool doCloneBodies)
{
    errorMessages.clear();

    if(!targetBodySet){
        return false;
    }
    if(waypoints.size() < 2){
        return false;
    }
    if(!checkWorkspaceBounds()){
        return false;
    }

    CloneMap cloneMap;
    SgObject::setNonNodeCloning(cloneMap, false);

    KinematicBodySetPtr targetBodySet = this->targetBodySet;
    if(doCloneBodies){
        targetBodySet = this->targetBodySet->clone(cloneMap);
    }

    BodyPtr targetBody = targetBodySet->mainBodyPart()->body();

    // Create state space for joint space planning
    auto numJoints = targetBody->numJoints();
    auto jointSpace = make_shared<ompl::base::RealVectorStateSpace>(numJoints);
    
    ompl::base::RealVectorBounds bounds(numJoints);
    for(int i=0; i < numJoints; ++i){
        auto joint = targetBody->joint(i);
        double lower = joint->q_lower();
        double upper = joint->q_upper();
        if(joint->isRevoluteJoint()){
            lower = std::max(lower, -2.0 * M_PI);
            upper = std::min(upper, 2.0 * M_PI);
        }
        bounds.setLow(i, lower);
        bounds.setHigh(i, upper);
    }
    jointSpace->setBounds(bounds);
    space = jointSpace;
    
    // Common setup for both space types
    spaceInformation = make_shared<ompl::base::SpaceInformation>(space);
    spaceInformation->setStateValidityCheckingResolution(stateValidityCheckingResolutionRatio);
    stateValidityChecker = make_shared<BodyStateValidityChecker>(spaceInformation);
    spaceInformation->setStateValidityChecker(stateValidityChecker);
    
    stateValidityChecker->setTargetBodySet(targetBodySet);

    BodyPtr body;
    for(auto& object : attachedObjects){
        body = doCloneBodies ? object->clone(cloneMap) : object.get();
        stateValidityChecker->addAttachedObject(body);
    }
    for(auto& object : environmentalObjects){
        body = doCloneBodies ? object->clone(cloneMap) : object.get();
        stateValidityChecker->addEnvironmentalObject(body);
    }

    setupWorkspaceBounds();

    stateValidityChecker->setJointSpaceConfigurationHandlerCheckEnabled(
        isJointSpaceConfigurationHandlerCheckEnabled);

    if(!stateValidityChecker->makeReady()){
        return false;
    }

    spaceInformation->setup();

    isReady = true;
    return true;
}


void BodyMotionPlanning::Impl::setupWorkspaceBounds()
{
    if(!isWorkspaceBoundsEnabled){
        stateValidityChecker->setWorkspaceBoundsBody(nullptr);
        return;
    }

    const auto& lower = workspaceLowerBound;
    const auto& upper = workspaceUpperBound;
    if(lower.x() >= upper.x() || lower.y() >= upper.y() || lower.z() >= upper.z()){
        stateValidityChecker->setWorkspaceBoundsBody(nullptr);
        return;
    }

    constexpr double thickness = 1.0;
    const double width_x = upper.x() - lower.x();
    const double width_y = upper.y() - lower.y();
    const double width_z = upper.z() - lower.z();
    const double center_x = (lower.x() + upper.x()) / 2.0;
    const double center_y = (lower.y() + upper.y()) / 2.0;
    const double center_z = (lower.z() + upper.z()) / 2.0;

    auto body = new Body;
    // Initialize body with root link
    auto rootLink = new Link;
    body->setRootLink(rootLink);

    // Generate box meshes using MeshGenerator (3 meshes for 3 pairs of walls)
    MeshGenerator meshGenerator;

    // Create shared meshes for each pair of opposing walls
    auto xWallMesh = meshGenerator.generateBox(Vector3(thickness, width_y, width_z));
    auto yWallMesh = meshGenerator.generateBox(Vector3(width_x, thickness, width_z));
    auto zWallMesh = meshGenerator.generateBox(Vector3(width_x, width_y, thickness));

    // Helper lambda to add box wall with shared mesh
    auto addBoxWall = [&](const Vector3& position, SgMesh* mesh){
        auto shape = new SgShape;
        shape->setMesh(mesh);
        auto posTransform = new SgPosTransform;
        posTransform->setTranslation(position);
        posTransform->addChild(shape);
        auto link = body->createLink();
        link->addShapeNode(posTransform);
        body->rootLink()->appendChild(link);
    };

    // X-axis walls (left and right) - share xWallMesh
    addBoxWall(Vector3(lower.x() - thickness/2.0, center_y, center_z), xWallMesh);
    addBoxWall(Vector3(upper.x() + thickness/2.0, center_y, center_z), xWallMesh);

    // Y-axis walls (front and back) - share yWallMesh
    addBoxWall(Vector3(center_x, lower.y() - thickness/2.0, center_z), yWallMesh);
    addBoxWall(Vector3(center_x, upper.y() + thickness/2.0, center_z), yWallMesh);

    // Z-axis walls (top and bottom) - share zWallMesh
    addBoxWall(Vector3(center_x, center_y, upper.z() + thickness/2.0), zWallMesh);
    addBoxWall(Vector3(center_x, center_y, lower.z() - thickness/2.0), zWallMesh);

    body->updateLinkTree();

    stateValidityChecker->setWorkspaceBoundsBody(body);
}


bool BodyMotionPlanning::Impl::checkWorkspaceBounds()
{
    bool isValid = true;
    if(isWorkspaceBoundsEnabled){
        // Check if the initial position of the robot base link is within the bounds.
        // Since the robot base is fixed during planning (only joint angles change),
        // a single check at initialization is sufficient.
        auto baseLink = targetBodySet->mainBodyPart()->body()->rootLink();
        const Vector3& basePos = baseLink->translation();
        if(basePos.x() < workspaceLowerBound.x() || basePos.x() > workspaceUpperBound.x() ||
           basePos.y() < workspaceLowerBound.y() || basePos.y() > workspaceUpperBound.y() ||
           basePos.z() < workspaceLowerBound.z() || basePos.z() > workspaceUpperBound.z()){
            mout->putError(_("Robot position is outside workspace bounds."));
            isValid = false;
        }
    }
    return isValid;
}


BodyMotionPlanning::Result BodyMotionPlanning::solve()
{
    return impl->solve();
}


BodyMotionPlanning::Result BodyMotionPlanning::Impl::solve()
{
    if(!isReady){
        if(!makeReady(false)){
            return Invalid;
        }
    }
    auto result = solveAllSegments();

    pathVisualizationGroup.reset();

    return result;
}


BodyMotionPlanning::Result BodyMotionPlanning::solveAsynchronously(std::function<void(Result result)> onFinished)
{
    return impl->solveAsynchronously(onFinished);
}


BodyMotionPlanning::Result BodyMotionPlanning::Impl::solveAsynchronously(const std::function<void(Result result)>& onFinished)
{
    std::lock_guard<std::mutex> lock(threadMutex);
    
    // If previous thread is still running, wait for it to finish
    if(solveThread.joinable()){
        terminationRequested = true;
        solveThread.join();
    }
    
    // Prepare in the calling thread
    if(!isReady){
        if(!makeReady(true)){
            return Invalid;
        }
    }
    
    // Reset termination flag
    terminationRequested = false;
    
    // Start new thread
    solveThread = std::thread([this, onFinished](){
        auto result = solveAllSegments(true);
        pathVisualizationGroup.reset();
        if(onFinished){
            onFinished(result);
        }
    });
    
    return Started;
}


void BodyMotionPlanning::terminate()
{
    impl->terminationRequested = true;
    if(impl->solveThread.joinable()){
        impl->solveThread.join();
    }
}


BodyMotionPlanning::Result BodyMotionPlanning::Impl::solveAllSegments(bool useThread)
{
    // Create planner once for all segments
    std::shared_ptr<ompl::base::Planner> planner;
    switch(plannerType){
    case RRTConnect: {
        auto rrtconnect = make_shared<ompl::geometric::RRTConnect>(spaceInformation);
        rrtconnect->setRange(0.15);
        planner = rrtconnect;
        break;
    }
    case PRM: {
        auto prm = make_shared<ompl::geometric::PRM>(spaceInformation);
        // Improve PRM stability and performance
        prm->setMaxNearestNeighbors(15);  // Default is 10, increase for better connectivity
        planner = prm;
        break;
    }
    default:
        return NotSolved;
    }

    // Setup the planner once before solving all segments
    // This should be called only once per planner instance
    planner->setup();

    Result result = NotSolved;
    int numSegments = waypoints.size() - 1;
    segmentPaths.clear();
    segmentPaths.reserve(numSegments);
    
    for(int i=0; i < numSegments; ++i){
        if(terminationRequested){
            result = Terminated;
            break;
        }
        result = solveSegment(planner, i);
        if(result != Solved){
            break;
        }
    }
    
    // Combine all segment paths into a single solution path
    if(result == Solved && !segmentPaths.empty()){
        solutionPath = make_shared<ompl::geometric::PathGeometric>(spaceInformation);
        
        // Add states from all segments
        for(size_t i = 0; i < segmentPaths.size(); ++i){
            auto& segmentPath = segmentPaths[i];
            int startIdx = (i == 0) ? 0 : 1; // Skip the first state of subsequent segments to avoid duplication
            for(int j = startIdx; j < segmentPath->getStateCount(); ++j){
                solutionPath->append(segmentPath->getState(j));
            }
        }
        
        mout->putln(
            formatR(_("Combined path has {0} states from {1} segments"),
                    solutionPath->getStateCount(), segmentPaths.size()));
    }
    
    return result;
}


bool BodyMotionPlanning::Impl::checkAndReportStateValidity(
    const ompl::base::State* state, const char* stateType)
{
    if(!spaceInformation->isValid(state)){
        using InvalidReason = BodyStateValidityChecker::InvalidReason;
        InvalidReason reason = stateValidityChecker->getLastInvalidReason();

        switch(reason){
        case InvalidReason::NearSingularPoint:
            mout->putErrorln(formatR(_("{0} is near a singular configuration."), stateType));
            break;
        case InvalidReason::CollisionWithEnvironment:
            mout->putErrorln(formatR(_("{0} is in collision with the environment."), stateType));
            break;
        case InvalidReason::OutsideWorkspaceBounds:
            mout->putErrorln(formatR(_("{0} is outside the workspace bounds."), stateType));
            break;
        default:
            mout->putErrorln(formatR(_("{0} is invalid."), stateType));
            break;
        }
        return false;
    }
    return true;
}


std::string BodyMotionPlanning::Impl::getWaypointName(int index)
{
    int totalWaypoints = waypoints.size();
    if(index == 0){
        return _("Start point");
    } else if(index == totalWaypoints - 1){
        return _("Goal point");
    } else {
        return formatR(_("{0}th waypoint"), index + 1);
    }
}


BodyMotionPlanning::Result BodyMotionPlanning::Impl::solveSegment
(std::shared_ptr<ompl::base::Planner> planner, int waypointIndex)
{
    Result result = NotSolved;
    bool timedOut = false;
    
    // Create a new ProblemDefinition for each segment to ensure clean state
    auto problem = make_shared<ompl::base::ProblemDefinition>(spaceInformation);
    
    // Use ScopedState for automatic memory management
    ompl::base::ScopedState<> start(space);
    ompl::base::ScopedState<> goal(space);
    
    // Set waypoint states
    setWaypointState(waypointIndex, start);
    setWaypointState(waypointIndex + 1, goal);

    // Convert states to string for output
    std::ostringstream startStr, goalStr;
    space->printState(start.get(), startStr);
    space->printState(goal.get(), goalStr);
    
    // Remove trailing newline if present
    std::string startString = startStr.str();
    std::string goalString = goalStr.str();
    if(!startString.empty() && startString.back() == '\n') {
        startString.pop_back();
    }
    if(!goalString.empty() && goalString.back() == '\n') {
        goalString.pop_back();
    }
    
    std::string startName = getWaypointName(waypointIndex);
    std::string goalName = getWaypointName(waypointIndex + 1);

    mout->putln(formatR("{0}:\n{1}", startName, startString));
    mout->putln(formatR("{0}:\n{1}", goalName, goalString));

    if(!checkAndReportStateValidity(start.get(), startName.c_str())){
        return result;
    }
    if(!checkAndReportStateValidity(goal.get(), goalName.c_str())){
        return result;
    }

    problem->setStartAndGoalStates(start, goal);
    //problem->getGoal()->as<ompl::base::GoalState>()->setThreshold(1e-6);

    // Clear previous problem based on planner type
    if(plannerType == PRM){
        // For PRM, use clearQuery to preserve the roadmap
        planner->as<ompl::geometric::PRM>()->clearQuery();
    } else {
        // For other planners, do full clear
        planner->clear();
    }

    planner->setProblemDefinition(problem);
    // Note: setup() is called once in solveAllSegments(), not here.
    // Calling setup() multiple times causes OMPL to emit a warning.

#ifdef NDEBUG
    // Combine termination flag check with time limit in release mode
    auto startTime = std::chrono::steady_clock::now();
    auto status = planner->solve(ompl::base::PlannerTerminationCondition(
        [this, &timedOut, startTime](){
            if(terminationRequested.load()){
                return true;
            }
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - startTime).count();
            if(elapsed >= timeoutSeconds){
                timedOut = true;
                return true;
            }
            return false;
        }
    ));
#else
    // In debug mode, only check termination flag
    auto status = planner->solve(ompl::base::PlannerTerminationCondition(
        [this](){ return terminationRequested.load(); }
    ));
#endif

    if(status != ompl::base::PlannerStatus::EXACT_SOLUTION){
        if(terminationRequested){
            result = Terminated;
            mout->putln(_("Motion planning was terminated."));
        } else if(timedOut){
            result = Timeout;
            mout->putln(_("Motion planning timed out."));
        } else {
            result = NotSolved;
            mout->putln(_("Motion planning failed. No solution was found."));
        }
        if(solutionPath){
            solutionPath->clear();
        }

    } else {
        auto path = problem->getSolutionPath()->as<ompl::geometric::PathGeometric>();
        
        if(!isPathSimplificationEnabled){
            mout->putln(
                formatR(_("Path simplification disabled. Path length: {0} states"),
                        path->getStateCount()));
        } else {
            mout->putln(formatR(_("Original path: {0} states (waypoints)"), path->getStateCount()));

            // Optimize the path using PathSimplifier
            ompl::geometric::PathSimplifier simplifier(spaceInformation);

            // Get the state validity checking resolution
            double checkingResolution = spaceInformation->getStateValidityCheckingResolution();

            size_t initialStateCount = path->getStateCount();

            // ==================================================================
            // Phase 1: ESSENTIAL vertex reduction (always performed)
            // This is REQUIRED for RRT-Connect/PRM outputs to be usable
            // ==================================================================
            mout->putln(_("Phase 1: Essential vertex reduction..."));

            // Step 1a: Iterative reduceVertices with increasing range
            bool madeProgress = true;
            int reductionRound = 0;
            const int maxRounds = 30;  // Safety limit
            int consecutiveFailures = 0;
            const int maxConsecutiveFailures = 5;  // Stop after 5 consecutive validation failures
            double currentRangeRatio = 0.33;

            while(madeProgress && reductionRound < maxRounds) {
                size_t beforeCount = path->getStateCount();

                // Save current path state for rollback if needed
                ompl::geometric::PathGeometric savedPath(*path);

                // Gradually increase range ratio for more aggressive reduction
                if(reductionRound > 10) {
                    currentRangeRatio = std::min(0.5, currentRangeRatio + 0.02);
                }

                // Try to reduce vertices
                simplifier.reduceVertices(*path,
                                         100,   // maxSteps - many attempts per round
                                         20,    // maxEmptySteps - keep trying
                                         currentRangeRatio);

                size_t afterCount = path->getStateCount();

                // Check if reduction happened
                if(afterCount < beforeCount) {
                    // Validate the path
                    if(path->check()) {
                        mout->putln(formatR(_("  Round {0}: {1} -> {2} waypoints (removed {3}, range={4:.2f})"),
                                            reductionRound + 1, beforeCount, afterCount,
                                            beforeCount - afterCount, currentRangeRatio));
                        madeProgress = true;
                        consecutiveFailures = 0;  // Reset failure counter
                    } else {
                        // Rollback - validation failed, try next round with different parameters
                        *path = savedPath;
                        mout->putWarningln(formatR(_("  Round {0}: Validation failed, rolling back (range={1:.2f})"),
                                                  reductionRound + 1, currentRangeRatio));
                        consecutiveFailures++;

                        // Stop if too many consecutive failures
                        if(consecutiveFailures >= maxConsecutiveFailures) {
                            mout->putWarningln(_("  Too many consecutive validation failures, stopping reduction"));
                            break;
                        }
                    }
                } else {
                    // No reduction achieved
                    madeProgress = false;
                }

                reductionRound++;
            }

            // Step 1b: After main reduction, apply collapseCloseVertices with retry
            {
                size_t beforeCollapse = path->getStateCount();
                bool collapseSucceeded = false;
                const int maxCollapseRetries = 3;

                for(int retry = 0; retry < maxCollapseRetries; ++retry) {
                    ompl::geometric::PathGeometric savedPath(*path);

                    simplifier.collapseCloseVertices(*path, 50, 10);
                    size_t afterCollapse = path->getStateCount();

                    if(afterCollapse < beforeCollapse) {
                        if(path->check()) {
                            mout->putln(formatR(_("  Collapsed close vertices: {0} -> {1} waypoints"),
                                                beforeCollapse, afterCollapse));
                            collapseSucceeded = true;
                            break;
                        } else {
                            *path = savedPath;
                            if(retry < maxCollapseRetries - 1) {
                                mout->putWarningln(formatR(_("  Collapse attempt {0} failed validation, retrying..."),
                                                          retry + 1));
                            }
                        }
                    } else {
                        // No collapse achieved, no point in retrying
                        break;
                    }
                }

                if(!collapseSucceeded && beforeCollapse > path->getStateCount()) {
                    mout->putWarningln(_("  All collapse attempts failed validation"));
                }
            }

            mout->putln(formatR(_("Phase 1 complete: {0} -> {1} waypoints ({2:.1f}% reduction)"),
                                initialStateCount, path->getStateCount(),
                                (1.0 - (double)path->getStateCount() / initialStateCount) * 100.0));

            // ==================================================================
            // Phase 2: OPTIONAL shortcutting (only if enabled)
            // This can further optimize but may get closer to obstacles
            // ==================================================================
            if(isPathShortcuttingEnabled) {
                mout->putln(_("Phase 2: Optional path shortcutting (enabled)..."));

                size_t beforeStates = path->getStateCount();
                double beforeLength = path->length();

                // Try shortcutting with retry on failure
                const int maxRetries = 3;
                bool shortcutSucceeded = false;
                bool anyPathChanged = false;

                for(int retry = 0; retry < maxRetries; ++retry) {
                    // Save current state for potential rollback
                    ompl::geometric::PathGeometric savedPath(*path);

                    // Conservative parameters for general-purpose use
                    // These are tuned to be safe across different scenarios
#if OMPL_VERSION_VALUE >= 1007000
                    // OMPL 1.7.0+: shortcutPath was renamed to partialShortcutPath
                    // NOTE: Consider using ropeShortcutPath() instead for OMPL 1.7.0+.
                    // It is more efficient (deterministic vs random) and produces statistically
                    // shorter paths. See: https://github.com/ompl/ompl/pull/1120
                    bool pathChanged = simplifier.partialShortcutPath(*path,
                                                              50,     // maxSteps - moderate attempts
                                                              25,     // maxEmptySteps - stop if not productive
                                                              0.25,   // rangeRatio - only look 1/4 of path
                                                              0.01);  // snapToVertex - snap to existing vertices
#else
                    // OMPL 1.5.0/1.6.0: use shortcutPath
                    bool pathChanged = simplifier.shortcutPath(*path,
                                                              50,     // maxSteps - moderate attempts
                                                              25,     // maxEmptySteps - stop if not productive
                                                              0.25,   // rangeRatio - only look 1/4 of path
                                                              0.01);  // snapToVertex - snap to existing vertices
#endif

                    if(pathChanged) {
                        anyPathChanged = true;
                        // Check validity
                        if(path->check()) {
                            size_t afterStates = path->getStateCount();
                            double afterLength = path->length();

                            mout->putln(formatR(_("  Shortcut applied: states {0} -> {1}, length {2:.4f} -> {3:.4f} ({4:.2f}% shorter)"),
                                                beforeStates, afterStates,
                                                beforeLength, afterLength,
                                                (beforeLength - afterLength) / beforeLength * 100.0));
                            shortcutSucceeded = true;
                            break;  // Success, exit retry loop
                        } else {
                            // Rollback and try again
                            *path = savedPath;
                            mout->putWarningln(formatR(_("  Shortcut attempt {0} failed validation, retrying..."),
                                                       retry + 1));
                        }
                    } else {
                        if(retry == 0) {
                            mout->putln(_("  No shortcuts found"));
                        }
                        break;  // No changes, no point retrying
                    }
                }

                if(!shortcutSucceeded && anyPathChanged) {
                    mout->putWarningln(_("  All shortcut attempts failed validation"));
                }
            } else {
                mout->putln(_("Phase 2: Shortcutting disabled"));
            }

            // ==================================================================
            // Phase 3: Final aggressive reduction pass
            // One more attempt with maximum range
            // ==================================================================
            mout->putln(_("Phase 3: Final optimization pass..."));

            size_t beforeFinal = path->getStateCount();

            // Step 3a: One more collapseCloseVertices (especially useful after shortcut)
            {
                const int maxFinalCollapseRetries = 2;
                bool finalCollapseSucceeded = false;
                size_t beforeFinalCollapse = path->getStateCount();

                for(int retry = 0; retry < maxFinalCollapseRetries; ++retry) {
                    ompl::geometric::PathGeometric savedPath(*path);
                    simplifier.collapseCloseVertices(*path, 30, 10);

                    if(path->check()) {
                        finalCollapseSucceeded = true;
                        break;
                    } else {
                        *path = savedPath;
                        if(retry < maxFinalCollapseRetries - 1) {
                            mout->putWarningln(formatR(_("  Final collapse attempt {0} failed, retrying..."),
                                                      retry + 1));
                        }
                    }
                }

                if(!finalCollapseSucceeded && beforeFinalCollapse > path->getStateCount()) {
                    mout->putWarningln(_("  Final collapse vertices failed all attempts"));
                }
            }

            size_t afterCollapse2 = path->getStateCount();

            // Step 3b: Very aggressive reduceVertices with maximum range
            {
                const int maxFinalReduceRetries = 2;
                bool finalReduceSucceeded = false;
                size_t beforeFinalReduce = path->getStateCount();

                for(int retry = 0; retry < maxFinalReduceRetries; ++retry) {
                    ompl::geometric::PathGeometric savedPath(*path);
                    simplifier.reduceVertices(*path,
                                             200,   // maxSteps - many attempts
                                             50,    // maxEmptySteps
                                             0.5);  // rangeRatio - look at half the path

                    if(path->check()) {
                        finalReduceSucceeded = true;
                        break;
                    } else {
                        *path = savedPath;
                        if(retry < maxFinalReduceRetries - 1) {
                            mout->putWarningln(formatR(_("  Final reduction attempt {0} failed, retrying..."),
                                                      retry + 1));
                        }
                    }
                }

                if(!finalReduceSucceeded && beforeFinalReduce > path->getStateCount()) {
                    mout->putWarningln(_("  Final reduction failed all attempts"));
                }
            }

            size_t afterFinal = path->getStateCount();

            if(afterFinal < beforeFinal) {
                mout->putln(formatR(_("  Final pass: {0} -> {1} waypoints (removed {2})"),
                                    beforeFinal, afterFinal, beforeFinal - afterFinal));
            } else {
                mout->putln(_("  No further reduction possible"));
            }

            // ==================================================================
            // Summary
            // ==================================================================
            mout->putln(formatR(_("FINAL RESULT: {0} -> {1} waypoints ({2:.1f}% reduction)"),
                                initialStateCount, path->getStateCount(),
                                (1.0 - (double)path->getStateCount() / initialStateCount) * 100.0));

            // Final validation - this should always pass now
            if(!path->check()) {
                mout->putErrorln(_("ERROR: Final path validation failed despite rollbacks!"));
            }
        }
        
        // Store this segment's path
        auto segmentPath = make_shared<ompl::geometric::PathGeometric>(*path);
        
        // Set the final state exactly to the goal state (required for manipulator precision)
        ompl::base::State* lastState = segmentPath->getState(segmentPath->getStateCount() - 1);
        space->copyState(lastState, goal.get());
        
        segmentPaths.push_back(segmentPath);
        
        mout->putln(formatR(_("Found solution for segment {0}:"), waypointIndex));
        mout->putln(formatR(_("Segment path has {0} states"), segmentPath->getStateCount()));

        result = Solved;
    }
    
    return result;
}


void BodyMotionPlanning::Impl::setWaypointState(int index, ompl::base::ScopedState<>& out_state)
{
    const auto& bodyState = waypoints[index];
    auto n = bodyState.numJointDisplacements();
    const double* jointDisplacements = bodyState.jointDisplacements();
    auto* jointState = out_state->as<ompl::base::RealVectorStateSpace::StateType>();
    for(int i=0; i < n; ++i){
        jointState->values[i] = jointDisplacements[i];
    }
}


int BodyMotionPlanning::numSolutionPathStates() const
{
    return impl->solutionPath->getStateCount();
}


bool BodyMotionPlanning::restoreBodyStateOfSolutionPathState(int index)
{
    return impl->restoreBodyStateOfSolutionPathState(index);
}


bool BodyMotionPlanning::Impl::restoreBodyStateOfSolutionPathState(int index)
{
    const auto state = solutionPath->getState(index)->as<ompl::base::RealVectorStateSpace::StateType>();
    auto body = targetBodySet->mainBodyPart()->body();
    int numJoints = body->numJoints();
    for(int j = 0; j < numJoints; ++j){
        body->joint(j)->q() = state->values[j];
    }
    body->calcForwardKinematics();
    return true;
}


SgNode* BodyMotionPlanning::getPathVisualizationNode()
{
    return impl->getPathVisualizationNode();
}


SgNode* BodyMotionPlanning::Impl::getPathVisualizationNode()
{
    pathVisualizationGroup.reset();

    BodyKinematicsKit* kinematicsKit = targetBodySet->mainBodyPart();

    if(kinematicsKit && solutionPath){
        int numStates = solutionPath->getStateCount();
        if(numStates >= 2){
            std::vector<Vector3f> waypoints;

            pathVisualizationGroup = new SgGroup;

            // Create path line visualization and collect waypoints
            auto lineSet = new SgLineSet;
            lineSet->setLineWidth(2.0f);
            lineSet->getOrCreateMaterial()->setDiffuseColor(Vector3f(1.0f, 0.0f, 0.0f));

            bool success;
            if(pathVisualizationMode == DirectLine){
                success = createDirectLineVisualization(lineSet, waypoints, kinematicsKit, numStates);
            } else {
                success = createJointSpaceInterpolationVisualization(lineSet, waypoints, kinematicsKit, numStates);
            }

            if(!success){
                pathVisualizationGroup.reset();
                return nullptr;
            }

            pathVisualizationGroup->addChild(lineSet);

            // Create waypoint visualization using collected waypoints
            auto pointSet = new SgPointSet;
            pointSet->setPointSize(8.0f);
            pointSet->getOrCreateMaterial()->setDiffuseColor(Vector3f(1.0f, 0.0f, 0.0f));
            createPointVisualization(pointSet, waypoints);
            pathVisualizationGroup->addChild(pointSet);
        }
    }

    return pathVisualizationGroup;
}


bool BodyMotionPlanning::Impl::createDirectLineVisualization
(SgLineSet* lineSet, std::vector<Vector3f>& waypoints, BodyKinematicsKit* kinematicsKit, int numStates)
{
    auto& vertices = *lineSet->getOrCreateVertices();
    waypoints.clear();

    // Compute waypoints and create lines
    for(int i = 0; i < numStates; ++i){
        if(!restoreBodyStateOfSolutionPathState(i)){
            return false;
        }
        Vector3f p = kinematicsKit->globalEndPosition().translation().cast<float>();
        waypoints.push_back(p);
        vertices.push_back(p);
        if(i >= 1){
            lineSet->addLine(i - 1, i);
        }
    }

    return true;
}


bool BodyMotionPlanning::Impl::createJointSpaceInterpolationVisualization
(SgLineSet* lineSet, std::vector<Vector3f>& waypoints, BodyKinematicsKit* kinematicsKit, int numStates)
{
    auto& vertices = *lineSet->getOrCreateVertices();
    waypoints.clear();

    auto body = targetBodySet->mainBodyPart()->body();
    int numJoints = body->numJoints();

    std::vector<double> q0(numJoints);
    std::vector<double> q1(numJoints);
    std::vector<double> q_interp(numJoints);

    int vertexIndex = 0;

    for(int i = 0; i < numStates - 1; ++i){
        // Get joint angles at state i
        const auto state0 = solutionPath->getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
        for(int j = 0; j < numJoints; ++j){
            q0[j] = state0->values[j];
        }

        // Get joint angles at state i+1
        const auto state1 = solutionPath->getState(i + 1)->as<ompl::base::RealVectorStateSpace::StateType>();
        for(int j = 0; j < numJoints; ++j){
            q1[j] = state1->values[j];
        }

        // Interpolate between state i and state i+1
        int numInterpPoints = (i == 0) ? (pathVisualizationInterpolationSteps + 1) : pathVisualizationInterpolationSteps;
        for(int k = 0; k < numInterpPoints; ++k){
            double ratio = static_cast<double>(k) / pathVisualizationInterpolationSteps;

            // Linear interpolation in joint space
            for(int j = 0; j < numJoints; ++j){
                q_interp[j] = q0[j] + ratio * (q1[j] - q0[j]);
                body->joint(j)->q() = q_interp[j];
            }

            // Calculate forward kinematics
            body->calcForwardKinematics();

            // Get end effector position
            Vector3f p = kinematicsKit->globalEndPosition().translation().cast<float>();
            vertices.push_back(p);

            // Save waypoint (only at state i, not interpolated points)
            if(k == 0){
                waypoints.push_back(p);
            }

            // Add line from previous vertex
            if(vertexIndex >= 1){
                lineSet->addLine(vertexIndex - 1, vertexIndex);
            }

            ++vertexIndex;
        }
    }

    // Add the final state
    if(!restoreBodyStateOfSolutionPathState(numStates - 1)){
        return false;
    }
    Vector3f p = kinematicsKit->globalEndPosition().translation().cast<float>();
    waypoints.push_back(p);
    vertices.push_back(p);
    if(vertexIndex >= 1){
        lineSet->addLine(vertexIndex - 1, vertexIndex);
    }

    return true;
}


void BodyMotionPlanning::Impl::createPointVisualization
(SgPointSet* pointSet, const std::vector<Vector3f>& waypoints)
{
    auto& vertices = *pointSet->getOrCreateVertices();

    for(const auto& point : waypoints){
        vertices.push_back(point);
    }
}
