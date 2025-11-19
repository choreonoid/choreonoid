#ifndef CNOID_OMPL_PLUGIN_BODY_MOTION_PLANNING_H
#define CNOID_OMPL_PLUGIN_BODY_MOTION_PLANNING_H

#include "exportdecl.h"
#include <cnoid/EigenTypes>
#include <functional>

namespace cnoid {

class Body;
class KinematicBodySet;
class SgNode;
class MessageOut;

class CNOID_EXPORT BodyMotionPlanning
{
public:
    static int numPlannerTypes();
    static const char* plannerTypeName(int index);

    BodyMotionPlanning();
    ~BodyMotionPlanning();

    void setMessageOut(MessageOut* mout);
    void setMessageOutputEnabled(bool on);
    std::string getLastErrorMessages() const;

    void setPlannerType(int index);
    void setStateValidityCheckingResolutionRatio(double ratio);
    void setJointSpaceConfigurationHandlerCheckEnabled(bool on);
    void setPathSimplificationEnabled(bool on);
    bool isPathSimplificationEnabled() const;
    void setPathShortcuttingEnabled(bool on);
    bool isPathShortcuttingEnabled() const;
    void setTimeoutSeconds(double seconds);
    double timeoutSeconds() const;

    // Set workspace bounds for constraining joint space planning
    void setWorkspaceBounds(const Vector3& lower, const Vector3& upper);
    void clearWorkspaceBounds();
    const Vector3& workspaceLowerBound() const;
    const Vector3& workspaceUpperBound() const;
    SgNode* getWorkspaceBoundVisualizationNode();

    void clear();
    void setTargetBodySet(KinematicBodySet* bodySet);
    KinematicBodySet* targetBodySet();
    void setTargetBody(Body* body);
    void addAttachedObject(Body* body);
    void addEnvironmentalObject(Body* body);
    void addCurrentBodyStateAsWaypoint();

    enum Result { Invalid, Started, Solved, NotSolved, Timeout, Terminated };
    Result solve();
    // \note The onFinished callback is called from the thread of motion planning
    Result solveAsynchronously(std::function<void(Result result)> onFinished);
    void terminate();

    int numSolutionPathStates() const;
    bool restoreBodyStateOfSolutionPathState(int index);

    enum PathVisualizationMode {
        DirectLine,              // Connect waypoints with straight lines in Cartesian space
        JointSpaceInterpolation  // Interpolate in joint space
    };
    void setPathVisualizationMode(PathVisualizationMode mode);
    PathVisualizationMode pathVisualizationMode() const;
    void setPathVisualizationInterpolationSteps(int steps);
    int pathVisualizationInterpolationSteps() const;
    SgNode* getPathVisualizationNode();

protected:
    MessageOut* messageOut();

private:
    class Impl;
    Impl* impl;
};

}

#endif
