#ifndef CNOID_BODY_MANIPULATOR_POSITION_MANAGER_H
#define CNOID_BODY_MANIPULATOR_POSITION_MANAGER_H

#include <cnoid/EigenTypes>
#include <memory>
#include "exportdecl.h"

namespace cnoid {

class Body;
class Link;
class JointPath;
class JointPathConfigurationHandler;
class ManipulatorFrameSet;
class ManipulatorPosition;
class ManipulatorPositionSet;
class ManipulatorPositionManagerImpl;

class CNOID_EXPORT ManipulatorPositionManager
{
public:
    ManipulatorPositionManager();
    ManipulatorPositionManager(const ManipulatorPositionManager& org) = delete;
    ~ManipulatorPositionManager();

    bool setManipulator(Body* body);
    bool setManipulator(Body* body, Link* base, Link* end);

    Body* body();
    std::shared_ptr<JointPath> jointPath();
    std::shared_ptr<JointPathConfigurationHandler> jointPathConfigurationHandler();

    void setFrameSet(ManipulatorFrameSet* frameSet);
    ManipulatorFrameSet* frameSet();

    int currentConfiguration() const;

    ManipulatorPositionSet* positionSet();

private:
    ManipulatorPositionManagerImpl* impl;
};

}

#endif
