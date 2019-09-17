#ifndef CNOID_MANIPULATOR_PLUGIN_BODY_MANIPULATOR_MANAGER_H
#define CNOID_MANIPULATOR_PLUGIN_BODY_MANIPULATOR_MANAGER_H

#include <cnoid/Referenced>
#include <cnoid/EigenTypes>
#include <memory>
#include "exportdecl.h"

namespace cnoid {

class Body;
class Link;
class JointPath;
class JointPathConfigurationHandler;
class ManipulatorFrameSet;

class CNOID_EXPORT BodyManipulatorManager : public Referenced
{
public:
    static BodyManipulatorManager* getOrCreateManager(Body* body, Link* base = nullptr, Link* end = nullptr);
    
    ~BodyManipulatorManager();

    BodyManipulatorManager* clone(); // Deep copy

    Body* body();
    std::shared_ptr<JointPath> jointPath();
    std::shared_ptr<JointPathConfigurationHandler> jointPathConfigurationHandler();

    Body* findAttachedEndEffector(Body* manipulatorBody);

    void setFrameSet(ManipulatorFrameSet* frameSet);
    ManipulatorFrameSet* frameSet();

    int currentConfiguration() const;
    std::string configurationName(int index) const;

protected:
    BodyManipulatorManager();
    BodyManipulatorManager(const BodyManipulatorManager& org) = delete;

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<BodyManipulatorManager> BodyManipulatorManagerPtr;

}

#endif
