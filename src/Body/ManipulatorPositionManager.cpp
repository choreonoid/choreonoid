#include "ManipulatorPositionManager.h"
#include "ManipulatorPosition.h"
#include "ManipulatorFrame.h"
#include <cnoid/Body>
#include <cnoid/JointPath>
#include <cnoid/JointPathConfigurationHandler>

using namespace std;
using namespace cnoid;

namespace cnoid {

class ManipulatorPositionManagerImpl
{
public:
    BodyPtr body;
    shared_ptr<JointPath> jointPath;
    shared_ptr<JointPathConfigurationHandler> jointPathConfigurationHandler;
    ManipulatorFrameSetPtr frameSet;
    ManipulatorPositionSetPtr positions;

    ManipulatorPositionManagerImpl();
    bool setManipulator(Body* manipulator);
    bool setManipulator(Body* body, Link* base, Link* end);
};

}

ManipulatorPositionManager::ManipulatorPositionManager()
{
    impl = new ManipulatorPositionManagerImpl;
}


ManipulatorPositionManagerImpl::ManipulatorPositionManagerImpl()
{
    frameSet = new ManipulatorFrameSet;
    positions = new ManipulatorPositionSet;
}


ManipulatorPositionManager::~ManipulatorPositionManager()
{
    delete impl;
}


bool ManipulatorPositionManager::setManipulator(Body* body)
{
    return impl->setManipulator(body);
}


bool ManipulatorPositionManagerImpl::setManipulator(Body* body)
{
    auto endLink = body->findUniqueEndLink();
    if(endLink){
        return setManipulator(body, body->rootLink(), endLink);
    }
    return false;
}


bool ManipulatorPositionManager::setManipulator(Body* body, Link* base, Link* end)
{
    return impl->setManipulator(body, base, end);
}


bool ManipulatorPositionManagerImpl::setManipulator(Body* body, Link* base, Link* end)
{
    bool ready = false;

    this->body = body;
    jointPath = JointPath::getCustomPath(body, base, end);
    if(jointPath && jointPath->hasCustomIK()){
        jointPathConfigurationHandler =
            dynamic_pointer_cast<JointPathConfigurationHandler>(jointPath);
        if(jointPath->numJoints() <= ManipulatorPosition::MAX_NUM_JOINTS){
            ready = true;
        }
    }

    if(!ready){
        this->body.reset();
        jointPath.reset();
        jointPathConfigurationHandler.reset();
    }

    return ready;
}
        
        
Body* ManipulatorPositionManager::body()
{
    return impl->body;
}


std::shared_ptr<JointPath> ManipulatorPositionManager::jointPath()
{
    return impl->jointPath;
}
    

std::shared_ptr<JointPathConfigurationHandler> ManipulatorPositionManager::jointPathConfigurationHandler()
{
    return impl->jointPathConfigurationHandler;
}


void ManipulatorPositionManager::setFrameSet(ManipulatorFrameSet* frameSet)
{
    impl->frameSet = frameSet;
}


ManipulatorFrameSet* ManipulatorPositionManager::frameSet()
{
    return impl->frameSet;
}


int ManipulatorPositionManager::currentConfiguration() const
{
    if(impl->jointPathConfigurationHandler){
        impl->jointPathConfigurationHandler->getCurrentConfiguration();
    }
    return 0;
}


ManipulatorPositionSet* ManipulatorPositionManager::positionSet()
{
    return impl->positions;
}

