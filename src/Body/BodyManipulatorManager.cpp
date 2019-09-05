#include "BodyManipulatorManager.h"
#include "ManipulatorFrame.h"
#include "ManipulatorPosition.h"
#include <cnoid/Body>
#include <cnoid/JointPath>
#include <cnoid/JointPathConfigurationHandler>
#include "fmt/format.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

class BodyManipulatorManagerImpl
{
public:
    BodyPtr body;
    shared_ptr<JointPath> jointPath;
    shared_ptr<JointPathConfigurationHandler> jointPathConfigurationHandler;
    ManipulatorFrameSetPtr frameSet;

    BodyManipulatorManagerImpl();
    static bool complementManipulatorPath(Body* body, Link*& io_base, Link*& io_end);
    bool setManipulator(Body* body, Link* base, Link* end);
};

}


BodyManipulatorManager* BodyManipulatorManager::getOrCreateManager
(Body* body, Link* base, Link* end)
{
    if(!BodyManipulatorManagerImpl::complementManipulatorPath(body, base, end)){
        return nullptr;
    }

    BodyManipulatorManagerPtr manager;
    auto cacheName = format("BodyManipulatorManager_{}_{}", base->name(), end->name());
    manager = body->findCache<BodyManipulatorManager>(cacheName);
    if(!manager){
        manager = new BodyManipulatorManager;
        if(manager->impl->setManipulator(body, base, end)){
            body->setCache(cacheName, manager);
        } else {
            manager.reset();
        }
    }

    return manager;
}


bool BodyManipulatorManagerImpl::complementManipulatorPath(Body* body, Link*& io_base, Link*& io_end)
{
    if(!io_base){
        io_base = body->rootLink();
    }
    if(!io_end){
        io_end = body->findUniqueEndLink();
    }
    return (io_base && io_end);
}


BodyManipulatorManager::BodyManipulatorManager()
{
    impl = new BodyManipulatorManagerImpl;
}


BodyManipulatorManagerImpl::BodyManipulatorManagerImpl()
{
    frameSet = new ManipulatorFrameSet;
}


BodyManipulatorManager::~BodyManipulatorManager()
{
    delete impl;
}


bool BodyManipulatorManagerImpl::setManipulator(Body* body, Link* base, Link* end)
{
    bool isValid = false;

    if(base && end){
        jointPath = JointPath::getCustomPath(body, base, end);
        if(jointPath && jointPath->hasCustomIK()){
            jointPathConfigurationHandler =
                dynamic_pointer_cast<JointPathConfigurationHandler>(jointPath);
            if(jointPath->numJoints() <= ManipulatorPosition::MaxNumJoints){
                isValid = true;
            }
        }
    }

    if(isValid){
        this->body = body;
    } else {
        this->body.reset();
        jointPath.reset();
        jointPathConfigurationHandler.reset();
    }

    return isValid;
}
        
        
Body* BodyManipulatorManager::body()
{
    return impl->body;
}


std::shared_ptr<JointPath> BodyManipulatorManager::jointPath()
{
    return impl->jointPath;
}
    

std::shared_ptr<JointPathConfigurationHandler> BodyManipulatorManager::jointPathConfigurationHandler()
{
    return impl->jointPathConfigurationHandler;
}


void BodyManipulatorManager::setFrameSet(ManipulatorFrameSet* frameSet)
{
    impl->frameSet = frameSet;
}


ManipulatorFrameSet* BodyManipulatorManager::frameSet()
{
    return impl->frameSet;
}


int BodyManipulatorManager::currentConfiguration() const
{
    if(impl->jointPathConfigurationHandler){
        impl->jointPathConfigurationHandler->getCurrentConfiguration();
    }
    return 0;
}


std::string BodyManipulatorManager::configurationName(int index) const
{
    if(impl->jointPathConfigurationHandler){
        return impl->jointPathConfigurationHandler->getConfigurationName(index);
    }
    return std::string();
}
  
