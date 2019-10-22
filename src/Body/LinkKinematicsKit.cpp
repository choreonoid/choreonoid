#include "LinkKinematicsKit.h"
#include "Body.h"
#include "JointPath.h"
#include "JointPathConfigurationHandler.h"
#include "CompositeBodyIK.h"
#include "HolderDevice.h"
#include "AttachmentDevice.h"
#include <cnoid/CoordinateFrameSet>

using namespace std;
using namespace cnoid;

namespace {

enum FrameType { Base, Local };

}

namespace cnoid {

class LinkKinematicsKit::Impl
{
public:
    BodyPtr body;
    LinkPtr link;
    shared_ptr<InverseKinematics> inverseKinematics;
    shared_ptr<JointPath> jointPath;
    shared_ptr<JointPathConfigurationHandler> configurationHandler;
    CoordinateFrameSetPairPtr frameSetPair;
    GeneralId currentFrameId[2];
    Signal<void()> sigCurrentFrameChanged;
    
    Impl(Link* link);
    void setBaseLink(Link* link);
    void setInversetKinematics(std::shared_ptr<InverseKinematics> ik);
    void setFrameSetPair(CoordinateFrameSetPair* frameSets);
};

}


LinkKinematicsKit::LinkKinematicsKit(Link* link)
{
    impl = new Impl(link);
}


LinkKinematicsKit::Impl::Impl(Link* link)
    : link(link),
      currentFrameId{ 0, 0 }
{
    if(link){
        body = link->body();
    }
    setFrameSetPair(new CoordinateFrameSetPair);
}
    

LinkKinematicsKit::~LinkKinematicsKit()
{
    delete impl;
}


void LinkKinematicsKit::setBaseLink(Link* baseLink)
{
    impl->setBaseLink(baseLink);
}


void LinkKinematicsKit::Impl::setBaseLink(Link* baseLink)
{
    jointPath.reset();
    inverseKinematics.reset();
    configurationHandler.reset();

    if(baseLink && baseLink->body() == body){
        jointPath = JointPath::getCustomPath(body, baseLink, link);
        if(jointPath){
            inverseKinematics = jointPath;
            if(jointPath->hasCustomIK()){
                configurationHandler =
                    dynamic_pointer_cast<JointPathConfigurationHandler>(jointPath);
            }
        }
    }
}


void LinkKinematicsKit::setInversetKinematics(std::shared_ptr<InverseKinematics> ik)
{
    impl->setInversetKinematics(ik);
}


void LinkKinematicsKit::Impl::setInversetKinematics(std::shared_ptr<InverseKinematics> ik)
{
    jointPath.reset();
    inverseKinematics.reset();
    configurationHandler.reset();

    inverseKinematics = ik;

    if(auto compositeBodyIK = dynamic_pointer_cast<CompositeBodyIK>(ik)){
        configurationHandler =
            dynamic_pointer_cast<JointPathConfigurationHandler>(
                compositeBodyIK->getParentBodyIK());
    } else {
        configurationHandler = dynamic_pointer_cast<JointPathConfigurationHandler>(ik);
    }

    jointPath = dynamic_pointer_cast<JointPath>(ik);
}


void LinkKinematicsKit::setFrameSetPair(CoordinateFrameSetPair* frameSets)
{
    impl->setFrameSetPair(frameSets);
}


void LinkKinematicsKit::Impl::setFrameSetPair(CoordinateFrameSetPair* frameSets)
{
    this->frameSetPair = frameSets;
}


Body* LinkKinematicsKit::body()
{
    return impl->body;
}


Link* LinkKinematicsKit::link()
{
    return impl->link;
}


Link* LinkKinematicsKit::baseLink()
{
    if(impl->jointPath){
        return impl->jointPath->baseLink();
    }
    return nullptr;
}


std::shared_ptr<InverseKinematics> LinkKinematicsKit::inverseKinematics()
{
    return impl->inverseKinematics;
}


std::shared_ptr<JointPath> LinkKinematicsKit::jointPath()
{
    return impl->jointPath;
}


std::shared_ptr<JointPathConfigurationHandler> LinkKinematicsKit::configurationHandler()
{
    return impl->configurationHandler;
}


int LinkKinematicsKit::currentConfiguration() const
{
    if(impl->configurationHandler){
        impl->configurationHandler->getCurrentConfiguration();
    }
    return 0;
}


std::string LinkKinematicsKit::configurationName(int index) const
{
    if(impl->configurationHandler){
        return impl->configurationHandler->getConfigurationName(index);
    }
    return std::string();
}


bool LinkKinematicsKit::isManipulator() const
{
    if(impl->body && impl->link &&
       impl->link == impl->body->findUniqueEndLink() &&
       impl->configurationHandler){
        return true;
    }
    return false;
}


CoordinateFrameSetPair* LinkKinematicsKit::frameSetPair()
{
    return impl->frameSetPair;
}


CoordinateFrameSet* LinkKinematicsKit::frameSet(int which)
{
    return impl->frameSetPair->frameSet(which);
}


CoordinateFrameSet* LinkKinematicsKit::baseFrameSet()
{
    return impl->frameSetPair->baseFrameSet();
}


CoordinateFrameSet* LinkKinematicsKit::localFrameSet()
{
    return impl->frameSetPair->localFrameSet();
}


CoordinateFrame* LinkKinematicsKit::baseFrame(const GeneralId& id)
{
    return impl->frameSetPair->baseFrameSet()->getFrame(id);
}


CoordinateFrame* LinkKinematicsKit::localFrame(const GeneralId& id)
{
    return impl->frameSetPair->localFrameSet()->getFrame(id);
}


const GeneralId& LinkKinematicsKit::currentFrameId(int which)
{
    return impl->currentFrameId[which];
}


const GeneralId& LinkKinematicsKit::currentBaseFrameId()
{
    return impl->currentFrameId[Base];
}


const GeneralId& LinkKinematicsKit::currentLocalFrameId()
{
    return impl->currentFrameId[Local];
}


CoordinateFrame* LinkKinematicsKit::currentBaseFrame()
{
    return impl->frameSetPair->baseFrameSet()->getFrame(impl->currentFrameId[Base]);
}


CoordinateFrame* LinkKinematicsKit::currentLocalFrame()
{
    return impl->frameSetPair->localFrameSet()->getFrame(impl->currentFrameId[Local]);
}


void LinkKinematicsKit::setCurrentFrame(int which, const GeneralId& id)
{
    impl->currentFrameId[which] = id;
}


void LinkKinematicsKit::setCurrentBaseFrame(const GeneralId& id)
{
    impl->currentFrameId[Base] = id;
}


void LinkKinematicsKit::setCurrentLocalFrame(const GeneralId& id)
{
    impl->currentFrameId[Local] = id;
}


SignalProxy<void()> LinkKinematicsKit::sigCurrentFrameChanged()
{
    return impl->sigCurrentFrameChanged;
}


void LinkKinematicsKit::notifyCurrentFrameChange()
{
    impl->sigCurrentFrameChanged();
}


Body* LinkKinematicsKit::findAttachedEndEffector() const
{
    if(auto body = impl->body){
        for(auto& holder : body->devices<HolderDevice>()){
            if(holder->category() == "EndEffector"){
                if(auto attachment = holder->attachment()){
                    return attachment->link()->body();
                }
            }
        }
    }
    return nullptr;
}
