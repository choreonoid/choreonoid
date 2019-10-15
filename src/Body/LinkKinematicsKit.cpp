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
    CoordinateFrameSetPtr baseFrames;
    CoordinateFrameSetPtr localFrames;
    GeneralId currentBaseFrameId;
    GeneralId currentLocalFrameId;
    
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
    : link(link)
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


void LinkKinematicsKit::setFrameSetPair(CoordinateFrameSetPair* frameSets)
{
    impl->setFrameSetPair(frameSets);
}


void LinkKinematicsKit::Impl::setFrameSetPair(CoordinateFrameSetPair* frameSets)
{
    this->frameSetPair = frameSets;
    baseFrames = frameSets->baseFrames();
    localFrames = frameSets->localFrames();
}


CoordinateFrameSet* LinkKinematicsKit::baseFrames()
{
    return impl->baseFrames;    
}


CoordinateFrame* LinkKinematicsKit::baseFrame(const GeneralId& id)
{
    return impl->baseFrames->getFrame(id);
}


const GeneralId& LinkKinematicsKit::currentBaseFrameId()
{
    return impl->currentBaseFrameId;
}


CoordinateFrame* LinkKinematicsKit::currentBaseFrame()
{
    return impl->baseFrames->getFrame(impl->currentBaseFrameId);
}


void LinkKinematicsKit::setCurrentBaseFrame(const GeneralId& id)
{
    impl->currentBaseFrameId = id;
}


CoordinateFrameSet* LinkKinematicsKit::localFrames()
{
    return impl->localFrames;
}


CoordinateFrame* LinkKinematicsKit::localFrame(const GeneralId& id)
{
    return impl->localFrames->getFrame(id);
}


const GeneralId& LinkKinematicsKit::currentLocalFrameId()
{
    return impl->currentLocalFrameId;
}


CoordinateFrame* LinkKinematicsKit::currentLocalFrame()
{
    return impl->localFrames->getFrame(impl->currentLocalFrameId);
}


void LinkKinematicsKit::setCurrentLocalFrame(const GeneralId& id)
{
    impl->currentLocalFrameId = id;
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
