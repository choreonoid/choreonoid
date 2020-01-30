#include "LinkKinematicsKit.h"
#include "Body.h"
#include "JointPath.h"
#include "JointPathConfigurationHandler.h"
#include "CompositeBodyIK.h"
#include "HolderDevice.h"
#include "AttachmentDevice.h"
#include <cnoid/LinkCoordinateFrameSet>

using namespace std;
using namespace cnoid;

namespace cnoid {

class LinkKinematicsKit::Impl
{
public:
    BodyPtr body;
    LinkPtr link;
    Vector3 referenceRpy;
    bool isRpySpecified;
    shared_ptr<InverseKinematics> inverseKinematics;
    shared_ptr<JointPath> jointPath;
    shared_ptr<JointPathConfigurationHandler> configurationHandler;
    LinkCoordinateFrameSetPtr frameSets;
    GeneralId currentFrameId[3];
    int currentBaseFrameType;
    Signal<void()> sigCurrentFrameChanged;
    
    Impl(Link* link);
    void setBaseLink(Link* link);
    void setInversetKinematics(std::shared_ptr<InverseKinematics> ik);
    void setFrameSets(LinkCoordinateFrameSet* frameSets);
};

}


LinkKinematicsKit::LinkKinematicsKit(Link* link)
{
    impl = new Impl(link);
}


LinkKinematicsKit::Impl::Impl(Link* link)
    : link(link),
      referenceRpy(Vector3::Zero()),
      currentFrameId{ 0, 0, 0 },
      currentBaseFrameType(WorldFrame)
{
    if(link){
        body = link->body();
    }
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


void LinkKinematicsKit::setFrameSets(LinkCoordinateFrameSet* frameSets)
{
    impl->setFrameSets(frameSets);
}


void LinkKinematicsKit::Impl::setFrameSets(LinkCoordinateFrameSet* frameSets)
{
    this->frameSets = frameSets;
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


Vector3 LinkKinematicsKit::referenceRpy() const
{
    return impl->referenceRpy;
}


void LinkKinematicsKit::setReferenceRpy(const Vector3& rpy)
{
    impl->referenceRpy = rpy;
}


void LinkKinematicsKit::resetReferenceRpy()
{
    impl->referenceRpy.setZero();
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


LinkCoordinateFrameSet* LinkKinematicsKit::frameSets()
{
    return impl->frameSets;
}


CoordinateFrameSet* LinkKinematicsKit::frameSet(int frameType)
{
    return impl->frameSets->frameSet(frameType);
}


CoordinateFrameSet* LinkKinematicsKit::worldFrameSet()
{
    return impl->frameSets->worldFrameSet();
}


CoordinateFrameSet* LinkKinematicsKit::bodyFrameSet()
{
    return impl->frameSets->bodyFrameSet();
}


CoordinateFrameSet* LinkKinematicsKit::endFrameSet()
{
    return impl->frameSets->endFrameSet();
}


CoordinateFrame* LinkKinematicsKit::worldFrame(const GeneralId& id)
{
    return impl->frameSets->worldFrame(id);
}


CoordinateFrame* LinkKinematicsKit::bodyFrame(const GeneralId& id)
{
    return impl->frameSets->bodyFrame(id);
}


CoordinateFrame* LinkKinematicsKit::endFrame(const GeneralId& id)
{
    return impl->frameSets->endFrame(id);
}


const GeneralId& LinkKinematicsKit::currentFrameId(int frameType)
{
    return impl->currentFrameId[frameType];
}


const GeneralId& LinkKinematicsKit::currentWorldFrameId()
{
    return impl->currentFrameId[WorldFrame];
}


const GeneralId& LinkKinematicsKit::currentBodyFrameId()
{
    return impl->currentFrameId[BodyFrame];
}


const GeneralId& LinkKinematicsKit::currentEndFrameId()
{
    return impl->currentFrameId[EndFrame];
}


CoordinateFrame* LinkKinematicsKit::currentFrame(int frameType)
{
    return impl->frameSets->frameSet(frameType)->getFrame(currentFrameId(frameType));
}


CoordinateFrame* LinkKinematicsKit::currentWorldFrame()
{
    return impl->frameSets->worldFrameSet()->getFrame(currentWorldFrameId());
}


CoordinateFrame* LinkKinematicsKit::currentBodyFrame()
{
    return impl->frameSets->bodyFrameSet()->getFrame(currentBodyFrameId());
}


CoordinateFrame* LinkKinematicsKit::currentEndFrame()
{
    return impl->frameSets->endFrameSet()->getFrame(currentEndFrameId());
}


void LinkKinematicsKit::setCurrentFrame(int frameType, const GeneralId& id)
{
    impl->currentFrameId[frameType] = id;
}


void LinkKinematicsKit::setCurrentWorldFrame(const GeneralId& id)
{
    impl->currentFrameId[WorldFrame] = id;
}


void LinkKinematicsKit::setCurrentBodyFrame(const GeneralId& id)
{
    impl->currentFrameId[BodyFrame] = id;
}


void LinkKinematicsKit::setCurrentEndFrame(const GeneralId& id)
{
    impl->currentFrameId[EndFrame] = id;
}


int LinkKinematicsKit::currentBaseFrameType()
{
    return impl->currentBaseFrameType;
}


void LinkKinematicsKit::setCurrentBaseFrameType(int frameType)
{
    impl->currentBaseFrameType = frameType;
}


const GeneralId& LinkKinematicsKit::currentBaseFrameId()
{
    if(impl->currentBaseFrameType == WorldFrame){
        return currentWorldFrameId();
    } else {
        return currentBodyFrameId();
    }
}


CoordinateFrame* LinkKinematicsKit::currentBaseFrame()
{
    if(impl->currentBaseFrameType == WorldFrame){
        return currentWorldFrame();
    } else {
        return currentBodyFrame();
    }
}


void LinkKinematicsKit::setCurrentBaseFrame(const GeneralId& id)
{
    if(impl->currentBaseFrameType == WorldFrame){
        setCurrentWorldFrame(id);
    } else {
        setCurrentBodyFrame(id);
    }
}
    

SignalProxy<void()> LinkKinematicsKit::sigCurrentFrameChanged()
{
    return impl->sigCurrentFrameChanged;
}


void LinkKinematicsKit::notifyCurrentFrameChange()
{
    impl->sigCurrentFrameChanged();
}


/*
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
*/
