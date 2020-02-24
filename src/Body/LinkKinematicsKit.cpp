#include "LinkKinematicsKit.h"
#include "Body.h"
#include "JointPath.h"
#include "JointPathConfigurationHandler.h"
#include "CompositeBodyIK.h"
#include "HolderDevice.h"
#include "AttachmentDevice.h"
#include <cnoid/LinkCoordinateFrameSet>
#include <cnoid/CloneMap>
#include <cnoid/ValueTree>

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
    bool isCustomIkDisabled;
    bool isRpySpecified;
    Vector3 referenceRpy;
    LinkCoordinateFrameSetPtr frameSets;
    GeneralId currentFrameId[3];
    int currentBaseFrameType;
    Signal<void()> sigFrameUpdate;
    
    Impl(Link* link);
    Impl(const Impl& org, CloneMap* cloneMap);
    void setBaseLink(Link* link);
    void setInversetKinematics(std::shared_ptr<InverseKinematics> ik);
    void setFrameSets(LinkCoordinateFrameSet* frameSets);
    Link* baseLink();
};

}


LinkKinematicsKit::LinkKinematicsKit(Link* link)
{
    impl = new Impl(link);
}


LinkKinematicsKit::Impl::Impl(Link* link)
    : link(link),
      isCustomIkDisabled(false),
      referenceRpy(Vector3::Zero()),
      currentFrameId{ 0, 0, 0 },
      currentBaseFrameType(WorldFrame)
{
    if(link){
        body = link->body();
    }
}


LinkKinematicsKit::LinkKinematicsKit(const LinkKinematicsKit& org, CloneMap* cloneMap)
{
    impl = new Impl(*org.impl, cloneMap);
}


LinkKinematicsKit::Impl::Impl(const Impl& org, CloneMap* cloneMap)
{
    isCustomIkDisabled = org.isCustomIkDisabled;
    referenceRpy.setZero();
    for(int i=0; i < 3; ++i){
        currentFrameId[i] = org.currentFrameId[i];
    }
    currentBaseFrameType = org.currentBaseFrameType;
    
    if(cloneMap){
        body = cloneMap->getClone(org.body);
        if(body){
            link = body->link(org.link->index());
            if(auto orgBaseLink = const_cast<Impl&>(org).baseLink()){
                auto baseLink = body->link(orgBaseLink->index());
                setBaseLink(baseLink);
            }
        }
        setFrameSets(cloneMap->getClone(org.frameSets));
    }
}


Referenced* LinkKinematicsKit::doClone(CloneMap* cloneMap) const
{
    return new LinkKinematicsKit(*this, cloneMap);
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
                jointPath->setNumericalIKenabled(isCustomIkDisabled);
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
    if(jointPath){
        jointPath->setNumericalIKenabled(isCustomIkDisabled);
    }
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


Link* LinkKinematicsKit::Impl::baseLink()
{
    if(jointPath){
        return jointPath->baseLink();
    }
    return nullptr;
}


Link* LinkKinematicsKit::baseLink()
{
    return impl->baseLink();
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


bool LinkKinematicsKit::isCustomIkAvaiable() const
{
    return (impl->jointPath && impl->jointPath->hasCustomIK());
}


bool LinkKinematicsKit::isCustomIkDisabled() const
{
    return impl->isCustomIkDisabled;
}


void LinkKinematicsKit::setCustomIkDisabled(bool on)
{
    if(on != impl->isCustomIkDisabled){
        if(impl->jointPath){
            impl->jointPath->setNumericalIKenabled(on);
        }
        impl->isCustomIkDisabled = on;
    }
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


const GeneralId& LinkKinematicsKit::currentFrameId(int frameType) const
{
    return impl->currentFrameId[frameType];
}


const GeneralId& LinkKinematicsKit::currentWorldFrameId() const
{
    return impl->currentFrameId[WorldFrame];
}


const GeneralId& LinkKinematicsKit::currentBodyFrameId() const
{
    return impl->currentFrameId[BodyFrame];
}


const GeneralId& LinkKinematicsKit::currentEndFrameId() const
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


const GeneralId& LinkKinematicsKit::currentBaseFrameId() const
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
    

SignalProxy<void()> LinkKinematicsKit::sigFrameUpdate()
{
    return impl->sigFrameUpdate;
}


void LinkKinematicsKit::notifyFrameUpdate()
{
    impl->sigFrameUpdate();
}


bool LinkKinematicsKit::storeState(Mapping& archive) const
{
    const auto defaultId = CoordinateFrame::defaultFrameId();
    auto& worldId = currentWorldFrameId();
    if(worldId != defaultId){
        archive.write("world_frame", worldId.label(),
                      worldId.isString() ? DOUBLE_QUOTED : PLAIN_STRING);
    }
    auto& bodyId = currentBodyFrameId();
    if(bodyId != defaultId){
        archive.write("body_frame", bodyId.label(),
                      bodyId.isString() ? DOUBLE_QUOTED : PLAIN_STRING);
    }
    auto& endId = currentEndFrameId();
    if(endId != defaultId){
        archive.write("link_frame", endId.label(),
                      bodyId.isString() ? DOUBLE_QUOTED : PLAIN_STRING);
    }
    if(impl->isCustomIkDisabled){
        archive.write("disable_custom_ik", true);
    }
    return true;
}


bool LinkKinematicsKit::restoreState(const Mapping& archive)
{
    bool updated = false;
    GeneralId id;
    if(id.read(archive, "world_frame")){
        setCurrentWorldFrame(id);
        updated = true;
    }
    if(id.read(archive, "body_frame")){
        setCurrentBodyFrame(id);
        updated = true;
    }
    if(id.read(archive, "link_frame")){
        setCurrentEndFrame(id);
        updated = true;
    }
    bool on;
    if(archive.read("disable_custom_ik", on)){
        setCustomIkDisabled(on);
    }
    if(updated){
        notifyFrameUpdate();
    }
    return true;
}
