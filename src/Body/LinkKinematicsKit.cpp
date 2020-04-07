#include "LinkKinematicsKit.h"
#include "Body.h"
#include "JointPath.h"
#include "JointSpaceConfigurationHandler.h"
#include "CompositeBodyIK.h"
#include "HolderDevice.h"
#include "AttachmentDevice.h"
#include <cnoid/LinkCoordFrameSetSuite>
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
    shared_ptr<JointSpaceConfigurationHandler> configurationHandler;
    bool isCustomIkDisabled;
    bool isRpySpecified;
    Vector3 referenceRpy;
    LinkCoordFrameSetSuitePtr frameSetSuite;
    GeneralId defaultFrameId;
    CoordinateFramePtr defaultCoordinateFrame;
    GeneralId currentFrameId[3];
    int currentBaseFrameType;
    Signal<void()> sigFrameUpdate;
    Signal<void(const Position& T_frameCoordinate)> sigPositionError;
    
    Impl(Link* link);
    Impl(const Impl& org, CloneMap* cloneMap);
    void setBaseLink(Link* link);
    void setInversetKinematics(std::shared_ptr<InverseKinematics> ik);
    void setFrameSetSuite(LinkCoordFrameSetSuite* frameSetSuite);
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
      defaultFrameId(0),
      currentFrameId{ 0, 0, 0 },
      currentBaseFrameType(WorldFrame)
{
    if(link){
        body = link->body();
    }
    defaultCoordinateFrame = new CoordinateFrame;
}


LinkKinematicsKit::LinkKinematicsKit(const LinkKinematicsKit& org, CloneMap* cloneMap)
{
    impl = new Impl(*org.impl, cloneMap);
}


LinkKinematicsKit::Impl::Impl(const Impl& org, CloneMap* cloneMap)
    : defaultFrameId(0)
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
        setFrameSetSuite(cloneMap->getClone(org.frameSetSuite));
    }

    defaultCoordinateFrame = new CoordinateFrame;
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
                    dynamic_pointer_cast<JointSpaceConfigurationHandler>(jointPath);
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
            dynamic_pointer_cast<JointSpaceConfigurationHandler>(
                compositeBodyIK->getParentBodyIK());
    } else {
        configurationHandler = dynamic_pointer_cast<JointSpaceConfigurationHandler>(ik);
    }

    jointPath = dynamic_pointer_cast<JointPath>(ik);
    if(jointPath){
        jointPath->setNumericalIKenabled(isCustomIkDisabled);
    }
}


void LinkKinematicsKit::setFrameSetSuite(LinkCoordFrameSetSuite* frameSetSuite)
{
    impl->setFrameSetSuite(frameSetSuite);
}


void LinkKinematicsKit::Impl::setFrameSetSuite(LinkCoordFrameSetSuite* frameSetSuite)
{
    this->frameSetSuite = frameSetSuite;
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


std::shared_ptr<JointSpaceConfigurationHandler> LinkKinematicsKit::configurationHandler()
{
    return impl->configurationHandler;
}


int LinkKinematicsKit::currentConfigurationType() const
{
    if(impl->configurationHandler){
        return impl->configurationHandler->getCurrentConfigurationType();
    }
    return 0;
}


std::string LinkKinematicsKit::configurationLabel(int id) const
{
    if(impl->configurationHandler){
        string label;
        for(auto& element : impl->configurationHandler->getConfigurationStateNames(id)){
            if(!label.empty()){
                label.append("-");
            }
            label.append(element);
        }
        return label;
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


LinkCoordFrameSetSuite* LinkKinematicsKit::frameSetSuite()
{
    return impl->frameSetSuite;
}


CoordinateFrameSet* LinkKinematicsKit::frameSet(int frameType)
{
    if(impl->frameSetSuite){
        return impl->frameSetSuite->frameSet(frameType);
    }
    return nullptr;
}


CoordinateFrameSet* LinkKinematicsKit::worldFrameSet()
{
    if(impl->frameSetSuite){
        return impl->frameSetSuite->worldFrameSet();
    }
    return nullptr;
}


CoordinateFrameSet* LinkKinematicsKit::bodyFrameSet()
{
    if(impl->frameSetSuite){
        return impl->frameSetSuite->bodyFrameSet();
    }
    return nullptr;
}


CoordinateFrameSet* LinkKinematicsKit::linkFrameSet()
{
    if(impl->frameSetSuite){
        return impl->frameSetSuite->linkFrameSet();
    }
    return nullptr;
}


CoordinateFrame* LinkKinematicsKit::worldFrame(const GeneralId& id)
{
    if(impl->frameSetSuite){
        return impl->frameSetSuite->worldFrame(id);
    }
    return impl->defaultCoordinateFrame;
}


CoordinateFrame* LinkKinematicsKit::bodyFrame(const GeneralId& id)
{
    if(impl->frameSetSuite){
        return impl->frameSetSuite->bodyFrame(id);
    }
    return impl->defaultCoordinateFrame;
}


CoordinateFrame* LinkKinematicsKit::linkFrame(const GeneralId& id)
{
    if(impl->frameSetSuite){
        return impl->frameSetSuite->linkFrame(id);
    }
    return impl->defaultCoordinateFrame;
}


const GeneralId& LinkKinematicsKit::currentFrameId(int frameType) const
{
    if(impl->frameSetSuite){
        return impl->currentFrameId[frameType];
    }
    return impl->defaultFrameId;
}


const GeneralId& LinkKinematicsKit::currentWorldFrameId() const
{
    if(impl->frameSetSuite){
        return impl->currentFrameId[WorldFrame];
    }
    return impl->defaultFrameId;
}


const GeneralId& LinkKinematicsKit::currentBodyFrameId() const
{
    if(impl->frameSetSuite){
        return impl->currentFrameId[BodyFrame];
    }
    return impl->defaultFrameId;
}


const GeneralId& LinkKinematicsKit::currentLinkFrameId() const
{
    if(impl->frameSetSuite){
        return impl->currentFrameId[LinkFrame];
    }
    return impl->defaultFrameId;
}


CoordinateFrame* LinkKinematicsKit::currentFrame(int frameType)
{
    if(impl->frameSetSuite){
        return impl->frameSetSuite->frameSet(frameType)->getFrame(currentFrameId(frameType));
    }
    return impl->defaultCoordinateFrame;
}


CoordinateFrame* LinkKinematicsKit::currentWorldFrame()
{
    if(impl->frameSetSuite){
        return impl->frameSetSuite->worldFrameSet()->getFrame(currentWorldFrameId());
    }
    return impl->defaultCoordinateFrame;
}


CoordinateFrame* LinkKinematicsKit::currentBodyFrame()
{
    if(impl->frameSetSuite){
        return impl->frameSetSuite->bodyFrameSet()->getFrame(currentBodyFrameId());
    }
    return impl->defaultCoordinateFrame;
}


CoordinateFrame* LinkKinematicsKit::currentLinkFrame()
{
    if(impl->frameSetSuite){
        return impl->frameSetSuite->linkFrameSet()->getFrame(currentLinkFrameId());
    }
    return impl->defaultCoordinateFrame;
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


void LinkKinematicsKit::setCurrentLinkFrame(const GeneralId& id)
{
    impl->currentFrameId[LinkFrame] = id;
}


int LinkKinematicsKit::currentBaseFrameType() const
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


Position LinkKinematicsKit::globalBasePosition() const
{
    auto self = const_cast<LinkKinematicsKit*>(this);
    
    if(impl->currentBaseFrameType == WorldFrame){
        return self->currentWorldFrame()->T();
    } else {
        return self->baseLink()->Ta() * self->currentBodyFrame()->T();
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


SignalProxy<void(const Position& T_frameCoordinate)> LinkKinematicsKit::sigPositionError()
{
    return impl->sigPositionError;
}


void LinkKinematicsKit::notifyPositionError(const Position& T_frameCoordinate)
{
    impl->sigPositionError(T_frameCoordinate);
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
    auto& linkId = currentLinkFrameId();
    if(linkId != defaultId){
        archive.write("link_frame", linkId.label(),
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
        setCurrentLinkFrame(id);
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
