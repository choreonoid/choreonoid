#include "LinkKinematicsKit.h"
#include "Body.h"
#include "JointPath.h"
#include "JointSpaceConfigurationHandler.h"
#include "CompositeBodyIK.h"
#include "HolderDevice.h"
#include "AttachmentDevice.h"
#include <cnoid/CoordinateFrameList>
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
    
    CoordinateFrameListPtr baseFrames;
    CoordinateFrameListPtr offsetFrames;
    GeneralId currentBaseFrameId;
    GeneralId currentOffsetFrameId;
    CoordinateFramePtr defaultBaseFrame;
    CoordinateFramePtr defaultOffsetFrame;

    Signal<void()> sigFrameUpdate;
    Signal<void(const Position& T_frameCoordinate)> sigPositionError;
    
    Impl(Link* link);
    Impl(const Impl& org, CloneMap* cloneMap);
    void setBaseLink(Link* link);
    void setInversetKinematics(std::shared_ptr<InverseKinematics> ik);
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
      currentBaseFrameId(0),
      currentOffsetFrameId(0)
{
    if(link){
        body = link->body();
    }
    defaultBaseFrame = new CoordinateFrame;
    defaultOffsetFrame = new CoordinateFrame;
}


LinkKinematicsKit::LinkKinematicsKit(const LinkKinematicsKit& org, CloneMap* cloneMap)
{
    impl = new Impl(*org.impl, cloneMap);
}


LinkKinematicsKit::Impl::Impl(const Impl& org, CloneMap* cloneMap)
{
    isCustomIkDisabled = org.isCustomIkDisabled;
    referenceRpy.setZero();

    currentBaseFrameId = org.currentBaseFrameId;
    currentOffsetFrameId = org.currentOffsetFrameId;
    
    if(cloneMap){
        body = cloneMap->getClone(org.body);
        if(body){
            link = body->link(org.link->index());
            if(auto orgBaseLink = const_cast<Impl&>(org).baseLink()){
                auto baseLink = body->link(orgBaseLink->index());
                setBaseLink(baseLink);
            }
        }
        baseFrames = cloneMap->getClone(org.baseFrames);
        offsetFrames = cloneMap->getClone(org.offsetFrames);
    }

    defaultBaseFrame = new CoordinateFrame;
    defaultOffsetFrame = new CoordinateFrame;
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


void LinkKinematicsKit::setBaseFrames(CoordinateFrameList* frames)
{
    impl->baseFrames = frames;
}


void LinkKinematicsKit::setOffsetFrames(CoordinateFrameList* frames)
{
    impl->offsetFrames = frames;
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


CoordinateFrameList* LinkKinematicsKit::baseFrames()
{
    return impl->baseFrames;
}


CoordinateFrameList* LinkKinematicsKit::offsetFrames()
{
    return impl->offsetFrames;
}


CoordinateFrame* LinkKinematicsKit::baseFrame(const GeneralId& id)
{
    if(impl->baseFrames){
        if(auto frame = impl->baseFrames->findFrame(id)){
            return frame;
        }
    }
    return impl->defaultBaseFrame;
}


CoordinateFrame* LinkKinematicsKit::offsetFrame(const GeneralId& id)
{
    if(impl->offsetFrames){
        if(auto frame = impl->offsetFrames->findFrame(id)){
            return frame;
        }
    }
    return impl->defaultOffsetFrame;

}


const GeneralId& LinkKinematicsKit::currentBaseFrameId() const
{
    return impl->currentBaseFrameId;
}


const GeneralId& LinkKinematicsKit::currentOffsetFrameId() const
{
    return impl->currentOffsetFrameId;
}


CoordinateFrame* LinkKinematicsKit::currentBaseFrame()
{
    if(impl->baseFrames){
        if(auto frame = impl->baseFrames->findFrame(impl->currentBaseFrameId)){
            return frame;
        }
    }
    return impl->defaultBaseFrame;
}


CoordinateFrame* LinkKinematicsKit::currentOffsetFrame()
{
    if(impl->offsetFrames){
        if(auto frame = impl->offsetFrames->findFrame(impl->currentOffsetFrameId)){
            return frame;
        }
    }
    return impl->defaultOffsetFrame;
}


void LinkKinematicsKit::setCurrentBaseFrame(const GeneralId& id)
{
    impl->currentBaseFrameId = id;
}


void LinkKinematicsKit::setCurrentOffsetFrame(const GeneralId& id)
{
    impl->currentOffsetFrameId = id;
}


Position LinkKinematicsKit::globalBasePosition() const
{
    auto casted = const_cast<LinkKinematicsKit*>(this);
    auto baseFrame = casted->currentBaseFrame();
    if(baseFrame->isGlobal()){
        return baseFrame->T();
    } else {
        return casted->baseLink()->T() * baseFrame->T();
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
    auto baseId = impl->currentBaseFrameId;
    if(baseId.isValid()){
        archive.write("base_frame", baseId.label(),
                      baseId.isString() ? DOUBLE_QUOTED : PLAIN_STRING);
    }
    auto offsetId = impl->currentOffsetFrameId;
    if(offsetId.isValid()){
        archive.write("offset_frame", offsetId.label(),
                      offsetId.isString() ? DOUBLE_QUOTED : PLAIN_STRING);
    }
    if(impl->isCustomIkDisabled){
        archive.write("disable_custom_ik", true);
    }
    return true;
}


bool LinkKinematicsKit::restoreState(const Mapping& archive)
{
    bool updated = false;

    if(impl->currentBaseFrameId.read(archive, "base_frame")){
        updated = true;
    }
    if(impl->currentOffsetFrameId.read(archive, "offset_frame")){
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
