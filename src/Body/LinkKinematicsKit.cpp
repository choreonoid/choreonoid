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
    shared_ptr<JointPath> jointPath;
    shared_ptr<InverseKinematics> inverseKinematics;
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
    Signal<void(const Isometry3& T_frameCoordinate)> sigPositionError;
    
    Impl(Link* link);
    Impl(const Impl& org, CloneMap* cloneMap);
    void setBaseLink(Link* link);
    void setInverseKinematics(std::shared_ptr<InverseKinematics> ik);
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
                jointPath->setCustomIkDisabled(isCustomIkDisabled);
            }
        }
    }
}


void LinkKinematicsKit::setInverseKinematics(std::shared_ptr<InverseKinematics> ik)
{
    impl->setInverseKinematics(ik);
}


void LinkKinematicsKit::Impl::setInverseKinematics(std::shared_ptr<InverseKinematics> ik)
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
        jointPath->setCustomIkDisabled(isCustomIkDisabled);
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


const Body* LinkKinematicsKit::body() const
{
    return impl->body;
}


Link* LinkKinematicsKit::link()
{
    return impl->link;
}


const Link* LinkKinematicsKit::link() const
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


const Link* LinkKinematicsKit::baseLink() const
{
    return const_cast<LinkKinematicsKit::Impl*>(impl)->baseLink();
}


bool LinkKinematicsKit::hasJointPath() const
{
    return (bool)impl->jointPath;
}


std::shared_ptr<JointPath> LinkKinematicsKit::jointPath()
{
    return impl->jointPath;
}


std::shared_ptr<InverseKinematics> LinkKinematicsKit::inverseKinematics()
{
    return impl->inverseKinematics;
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
            impl->jointPath->setCustomIkDisabled(on);
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


const CoordinateFrame* LinkKinematicsKit::baseFrame(const GeneralId& id) const
{
    return const_cast<LinkKinematicsKit*>(this)->baseFrame(id);
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


const CoordinateFrame* LinkKinematicsKit::offsetFrame(const GeneralId& id) const
{
    return const_cast<LinkKinematicsKit*>(this)->offsetFrame(id);
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


const CoordinateFrame* LinkKinematicsKit::currentBaseFrame() const
{
    return const_cast<LinkKinematicsKit*>(this)->currentBaseFrame();
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


const CoordinateFrame* LinkKinematicsKit::currentOffsetFrame() const
{
    return const_cast<LinkKinematicsKit*>(this)->currentOffsetFrame();
}


void LinkKinematicsKit::setCurrentBaseFrame(const GeneralId& id)
{
    impl->currentBaseFrameId = id;
}


void LinkKinematicsKit::setCurrentOffsetFrame(const GeneralId& id)
{
    impl->currentOffsetFrameId = id;
}


Isometry3 LinkKinematicsKit::endPosition
(const GeneralId& baseFrameId, const GeneralId& offsetFrameId) const
{
    Isometry3 T_base;
    const CoordinateFrame* baseFrame_;
    if(baseFrameId.isValid()){
        baseFrame_ = baseFrame(baseFrameId);
    } else {
        baseFrame_ = currentBaseFrame();
    }
    if(baseFrame_->isGlobal()){
        T_base = baseFrame_->T();
    } else {
        T_base = impl->jointPath->baseLink()->T() * baseFrame_->T();
    }
    
    const CoordinateFrame* offsetFrame_;
    if(offsetFrameId.isValid()){
        offsetFrame_ = offsetFrame(offsetFrameId);
    } else {
        offsetFrame_ = currentOffsetFrame();
    }

    Isometry3 T_end = impl->link->T() * offsetFrame_->T();
    return T_base.inverse(Eigen::Isometry) * T_end;
}


Isometry3 LinkKinematicsKit::globalEndPosition(const GeneralId& offsetFrameId) const
{
    const CoordinateFrame* offsetFrame_;
    if(offsetFrameId.isValid()){
        offsetFrame_ = offsetFrame(offsetFrameId);
    } else {
        offsetFrame_ = currentOffsetFrame();
    }
    return impl->link->T() * offsetFrame_->T();
}


Isometry3 LinkKinematicsKit::globalBasePosition(const GeneralId& baseFrameId) const
{
    const CoordinateFrame* baseFrame_;
    if(baseFrameId.isValid()){
        baseFrame_ = baseFrame(baseFrameId);
    } else {
        baseFrame_ = currentBaseFrame();
    }
    if(baseFrame_->isGlobal()){
        return baseFrame_->T();
    } else {
        return impl->jointPath->baseLink()->T() * baseFrame_->T();
    }
}


bool LinkKinematicsKit::setEndPosition
(const Isometry3& T, const GeneralId& baseFrameId, const GeneralId& offsetFrameId, int configuration)
{
    if(!impl->jointPath){
        return false;
    }

    Isometry3 T_base;
    CoordinateFrame* baseFrame_;
    if(baseFrameId.isValid()){
        baseFrame_ = baseFrame(baseFrameId);
    } else {
        baseFrame_ = currentBaseFrame();
    }
    if(baseFrame_->isGlobal()){
        T_base = baseFrame_->T();
    } else {
        T_base = baseLink()->T() * baseFrame_->T();
    }

    CoordinateFrame* offsetFrame_;
    if(offsetFrameId.isValid()){
        offsetFrame_ = offsetFrame(offsetFrameId);
    } else {
        offsetFrame_ = currentOffsetFrame();
    }
    Isometry3 T_offset = offsetFrame_->T();
    Isometry3 T_link = T_base * T * T_offset.inverse(Eigen::Isometry);

    if(impl->configurationHandler){
        impl->configurationHandler->setPreferredConfigurationType(configuration);
    }

    bool solved = impl->jointPath->calcInverseKinematics(T_link);

    if(impl->configurationHandler){
        impl->configurationHandler->resetPreferredConfigurationType();
    }

    return solved;
}


bool LinkKinematicsKit::setGlobalEndPosition
(const Isometry3& T_global, const GeneralId& offsetFrameId, int configuration)
{
    if(!impl->jointPath){
        return false;
    }

    CoordinateFrame* offsetFrame_;
    if(offsetFrameId.isValid()){
        offsetFrame_ = offsetFrame(offsetFrameId);
    } else {
        offsetFrame_ = currentOffsetFrame();
    }
    Isometry3 T_offset = offsetFrame_->T();
    Isometry3 T_link = T_global * T_offset.inverse(Eigen::Isometry);

    if(impl->configurationHandler){
        impl->configurationHandler->setPreferredConfigurationType(configuration);
    }

    bool solved = impl->jointPath->calcInverseKinematics(T_link);

    if(impl->configurationHandler){
        impl->configurationHandler->resetPreferredConfigurationType();
    }

    return solved;
}


SignalProxy<void()> LinkKinematicsKit::sigFrameUpdate()
{
    return impl->sigFrameUpdate;
}


void LinkKinematicsKit::notifyFrameUpdate()
{
    impl->sigFrameUpdate();
}


SignalProxy<void(const Isometry3& T_frameCoordinate)> LinkKinematicsKit::sigPositionError()
{
    return impl->sigPositionError;
}


void LinkKinematicsKit::notifyPositionError(const Isometry3& T_frameCoordinate)
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
