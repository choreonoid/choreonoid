#include "BodyKinematicsKit.h"
#include "Body.h"
#include "JointPath.h"
#include "JointSpaceConfigurationHandler.h"
#include "CompositeBodyIK.h"
#include "JointTraverse.h"
#include "HolderDevice.h"
#include "AttachmentDevice.h"
#include "LinkedJointHandler.h"
#include <cnoid/CoordinateFrameList>
#include <cnoid/CloneMap>
#include <cnoid/ValueTree>

using namespace std;
using namespace cnoid;

namespace cnoid {

class BodyKinematicsKit::Impl
{
public:
    BodyPtr body;
    LinkPtr endLink;
    shared_ptr<JointPath> jointPath;
    shared_ptr<InverseKinematics> inverseKinematics;
    shared_ptr<JointSpaceConfigurationHandler> configurationHandler;
    shared_ptr<JointTraverse> jointTraverse;
    LinkedJointHandlerPtr linkedJointHandler;
    bool isCustomIkDisabled;
    bool isRpySpecified;
    Vector3 referenceRpy;
    
    CoordinateFrameListPtr baseFrames;
    CoordinateFrameListPtr offsetFrames;
    GeneralId currentBaseFrameId;
    GeneralId currentOffsetFrameId;
    CoordinateFramePtr defaultBaseFrame;
    CoordinateFramePtr defaultOffsetFrame;

    Signal<void()> sigFrameSetChanged;
    Signal<void(const Isometry3& T_frameCoordinate)> sigPositionError;
    
    Impl();
    Impl(const Impl& org, CloneMap* cloneMap);
    void setJointPath(Link* baseLink, Link* endLink);
    void setInverseKinematics(Link* endLink, std::shared_ptr<InverseKinematics> ik);
    void setJointTraverse(std::shared_ptr<JointTraverse> jointTraverse);
    Link* baseLink();
};

}


BodyKinematicsKit::BodyKinematicsKit()
{
    impl = new Impl;
}


BodyKinematicsKit::Impl::Impl()
    : isCustomIkDisabled(false),
      referenceRpy(Vector3::Zero()),
      currentBaseFrameId(0),
      currentOffsetFrameId(0)
{
    defaultBaseFrame = new CoordinateFrame;
    defaultOffsetFrame = new CoordinateFrame;
}


BodyKinematicsKit::BodyKinematicsKit(const BodyKinematicsKit& org, CloneMap* cloneMap)
{
    impl = new Impl(*org.impl, cloneMap);
}


BodyKinematicsKit::Impl::Impl(const Impl& org, CloneMap* cloneMap)
{
    isCustomIkDisabled = org.isCustomIkDisabled;

    currentBaseFrameId = org.currentBaseFrameId;
    currentOffsetFrameId = org.currentOffsetFrameId;
    
    if(!cloneMap){
        body = org.body;
        endLink = org.endLink;
        jointPath = org.jointPath;
        inverseKinematics = org.inverseKinematics;
        jointTraverse = org.jointTraverse;
        isRpySpecified = org.isRpySpecified;
        referenceRpy = org.referenceRpy;
        baseFrames = org.baseFrames;
        offsetFrames = org.offsetFrames;

    } else {
        referenceRpy.setZero();
        body = cloneMap->getClone(org.body);
        if(body){
            if(org.endLink){
                endLink = body->link(org.endLink->index());
                if(auto orgBaseLink = const_cast<Impl&>(org).baseLink()){
                    auto baseLink = body->link(orgBaseLink->index());
                    setJointPath(baseLink, endLink);
                } else {
                    // The inverse kinematics object should be cloned here.
                }
            } else if(org.jointTraverse){
                setJointTraverse(make_shared<JointTraverse>(*org.jointTraverse, cloneMap));
            }
            linkedJointHandler = LinkedJointHandler::findOrCreateLinkedJointHandler(body);
        }
        baseFrames = cloneMap->getClone(org.baseFrames);
        offsetFrames = cloneMap->getClone(org.offsetFrames);
    }

    defaultBaseFrame = new CoordinateFrame;
    defaultOffsetFrame = new CoordinateFrame;
}


Referenced* BodyKinematicsKit::doClone(CloneMap* cloneMap) const
{
    return new BodyKinematicsKit(*this, cloneMap);
}


BodyKinematicsKit::~BodyKinematicsKit()
{
    delete impl;
}


void BodyKinematicsKit::setEndLink(Link* endLink)
{
    impl->jointPath.reset();
    impl->inverseKinematics.reset();
    impl->configurationHandler.reset();
    impl->jointTraverse.reset();
    impl->endLink = endLink;
}


void BodyKinematicsKit::setJointPath(Link* baseLink, Link* endLink)
{
    impl->setJointPath(baseLink, endLink);
}


void BodyKinematicsKit::Impl::setJointPath(Link* baseLink, Link* endLink)
{
    jointPath.reset();
    inverseKinematics.reset();
    configurationHandler.reset();
    jointTraverse.reset();
    Body* prevBody = body;

    if(baseLink && endLink && baseLink->body() == endLink->body()){
        body = baseLink->body();
        this->endLink = endLink;
        jointPath = JointPath::getCustomPath(baseLink, endLink);
        if(jointPath){
            inverseKinematics = jointPath;
            if(jointPath->hasCustomIK()){
                configurationHandler =
                    dynamic_pointer_cast<JointSpaceConfigurationHandler>(jointPath);
                jointPath->setCustomIkDisabled(isCustomIkDisabled);
            }
            if(!jointPath->empty()){
                bool isDownward = jointPath->isJointDownward(0);
                jointTraverse = make_shared<JointTraverse>(baseLink, !isDownward, isDownward);
            }
        }
        if(body != prevBody){
            linkedJointHandler = LinkedJointHandler::findOrCreateLinkedJointHandler(body);
        }
    } else {
        linkedJointHandler.reset();
    }
}


void BodyKinematicsKit::setInverseKinematics(Link* endLink, std::shared_ptr<InverseKinematics> ik)
{
    impl->setInverseKinematics(endLink, ik);
}


void BodyKinematicsKit::Impl::setInverseKinematics(Link* endLink, std::shared_ptr<InverseKinematics> ik)
{
    jointPath.reset();
    inverseKinematics.reset();
    configurationHandler.reset();
    jointTraverse.reset();
    Body* prevBody = body;

    body = endLink->body();
    this->endLink = endLink;
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

        if(!jointPath->empty()){
            bool isDownward = jointPath->isJointDownward(0);
            jointTraverse = make_shared<JointTraverse>(jointPath->baseLink(), !isDownward, isDownward);
        }
    }

    if(body != prevBody){
        linkedJointHandler = LinkedJointHandler::findOrCreateLinkedJointHandler(body);
    }
}


void BodyKinematicsKit::setJointTraverse(Body* body)
{
    impl->setJointTraverse(make_shared<JointTraverse>(body));
}


void BodyKinematicsKit::setJointTraverse(Link* baseLink)
{
    impl->setJointTraverse(make_shared<JointTraverse>(baseLink));
}


void BodyKinematicsKit::setJointTraverse(std::shared_ptr<JointTraverse> jointTraverse)
{
    impl->setJointTraverse(jointTraverse);
}


void BodyKinematicsKit::Impl::setJointTraverse(std::shared_ptr<JointTraverse> jointTraverse)
{
    Body* prevBody = body;
    
    body = jointTraverse->body();
    endLink.reset();
    jointPath.reset();
    inverseKinematics.reset();
    configurationHandler.reset();
    this->jointTraverse = jointTraverse;
    
    if(body != prevBody){
        linkedJointHandler = LinkedJointHandler::findOrCreateLinkedJointHandler(body);
    }
}


void BodyKinematicsKit::setBaseFrames(CoordinateFrameList* frames)
{
    impl->baseFrames = frames;
}


void BodyKinematicsKit::setOffsetFrames(CoordinateFrameList* frames)
{
    impl->offsetFrames = frames;
}


Body* BodyKinematicsKit::body()
{
    return impl->body;
}


const Body* BodyKinematicsKit::body() const
{
    return impl->body;
}


Link* BodyKinematicsKit::endLink()
{
    return impl->endLink;
}


const Link* BodyKinematicsKit::endLink() const
{
    return impl->endLink;
}


Link* BodyKinematicsKit::Impl::baseLink()
{
    if(jointPath){
        return jointPath->baseLink();
    } else if(jointTraverse){
        return jointTraverse->linkTraverse().rootLink();
    }
    return nullptr;
}


Link* BodyKinematicsKit::baseLink()
{
    return impl->baseLink();
}


const Link* BodyKinematicsKit::baseLink() const
{
    return const_cast<BodyKinematicsKit::Impl*>(impl)->baseLink();
}


bool BodyKinematicsKit::hasJointPath() const
{
    return (bool)impl->jointPath;
}


std::shared_ptr<JointPath> BodyKinematicsKit::jointPath()
{
    return impl->jointPath;
}


bool BodyKinematicsKit::hasJointTraverse() const
{
    return (bool)impl->jointTraverse;
}


std::shared_ptr<JointTraverse> BodyKinematicsKit::jointTraverse()
{
    return impl->jointTraverse;
}


int BodyKinematicsKit::numJoints() const
{
    if(impl->jointPath){
        return impl->jointPath->numJoints();
    } else if(impl->jointTraverse){
        return impl->jointTraverse->numJoints();
    }
    return 0;
}


Link* BodyKinematicsKit::joint(int index)
{
    if(impl->jointPath){
        return impl->jointPath->joint(index);
    } else if(impl->jointTraverse){
        return impl->jointTraverse->joint(index);
    }
    return nullptr;
}


const Link* BodyKinematicsKit::joint(int index) const
{
    return const_cast<BodyKinematicsKit*>(this)->joint(index);
}


std::vector<Link*> BodyKinematicsKit::joints() const
{
    std::vector<Link*> joints_;
    
    if(impl->jointPath){
        joints_.reserve(impl->jointPath->numJoints());
        for(auto& joint : impl->jointPath->joints()){
            joints_.push_back(joint);
        }
    } else if(impl->jointTraverse){
        joints_.reserve(impl->jointTraverse->numJoints());
        for(auto& joint : impl->jointTraverse->joints()){
            joints_.push_back(joint);
        }
    }
    
    return joints_;
}


std::shared_ptr<InverseKinematics> BodyKinematicsKit::inverseKinematics()
{
    return impl->inverseKinematics;
}


LinkedJointHandler* BodyKinematicsKit::linkedJointHandler()
{
    return impl->linkedJointHandler;
}


std::shared_ptr<JointSpaceConfigurationHandler> BodyKinematicsKit::configurationHandler()
{
    return impl->configurationHandler;
}


int BodyKinematicsKit::currentConfigurationType() const
{
    if(impl->configurationHandler){
        return impl->configurationHandler->getCurrentConfigurationType();
    }
    return 0;
}


std::string BodyKinematicsKit::configurationLabel(int id) const
{
    if(impl->configurationHandler){
        string label;
        for(auto& element : impl->configurationHandler->getConfigurationStateNames(id)){
            if(!label.empty() && !element.empty()){
                label.append("-");
            }
            label.append(element);
        }
        return label;
    }
    return std::string();
}


bool BodyKinematicsKit::isCustomIkAvaiable() const
{
    return (impl->jointPath && impl->jointPath->hasCustomIK());
}


bool BodyKinematicsKit::isCustomIkDisabled() const
{
    return impl->isCustomIkDisabled;
}


void BodyKinematicsKit::setCustomIkDisabled(bool on)
{
    if(on != impl->isCustomIkDisabled){
        if(impl->jointPath){
            impl->jointPath->setCustomIkDisabled(on);
        }
        impl->isCustomIkDisabled = on;
    }
}


Vector3 BodyKinematicsKit::referenceRpy() const
{
    return impl->referenceRpy;
}


void BodyKinematicsKit::setReferenceRpy(const Vector3& rpy)
{
    impl->referenceRpy = rpy;
}


void BodyKinematicsKit::resetReferenceRpy()
{
    impl->referenceRpy.setZero();
}


bool BodyKinematicsKit::isManipulator() const
{
    if(impl->body && impl->endLink &&
       impl->endLink == impl->body->guessMainEndLink() &&
       impl->configurationHandler){
        return true;
    }
    return false;
}


CoordinateFrameList* BodyKinematicsKit::baseFrames()
{
    return impl->baseFrames;
}


CoordinateFrameList* BodyKinematicsKit::offsetFrames()
{
    return impl->offsetFrames;
}


CoordinateFrame* BodyKinematicsKit::baseFrame(const GeneralId& id)
{
    if(impl->baseFrames){
        if(auto frame = impl->baseFrames->findFrame(id)){
            return frame;
        }
    }
    return impl->defaultBaseFrame;
}


const CoordinateFrame* BodyKinematicsKit::baseFrame(const GeneralId& id) const
{
    return const_cast<BodyKinematicsKit*>(this)->baseFrame(id);
}


CoordinateFrame* BodyKinematicsKit::offsetFrame(const GeneralId& id)
{
    if(impl->offsetFrames){
        if(auto frame = impl->offsetFrames->findFrame(id)){
            return frame;
        }
    }
    return impl->defaultOffsetFrame;
}


const CoordinateFrame* BodyKinematicsKit::offsetFrame(const GeneralId& id) const
{
    return const_cast<BodyKinematicsKit*>(this)->offsetFrame(id);
}


const GeneralId& BodyKinematicsKit::currentBaseFrameId() const
{
    return impl->currentBaseFrameId;
}


const GeneralId& BodyKinematicsKit::currentOffsetFrameId() const
{
    return impl->currentOffsetFrameId;
}


CoordinateFrame* BodyKinematicsKit::currentBaseFrame()
{
    if(impl->baseFrames){
        if(auto frame = impl->baseFrames->findFrame(impl->currentBaseFrameId)){
            return frame;
        }
    }
    return impl->defaultBaseFrame;
}


const CoordinateFrame* BodyKinematicsKit::currentBaseFrame() const
{
    return const_cast<BodyKinematicsKit*>(this)->currentBaseFrame();
}

    
CoordinateFrame* BodyKinematicsKit::currentOffsetFrame()
{
    if(impl->offsetFrames){
        if(auto frame = impl->offsetFrames->findFrame(impl->currentOffsetFrameId)){
            return frame;
        }
    }
    return impl->defaultOffsetFrame;
}


const CoordinateFrame* BodyKinematicsKit::currentOffsetFrame() const
{
    return const_cast<BodyKinematicsKit*>(this)->currentOffsetFrame();
}


void BodyKinematicsKit::setCurrentBaseFrame(const GeneralId& id)
{
    impl->currentBaseFrameId = id;
}


void BodyKinematicsKit::setCurrentOffsetFrame(const GeneralId& id)
{
    impl->currentOffsetFrameId = id;
}


Isometry3 BodyKinematicsKit::endPosition
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
        T_base = impl->baseLink()->T() * baseFrame_->T();
    }
    
    const CoordinateFrame* offsetFrame_;
    if(offsetFrameId.isValid()){
        offsetFrame_ = offsetFrame(offsetFrameId);
    } else {
        offsetFrame_ = currentOffsetFrame();
    }

    Isometry3 T_end = impl->endLink->T() * offsetFrame_->T();
    return T_base.inverse(Eigen::Isometry) * T_end;
}


Isometry3 BodyKinematicsKit::globalEndPosition(const GeneralId& offsetFrameId) const
{
    const CoordinateFrame* offsetFrame_;
    if(offsetFrameId.isValid()){
        offsetFrame_ = offsetFrame(offsetFrameId);
    } else {
        offsetFrame_ = currentOffsetFrame();
    }
    return impl->endLink->T() * offsetFrame_->T();
}


Isometry3 BodyKinematicsKit::globalBasePosition(const GeneralId& baseFrameId) const
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
        return impl->baseLink()->T() * baseFrame_->T();
    }
}


bool BodyKinematicsKit::setEndPosition
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
        T_base = impl->baseLink()->T() * baseFrame_->T();
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


bool BodyKinematicsKit::setGlobalEndPosition
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


SignalProxy<void()> BodyKinematicsKit::sigFrameSetChanged()
{
    return impl->sigFrameSetChanged;
}


void BodyKinematicsKit::notifyFrameSetChange()
{
    impl->sigFrameSetChanged();
}


SignalProxy<void(const Isometry3& T_frameCoordinate)> BodyKinematicsKit::sigPositionError()
{
    return impl->sigPositionError;
}


void BodyKinematicsKit::notifyPositionError(const Isometry3& T_frameCoordinate)
{
    impl->sigPositionError(T_frameCoordinate);
}


bool BodyKinematicsKit::storeState(Mapping& archive) const
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


bool BodyKinematicsKit::restoreState(const Mapping& archive)
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
        notifyFrameSetChange();
    }
    return true;
}
