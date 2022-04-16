#include "MprPosition.h"
#include "MprPositionList.h"
#include <cnoid/Body>
#include <cnoid/LinkKinematicsKit>
#include <cnoid/LinkKinematicsKitSet>
#include <cnoid/JointPath>
#include <cnoid/JointSpaceConfigurationHandler>
#include <cnoid/CoordinateFrame>
#include <cnoid/EigenUtil>
#include <cnoid/EigenArchive>
#include <cnoid/CloneMap>
#include <cnoid/MessageOut>
#include <fmt/format.h>
#include <stdexcept>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

constexpr int MprPosition::MaxNumJoints;

namespace {

bool checkJointDisplacementRanges(JointPath& jointPath, MessageOut* mout)
{
    bool isOver = false;
    for(auto& joint : jointPath){
        if(joint->isRevoluteJoint() || joint->isPrismaticJoint()){
            if(joint->q() < joint->q_lower() || joint->q() > joint->q_upper()){
                if(mout){
                    mout->putError(
                        format(_("The joint displacement of {0} is out of its movable range."),
                               joint->jointName()));
                }
                isOver = true;
                break;
            }
        }
    }
    return !isOver;
}

}


MprPosition::MprPosition(PositionType type)
    : positionType_(type)
{

}


MprPosition::MprPosition(PositionType type, const GeneralId& id)
    : positionType_(type),
      id_(id)
{

}


MprPosition::MprPosition(const MprPosition& org)
    : positionType_(org.positionType_),
      id_(org.id_),
      note_(org.note_)
{

}


void MprPosition::setId(const GeneralId& id)
{
    if(!ownerPositionList_){
        id_ = id;
    }
}


MprIkPosition* MprPosition::ikPosition()
{
    if(positionType_ == IK){
        return static_cast<MprIkPosition*>(this);
    }
    return nullptr;
}


MprFkPosition* MprPosition::fkPosition()
{
    if(positionType_ == FK){
        return static_cast<MprFkPosition*>(this);
    }
    return nullptr;
}


MprCompositePosition* MprPosition::compositePosition()
{
    if(positionType_ == Composite){
        return static_cast<MprCompositePosition*>(this);
    }
    return nullptr;
}


MprPositionList* MprPosition::ownerPositionList()
{
    return ownerPositionList_.lock();
}


bool MprPosition::fetch(LinkKinematicsKitSet* kinematicsKitSet, MessageOut* mout)
{
    if(auto kinematicsKit = kinematicsKitSet->mainKinematicsKit()){
        return fetch(kinematicsKit);
    }
    return false;
}
    

bool MprPosition::apply(LinkKinematicsKitSet* kinematicsKitSet) const
{
    if(auto kinematicsKit = kinematicsKitSet->mainKinematicsKit()){
        return apply(kinematicsKit);
    }
    return false;
}


void MprPosition::notifyUpdate(int flags)
{
    sigUpdated_(flags);
    if(auto positionList = ownerPositionList()){
        positionList->notifyPositionUpdate(this, flags);
    }
}


bool MprPosition::read(const Mapping* archive)
{
    id_.read(archive, "id");
    archive->read("note", note_);
    return true;
}


bool MprPosition::write(Mapping* archive) const
{
    id_.write(archive, "id");
    if(!note_.empty()){
        archive->write("note", note_, DOUBLE_QUOTED);
    }
    return true;
}


MprIkPosition::MprIkPosition()
    : MprIkPosition(GeneralId())
{

}


MprIkPosition::MprIkPosition(const GeneralId& id)
    : MprPosition(IK, id),
      baseFrameId_(0),
      offsetFrameId_(0)
{
    T.setIdentity();
    referenceRpy_.setZero();
    configuration_ = 0;
    phase_.fill(0);
}


MprIkPosition::MprIkPosition(const MprIkPosition& org)
    : MprPosition(org),
      T(org.T),
      referenceRpy_(org.referenceRpy_),
      baseFrameId_(org.baseFrameId_),
      offsetFrameId_(org.offsetFrameId_),
      configuration_(org.configuration_),
      phase_(org.phase_)
{

}


Referenced* MprIkPosition::doClone(CloneMap*) const
{
    return new MprIkPosition(*this);
}


Vector3 MprIkPosition::rpy() const
{
    return rpyFromRot(T.linear(), referenceRpy_);
}


void MprIkPosition::setRpy(const Vector3& rpy)
{
    T.linear() = rotFromRpy(rpy);
    referenceRpy_ = rpy;
}


void MprIkPosition::setReferenceRpy(const Vector3& rpy)
{
    referenceRpy_ = rpy;
}


void MprIkPosition::resetReferenceRpy()
{
    referenceRpy_.setZero();
}


bool MprIkPosition::fetch(LinkKinematicsKit* kinematicsKit, MessageOut* mout)
{
    bool fetched = false;
    
    if(auto jointPath = kinematicsKit->jointPath()){
        bool failed = false;
        if(auto configuration = kinematicsKit->configurationHandler()){
            int state = configuration->getCurrentNearSingularPointState();
            if(state > 0){
                if(mout){
                    mout->putError(
                        format(_("The current manipulator position is not valid: {0}."),
                               configuration->getNearSingularPointFactorString(state)));
                }
                failed = true;
            }
        }
        if(!failed && !checkJointDisplacementRanges(*jointPath, mout)){
            failed = true;
        }
        if(!failed){
            T = kinematicsKit->endPosition();
            baseFrameId_ = kinematicsKit->currentBaseFrameId();
            offsetFrameId_ = kinematicsKit->currentOffsetFrameId();
            referenceRpy_ = rpyFromRot(T.linear(), kinematicsKit->referenceRpy());
            configuration_ = kinematicsKit->currentConfigurationType();
            //! \todo set phase here
            fetched = true;
        }
    }

    return fetched;
}


bool MprIkPosition::apply(LinkKinematicsKit* kinematicsKit) const
{
    if(kinematicsKit->setEndPosition(T, baseFrameId_, offsetFrameId_, configuration_)){
        kinematicsKit->setReferenceRpy(rpyFromRot(T.linear(), referenceRpy_));
        return true;
    }
    return false;
}
    

bool MprIkPosition::read(const Mapping* archive)
{
    if(!MprPosition::read(archive)){
        return false;
    }

    Vector3 v;
    if(cnoid::read(archive, "translation", v)){
        T.translation() = v;
    } else {
        T.translation().setZero();
    }
    if(cnoid::read(archive, "rotation", v)){
        setRpy(radian(v));
    } else {
        T.linear().setIdentity();
        resetReferenceRpy();
    }

    if(!baseFrameId_.read(archive, "base_frame")){
        baseFrameId_.read(archive, "baseFrame"); // old
    }
    if(!offsetFrameId_.read(archive, "offset_frame")){
        if(!offsetFrameId_.read(archive, "tool_frame")){ // old
            offsetFrameId_.read(archive, "toolFrame"); // old
        }
    }

    configuration_ = 0;
    if(!archive->read("config_id", configuration_)){
        // old
        if(!archive->read("config_index", configuration_)){
            archive->read("configIndex", configuration_); // old
        }
    }

    /*
    auto phaseNodes = archive->findListing("phases");
    if(phaseNodes->isValid()){
        int i = 0;
        int n = std::min(phaseNodes->size(), MaxNumJoints);
        while(i < n){
            phase_[i] = phaseNodes->at(i)->toInt();
            ++i;
        }
        while(i < MaxNumJoints){
            phase_[i] = 0;
        }
    }
    */

    return true;
}


bool MprIkPosition::write(Mapping* archive) const
{
    archive->write("type", "IkPosition");
    
    if(!MprPosition::write(archive)){
        return false;
    }
    
    archive->setFloatingNumberFormat("%.10g");
    cnoid::write(archive, "translation", Vector3(T.translation()));
    cnoid::write(archive, "rotation", degree(rpy()));

    baseFrameId_.write(archive, "base_frame");
    offsetFrameId_.write(archive, "offset_frame");

    archive->write("config_id", configuration_);

    /*
    auto phaseNodes = archive->createFlowStyleListing("phases");
    for(auto& phase : phase_){
        phaseNodes->append(phase);
    }
    */
    
    return true;
}


MprFkPosition::MprFkPosition()
    : MprFkPosition(GeneralId())
{

}


MprFkPosition::MprFkPosition(const GeneralId& id)
    : MprPosition(FK, id),
      prismaticJointFlags_(0)
{
    jointDisplacements_.fill(0.0);
    numJoints_ = 0;
}


MprFkPosition::MprFkPosition(const MprFkPosition& org)
    : MprPosition(org),
      jointDisplacements_(org.jointDisplacements_),
      prismaticJointFlags_(org.prismaticJointFlags_)
{
    numJoints_ = org.numJoints_;
}


Referenced* MprFkPosition::doClone(CloneMap*) const
{
    return new MprFkPosition(*this);
}


bool MprFkPosition::fetch(LinkKinematicsKit* kinematicsKit, MessageOut* mout)
{
    bool fetched = false;
    
    if(auto jointPath = kinematicsKit->jointPath()){
        if(checkJointDisplacementRanges(*jointPath, mout)){
            numJoints_ = std::min(jointPath->numJoints(), MaxNumJoints);
            int i;
            for(i = 0; i < numJoints_; ++i){
                auto joint = jointPath->joint(i);
                jointDisplacements_[i] = joint->q();
                prismaticJointFlags_[i] = joint->isPrismaticJoint();
            }
            for( ; i < MaxNumJoints; ++i){
                jointDisplacements_[i] = 0.0;
                prismaticJointFlags_[i] = false;
            }
            fetched = true;
        }
    }

    return fetched;
}


bool MprFkPosition::apply(LinkKinematicsKit* kinematicsKit) const
{
    auto path = kinematicsKit->jointPath();
    int nj = std::min(path->numJoints(), numJoints_);
    for(int i = 0; i < nj; ++i){
        path->joint(i)->q() = jointDisplacements_[i];
    }
    path->calcForwardKinematics();

    return true;
}


bool MprFkPosition::read(const Mapping* archive)
{
    if(!MprPosition::read(archive)){
        return false;
    }

    prismaticJointFlags_.reset();
    auto plist = archive->findListing("prismatic_joints");
    if(!plist->isValid()){
        plist = archive->findListing("prismaticJoints"); // old
    }
    if(plist->isValid()){
        for(int i=0; i < plist->size(); ++i){
            int index = plist->at(i)->toInt();
            if(index < MaxNumJoints){
                prismaticJointFlags_[index] = true;
            }
        }
    }

    
    auto nodes = archive->findListing("joint_displacements");
    if(!nodes->isValid()){
        nodes = archive->findListing("jointDisplacements"); // old
    }
    if(!nodes->isValid()){
        numJoints_ = 0;
    } else {
        numJoints_ = std::min(nodes->size(), MaxNumJoints);
        int i = 0;
        while(i < numJoints_){
            double q = nodes->at(i)->toDouble();
            if(!prismaticJointFlags_[i]){
                q = radian(q);
            }
            jointDisplacements_[i] = q;
            ++i;
        }
        while(i < MaxNumJoints){
            jointDisplacements_[i++] = 0.0;
        }
    }
    
    return true;
}


bool MprFkPosition::write(Mapping* archive) const
{
    archive->write("type", "FkPosition");
    
    MprPosition::write(archive);

    archive->setFloatingNumberFormat("%.9g");

    auto qlist = archive->createFlowStyleListing("joint_displacements");
    ListingPtr plist = new Listing;

    for(int i=0; i < numJoints_; ++i){
        double q = jointDisplacements_[i];
        if(prismaticJointFlags_[i]){
            plist->append(i);
        } else {
            q = degree(q);
        }
        qlist->append(q);
    }

    if(!plist->empty()){
        plist->setFlowStyle(true);
        archive->insert("prismatic_joints", plist);
    }
    
    return true;
}


MprCompositePosition::MprCompositePosition()
    : MprCompositePosition(GeneralId())
{
    
}


MprCompositePosition::MprCompositePosition(const GeneralId& id)
    : MprPosition(Composite, id)
{

}


MprCompositePosition::MprCompositePosition(const MprCompositePosition& org, CloneMap* cloneMap)
    : MprPosition(org),
      mainPartId_(org.mainPartId_)
{
    if(!cloneMap){
        positionMap_ = org.positionMap_;
    } else {
        for(auto& kv : org.positionMap_){
            MprPosition* position = kv.second;
            positionMap_.emplace(kv.first, cloneMap->getClone(position));
        }
    }
}


Referenced* MprCompositePosition::doClone(CloneMap* cloneMap) const
{
    return new MprCompositePosition(*this, cloneMap);
}


void MprCompositePosition::clearPositions()
{
    positionMap_.clear();
    mainPartId_.reset();
}


void MprCompositePosition::setPosition(const GeneralId& partId, MprPosition* position)
{
    if(!partId.isValid()){
        throw std::invalid_argument("Invalid ID is given to MprCompositePosition::setPosition.");
    }
    if(!position){
        positionMap_.erase(partId);
        if(mainPartId_ == partId){
            mainPartId_.reset();
        }
    } else {
        if(position->isComposite()){
            throw std::invalid_argument("MprCompositePosition cannot contain a composite position.");
        }
        positionMap_[partId] = position;
    }
}


MprPosition* MprCompositePosition::position(const GeneralId& partId)
{
    auto it = positionMap_.find(partId);
    if(it != positionMap_.end()){
        return it->second;
    }
    return nullptr;
}


const MprPosition* MprCompositePosition::position(const GeneralId& partId) const
{
    return const_cast<MprCompositePosition*>(this)->position(partId);
}


MprPosition* MprCompositePosition::mainPosition()
{
    if(mainPartId_.isValid()){
        return position(mainPartId_);
    }
    return nullptr;

}


const MprPosition* MprCompositePosition::mainPosition() const
{
    return const_cast<MprCompositePosition*>(this)->mainPosition();
}


bool MprCompositePosition::fetch(LinkKinematicsKitSet* kinematicsKitSet, MessageOut* mout)
{
    int numFetched = 0;
    for(auto& kv : positionMap_){
        auto& partId = kv.first;
        if(auto kinematicsKit = kinematicsKitSet->kinematicsKit(partId)){
            auto& position = kv.second;
            if(position->fetch(kinematicsKit)){
                ++numFetched;
            }
        }
    }

    bool fetched;
    
    if(numFetched == 0){
        fetched = false;
        mout->putError(
            format(_("Position {0} cannot be fetched due to the position part set mismatch."),
                   id().label()));
    } else {
        fetched = true;
        if(numFetched < static_cast<int>(positionMap_.size())){
            mout->putWarning(format(_("Could not fetch all elements of position {0}."), id().label()));
        }
    }

    return fetched;
}


bool MprCompositePosition::apply(LinkKinematicsKitSet* kinematicsKitSet) const
{
    int numApplied = 0;
    for(auto& kv : positionMap_){
        auto& partId = kv.first;
        if(auto kinematicsKit = kinematicsKitSet->kinematicsKit(partId)){
            auto& position = kv.second;
            if(position->apply(kinematicsKit)){
                ++numApplied;
            }
        }
    }

    return numApplied > 0;
}


bool MprCompositePosition::fetch(LinkKinematicsKit* kinematicsKit, MessageOut* mout)
{
    if(auto position = mainPosition()){
        return position->fetch(kinematicsKit, mout);
    }
    return false;
}


bool MprCompositePosition::apply(LinkKinematicsKit* kinematicsKit) const
{
    if(auto position = mainPosition()){
        return position->apply(kinematicsKit);
    }
    return false;
}


bool MprCompositePosition::read(const Mapping* archive)
{
    if(!MprPosition::read(archive)){
        return false;
    }

    mainPartId_.read(archive, "main_part");
    
    auto positionList = archive->findListing("positions");
    if(!positionList->isValid()){
        return false;
    }
    clearPositions();

    int n = positionList->size();
    for(int i=0; i < n; ++i){
        auto node = positionList->at(i)->toMapping();
        GeneralId partId;
        partId.readEx(node, "part");
        auto& typeNode = node->get("type");
        auto type = typeNode.toString();
        MprPositionPtr position;
        if(type == "IkPosition"){
            position = new MprIkPosition;
        } else if(type == "FkPosition"){
            position = new MprFkPosition;
        } else if(type == "CompositePosition"){
            typeNode.throwException(_("Recursive CompositePosition structure is not supported"));
        } else {
            typeNode.throwException(format(_("{0} is not supported"), type));
        }
        if(position->read(node)){
            setPosition(partId, position);
        }
    }
    
    return true;
}


bool MprCompositePosition::write(Mapping* archive) const
{
    archive->write("type", "CompositePosition");

    if(!MprPosition::write(archive)){
        return false;
    }

    auto positionList = archive->createListing("positions");
    for(auto& kv : positionMap_){
        auto positionArchive = positionList->newMapping();
        auto& partId = kv.first;
        positionArchive->write("part", partId.label());
        auto& position = kv.second;
        position->write(positionArchive);
    }

    return true;
}
