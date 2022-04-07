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


bool MprPosition::read(const Mapping& archive)
{
    id_.read(archive, "id");
    archive.read("note", note_);
    return true;
}


bool MprPosition::write(Mapping& archive) const
{
    id_.write(archive, "id");
    if(!note_.empty()){
        archive.write("note", note_, DOUBLE_QUOTED);
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
    

bool MprIkPosition::read(const Mapping& archive)
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
    if(!archive.read("config_id", configuration_)){
        // old
        if(!archive.read("config_index", configuration_)){
            archive.read("configIndex", configuration_); // old
        }
    }

    /*
    auto& phaseNodes = *archive.findListing("phases");
    if(phaseNodes.isValid()){
        int i = 0;
        int n = std::min(phaseNodes.size(), MaxNumJoints);
        while(i < n){
            phase_[i] = phaseNodes[i].toInt();
            ++i;
        }
        while(i < MaxNumJoints){
            phase_[i] = 0;
        }
    }
    */

    return true;
}


bool MprIkPosition::write(Mapping& archive) const
{
    archive.write("type", "IkPosition");
    
    if(!MprPosition::write(archive)){
        return false;
    }
    
    archive.setFloatingNumberFormat("%.10g");
    cnoid::write(archive, "translation", Vector3(T.translation()));
    cnoid::write(archive, "rotation", degree(rpy()));

    baseFrameId_.write(archive, "base_frame");
    offsetFrameId_.write(archive, "offset_frame");

    archive.write("config_id", configuration_);

    /*
    auto& phaseNodes = *archive.createFlowStyleListing("phases");
    for(auto& phase : phase_){
        phaseNodes.append(phase);
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


bool MprFkPosition::read(const Mapping& archive)
{
    if(!MprPosition::read(archive)){
        return false;
    }

    prismaticJointFlags_.reset();
    auto plist = archive.findListing("prismatic_joints");
    if(!plist){
        plist = archive.findListing("prismaticJoints"); // old
    }
    if(plist){
        for(int i=0; i < plist->size(); ++i){
            int index = plist->at(i)->toInt();
            if(index < MaxNumJoints){
                prismaticJointFlags_[index] = true;
            }
        }
    }

    
    auto nodes = archive.findListing("joint_displacements");
    if(!*nodes){
        nodes = archive.findListing("jointDisplacements"); // old
    }
    if(!*nodes){
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


bool MprFkPosition::write(Mapping& archive) const
{
    archive.write("type", "FkPosition");
    
    MprPosition::write(archive);

    archive.setFloatingNumberFormat("%.9g");

    auto& qlist = *archive.createFlowStyleListing("joint_displacements");
    ListingPtr plist = new Listing;

    for(int i=0; i < numJoints_; ++i){
        double q = jointDisplacements_[i];
        if(prismaticJointFlags_[i]){
            plist->append(i);
        } else {
            q = degree(q);
        }
        qlist.append(q);
    }

    if(!plist->empty()){
        plist->setFlowStyle(true);
        archive.insert("prismatic_joints", plist);
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
    mainPositionIndex_ = -1;
}


MprCompositePosition::MprCompositePosition(const MprCompositePosition& org, CloneMap* cloneMap)
    : MprPosition(org)
{
    for(auto position : org.positions_){
        if(cloneMap){
            position = cloneMap->getClone(position);
        }
        positions_.push_back(position);
    }
    
    mainPositionIndex_ = org.mainPositionIndex_;
}


Referenced* MprCompositePosition::doClone(CloneMap* cloneMap) const
{
    return new MprCompositePosition(*this, cloneMap);
}


void MprCompositePosition::clearPositions()
{
    positions_.clear();
    mainPositionIndex_ = -1;
}


void MprCompositePosition::setNumPositions(int n)
{
    if(n >= 0){
        positions_.resize(n);
        if(mainPositionIndex_ >= n){
            mainPositionIndex_ = -1;
        }
    }
}


void MprCompositePosition::setPosition(int index, MprPosition* position)
{
    if(position->isComposite()){
        throw std::invalid_argument("MprCompositePosition cannot contain a composite position.");
    }
    if(index >= static_cast<int>(positions_.size())){
        positions_.resize(index + 1);
    }
    positions_[index] = position;
}


namespace {

enum CompositeProcessResult { Unmatched, Tried, Processed, Completed };

CompositeProcessResult processPositionsAndKinematicsKits
(MprCompositePosition* compositePosition, LinkKinematicsKitSet* kinematicsKitSet,
 const std::function<bool(MprPosition* position, LinkKinematicsKit* kit)>& callback)
{
    bool tried = false;
    bool processed = false;
    bool completed = false;

    int mainPositionIndex = compositePosition->mainPositionIndex();
    int mainKinematicsKitIndex = kinematicsKitSet->mainKinematicsKitIndex();
    if(mainPositionIndex >= 0 && mainKinematicsKitIndex >= 0){
        int n = compositePosition->numPositions();
        int numKinematicsKits = kinematicsKitSet->numKinematicsKits();
        if(mainPositionIndex == mainKinematicsKitIndex){
            if(n == numKinematicsKits){
                completed = true;
            } else {
                n = std::min(n, numKinematicsKits);
            }
            tried = true;
            for(int i=0; i < n; ++i){
                auto position = compositePosition->position(i);
                auto kit = kinematicsKitSet->kinematicsKit(i);
                if(position && kit && callback(position, kit)){
                    processed = true;
                } else {
                    completed = false;
                }
            }
            if(!processed){
                completed = false;
            }
        } else if(n == 1 || numKinematicsKits == 1){
            tried = true;
            processed = callback(compositePosition->mainPosition(), kinematicsKitSet->mainKinematicsKit());
        }
    }

    CompositeProcessResult result;
    if(!tried){
        result = Unmatched;
    } else if(!processed){
        result = Tried;
    } else if(!completed){
        result = Processed;
    } else {
        result = Completed;
    }
    return result;
}

}


bool MprCompositePosition::fetch(LinkKinematicsKitSet* kinematicsKitSet, MessageOut* mout)
{
    auto result = processPositionsAndKinematicsKits(
        this, kinematicsKitSet,
        [this](MprPosition* position, LinkKinematicsKit* kit){ return position->fetch(kit); });

    if(result == Unmatched){
        mout->putError(
            format(_("Position {0} cannot be fetched due to the position set mismatch."),
                   id().label()));
    } else if(result == Tried){
        mout->putError(format(_("Fetching position {0} failed."), id().label()));
    } else if(result == Processed){
        mout->putWarning(format(_("Could not fetch all elements of position {0}."), id().label()));
    }

    return (result == Processed || result == Completed);
}


bool MprCompositePosition::apply(LinkKinematicsKitSet* kinematicsKitSet) const
{
    auto result = processPositionsAndKinematicsKits(
        const_cast<MprCompositePosition*>(this), kinematicsKitSet,
        [this](MprPosition* position, LinkKinematicsKit* kit){ return position->apply(kit); });

    return (result == Processed || result == Completed);
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


bool MprCompositePosition::read(const Mapping& archive)
{
    if(!MprPosition::read(archive)){
        return false;
    }
    auto positionList = archive.findListing("positions");
    if(!positionList->isValid()){
        return false;
    }
    clearPositions();
    int n = positionList->size();
    setNumPositions(n);
    for(int i=0; i < n; ++i){
        auto& node = *positionList->at(i)->toMapping();
        auto& typeNode = node["type"];
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
        if(position){
            if(position->read(node)){
                setPosition(i, position);
            }
        }
    }
    return true;
}


bool MprCompositePosition::write(Mapping& archive) const
{
    archive.write("type", "CompositePosition");

    if(!MprPosition::write(archive)){
        return false;
    }

    auto positionList = archive.createListing("positions");
    for(auto& position : positions_){
        auto positionArchive = positionList->newMapping();
        if(position){
            position->write(*positionArchive);
        }
    }

    return true;
}
