#include "MprPosition.h"
#include "MprPositionList.h"
#include <cnoid/Body>
#include <cnoid/LinkKinematicsKit>
#include <cnoid/JointPath>
#include <cnoid/JointSpaceConfigurationHandler>
#include <cnoid/CoordinateFrame>
#include <cnoid/EigenUtil>
#include <cnoid/EigenArchive>
#include <cnoid/CloneMap>
#include "gettext.h"

using namespace std;
using namespace cnoid;

constexpr int MprPosition::MaxNumJoints;


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


MprPositionList* MprPosition::ownerPositionList()
{
    return ownerPositionList_.lock();
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
    if(!id_.read(archive, "id")){
        archive.throwException(_("The \"id\" key is not found in a manipulator position node"));
    }
    archive.read("note", note_);
    return true;
}


bool MprPosition::write(Mapping& archive) const
{
    if(id_.write(archive, "id")){
        if(!note_.empty()){
            archive.write("note", note_, DOUBLE_QUOTED);
        }
        return true;
    }
    return false;
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


bool MprIkPosition::fetch(LinkKinematicsKit* kinematicsKit)
{
    if(kinematicsKit->hasJointPath()){
        T = kinematicsKit->endPosition();
        baseFrameId_ = kinematicsKit->currentBaseFrameId();
        offsetFrameId_ = kinematicsKit->currentOffsetFrameId();
        referenceRpy_ = rpyFromRot(T.linear(), kinematicsKit->referenceRpy());
        configuration_ = kinematicsKit->currentConfigurationType();
        //! \todo set phase here
        return true;
    }
    return false;
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
    
    MprPosition::write(archive);
    
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


bool MprFkPosition::fetch(LinkKinematicsKit* kinematicsKit)
{
    auto path = kinematicsKit->jointPath();
    numJoints_ = std::min(path->numJoints(), MaxNumJoints);
    int i;
    for(i = 0; i < numJoints_; ++i){
        auto joint = path->joint(i);
        jointDisplacements_[i] = joint->q();
        prismaticJointFlags_[i] = joint->isPrismaticJoint();
    }
    for( ; i < MaxNumJoints; ++i){
        jointDisplacements_[i] = 0.0;
        prismaticJointFlags_[i] = false;
    }
    return true;
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
