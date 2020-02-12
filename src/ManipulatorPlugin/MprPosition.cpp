#include "MprPosition.h"
#include "MprPositionList.h"
#include <cnoid/Body>
#include <cnoid/LinkKinematicsKit>
#include <cnoid/JointPath>
#include <cnoid/JointPathConfigurationHandler>
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
    if(!owner_){
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


MprPositionList* MprPosition::owner()
{
    return owner_.lock();
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
      toolFrameId_(0)
{
    T.setIdentity();
    referenceRpy_.setZero();
    baseFrameType_ = BodyFrame;
    configuration_ = 0;
    phase_.fill(0);
}


MprIkPosition::MprIkPosition(const MprIkPosition& org)
    : MprPosition(org),
      T(org.T),
      referenceRpy_(org.referenceRpy_),
      baseFrameId_(org.baseFrameId_),
      toolFrameId_(org.toolFrameId_),
      baseFrameType_(org.baseFrameType_),
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


bool MprIkPosition::setCurrentPosition(LinkKinematicsKit* kinematicsKit)
{
    return setCurrentPosition_(kinematicsKit, false);
}


bool MprIkPosition::setCurrentPositionWithCurrentBaseFrameType(LinkKinematicsKit* kinematicsKit)
{
    return setCurrentPosition_(kinematicsKit, true);
}


bool MprIkPosition::setCurrentPosition_(LinkKinematicsKit* kinematicsKit, bool useCurrentBaseFrameType)
{
    auto baseLink = kinematicsKit->baseLink();
    if(!baseLink){
        return false;
    }

    Position T_base;

    if(useCurrentBaseFrameType){
        baseFrameType_ = kinematicsKit->currentBaseFrameType();
    } else {
        baseFrameType_ = BodyFrame;
    }

    if(baseFrameType_ == WorldFrame){
        baseFrameId_ = kinematicsKit->currentWorldFrameId();
        auto baseFrame = kinematicsKit->currentWorldFrame();
        T_base = baseFrame->T();

    } else {
        baseFrameId_ = kinematicsKit->currentBodyFrameId();
        auto baseFrame = kinematicsKit->currentBodyFrame();
        T_base = baseLink->Ta() * baseFrame->T();
    }

    toolFrameId_ = kinematicsKit->currentEndFrameId();
    auto endFrame = kinematicsKit->currentEndFrame();
    Position T_end = kinematicsKit->link()->Ta() * endFrame->T();

    T = T_base.inverse(Eigen::Isometry) * T_end;

    referenceRpy_ = rpyFromRot(T.linear(), kinematicsKit->referenceRpy());

    configuration_ = kinematicsKit->currentConfiguration();

    //! \todo set phase here

    return true;
}


bool MprIkPosition::apply(LinkKinematicsKit* kinematicsKit) const
{
    auto jointPath = kinematicsKit->jointPath();
    if(!jointPath){
        return false;
    }

    if(auto handler = kinematicsKit->configurationHandler()){
        handler->setPreferredConfiguration(configuration_);
    }

    Position T_base;
    if(baseFrameType_ == WorldFrame){
        auto worldFrame = kinematicsKit->worldFrame(baseFrameId_);
        T_base = worldFrame->T();
    } else {
        auto bodyFrame = kinematicsKit->bodyFrame(baseFrameId_);
        T_base = kinematicsKit->baseLink()->Ta() * bodyFrame->T();
    }
    
    Position T_tool = kinematicsKit->endFrame(toolFrameId_)->T();
    Position Ta_end = T_base * T * T_tool.inverse(Eigen::Isometry);
    Position T_end;
    T_end.translation() = Ta_end.translation();
    T_end.linear() = kinematicsKit->link()->calcRfromAttitude(Ta_end.linear());

    kinematicsKit->setReferenceRpy(rpyFromRot(T.linear(), referenceRpy_));

    return jointPath->calcInverseKinematics(T_end);
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
    if(!toolFrameId_.read(archive, "tool_frame")){
        toolFrameId_.read(archive, "toolFrame"); // old
    }

    string type;
    if(!archive.read("base_frame_type", type)){
        archive.read("baseFrameType", type); // old
    }
    if(!type.empty()){
        if(type == "world"){
            baseFrameType_ = WorldFrame;
        } else if(type == "body"){
            baseFrameType_ = BodyFrame;
        }
    }

    configuration_ = 0;
    if(!archive.read("config_index", configuration_)){
        archive.read("configIndex", configuration_); // old
    }

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

    return true;
}


bool MprIkPosition::write(Mapping& archive) const
{
    archive.write("type", "IkPosition");
    
    MprPosition::write(archive);
    
    archive.setDoubleFormat("%.9g");

    cnoid::write(archive, "translation", Vector3(T.translation()));
    cnoid::write(archive, "rotation", degree(rpy()));

    if(baseFrameType_ == WorldFrame){
        archive.write("base_frame_type", "world");
    } else if(baseFrameType_ == BodyFrame){
        archive.write("base_frame_type", "body");
    }

    baseFrameId_.write(archive, "base_frame");
    toolFrameId_.write(archive, "tool_frame");

    archive.write("config_index", configuration_);
    auto& phaseNodes = *archive.createFlowStyleListing("phases");
    for(auto& phase : phase_){
        phaseNodes.append(phase);
    }
    
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


bool MprFkPosition::setCurrentPosition(LinkKinematicsKit* kinematicsKit)
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

    archive.setDoubleFormat("%.9g");

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
