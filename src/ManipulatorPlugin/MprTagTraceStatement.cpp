#include "MprTagTraceStatement.h"
#include "MprStatementRegistration.h"
#include <cnoid/LinkKinematicsKit>
#include <cnoid/ValueTree>
#include <cnoid/EigenArchive>
#include <cnoid/EigenUtil>

using namespace std;
using namespace cnoid;


MprTagTraceStatement::MprTagTraceStatement()
    : T_tags(Position::Identity()),
      baseFrameId_(0),
      offsetFrameId_(0),
      originalTagGroupId_(-1)
{
    lowerLevelProgram()->setLocalPositionListEnabled(true);
}


MprTagTraceStatement::MprTagTraceStatement(const MprTagTraceStatement& org, CloneMap* cloneMap)
    : MprStructuredStatement(org, cloneMap),
      tagGroup_(org.tagGroup_),
      T_tags(org.T_tags),
      baseFrameId_(org.baseFrameId_),
      offsetFrameId_(org.offsetFrameId_),
      originalTagGroupId_(-1)
{

}


std::string MprTagTraceStatement::label(int index) const
{
    if(index == 0){
        return "Trace";
    } else if(index == 1){
        const auto& name = lowerLevelProgram()->name();
        if(!name.empty()){
            return name;
        } else {
            return "-----";
        }
    }
    return string();
}


void MprTagTraceStatement::setTagGroup(PositionTagGroup* tags)
{
    tagGroup_ = tags;
    originalTagGroupName_.clear();
    originalTagGroupId_ = -1;
}


void MprTagTraceStatement::updateFramesWithCurrentFrames(LinkKinematicsKit* kinematicsKit)
{
    setBaseFrameId(kinematicsKit->currentBaseFrameId());
    setOffsetFrameId(kinematicsKit->currentOffsetFrameId());
}


void MprTagTraceStatement::setGlobalTagGroupPosition(LinkKinematicsKit* kinematicsKit, const Position& T_parent)
{
    auto T_base = kinematicsKit->globalBasePosition(baseFrameId_);
    Position T_tags = T_base.inverse(Eigen::Isometry) * T_parent;
    setTagGroupPosition(T_tags);
}


MprProgram::iterator MprTagTraceStatement::expandTraceStatements()
{
    return holderProgram()->end();
}


bool MprTagTraceStatement::read(MprProgram* program, const Mapping& archive)
{
    if(MprStructuredStatement::read(program, archive)){
        archive.read("tag_group", originalTagGroupName_);
        originalTagGroupId_ = 0;
        Vector3 v;
        if(cnoid::read(archive, "translation", v)){
            T_tags.translation() = v;
        } else {
            T_tags.translation().setZero();
        }
        if(cnoid::read(archive, "rotation", v)){
            T_tags.linear() = rotFromRpy(radian(v));
        } else {
            T_tags.linear().setIdentity();
        }
        baseFrameId_.read(archive, "base_frame");
        offsetFrameId_.read(archive, "offset_frame");
        return true;
    }
    return false;
}


bool MprTagTraceStatement::write(Mapping& archive) const
{
    if(MprStructuredStatement::write(archive)){
        if(tagGroup_ && !tagGroup_->name().empty()){
            archive.write("tag_group", tagGroup_->name());
        }
        archive.setDoubleFormat("%.10g");
        cnoid::write(archive, "translation", Vector3(T_tags.translation()));
        cnoid::write(archive, "rpy", degree(rpyFromRot(T_tags.linear())));
        baseFrameId_.write(archive, "base_frame");
        offsetFrameId_.write(archive, "offset_frame");
        return true;
    }
    return false;
}


namespace {

struct StatementTypeRegistration {
    StatementTypeRegistration(){
        MprStatementRegistration()
            .registerAbstractType<MprTagTraceStatement, MprStatement>();
    }
} registration;

}
