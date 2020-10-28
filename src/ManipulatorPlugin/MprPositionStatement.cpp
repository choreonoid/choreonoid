#include "MprPositionStatement.h"
#include "MprStatementRegistration.h"
#include <cnoid/LinkKinematicsKit>
#include <cnoid/ValueTree>
#include <fmt/format.h>

using namespace std;
using namespace cnoid;
using fmt::format;


MprPositionStatement::MprPositionStatement()
{

}


MprPositionStatement::MprPositionStatement(const MprPositionStatement& org)
    : MprStatement(org),
      positionId_(org.positionId_)
{

}


Referenced* MprPositionStatement::doClone(CloneMap*) const
{
    return new MprPositionStatement(*this);
}
    

std::string MprPositionStatement::label(int index) const
{
    if(index == 0){
        return "Position";

    } else if(index == 1){
        return positionLabel();
    }
    return string();
}


std::string MprPositionStatement::positionLabel() const
{
    if(positionId_.isInt()){
        return format("P{0}", positionId_.toInt());
    } else {
        return positionId_.toString();
    }
}


const MprPosition* MprPositionStatement::position(const MprPositionList* positions) const
{
    return positions->findPosition(positionId_);
}


MprPosition* MprPositionStatement::position(MprPositionList* positions) const
{
    return positions->findPosition(positionId_);
}


bool MprPositionStatement::read(MprProgram* program, const Mapping& archive)
{
    string symbol;
    positionId_.read(archive, "position");
    return true;
}


bool MprPositionStatement::write(Mapping& archive) const
{
    archive.write("position", positionId_.label());
    return true;
}


MprTagTraceStatement::MprTagTraceStatement()
    : T_tags(Position::Identity()),
      baseFrameId_(0),
      offsetFrameId_(0)
{
    positionList_ = new MprPositionList;
}


MprTagTraceStatement::MprTagTraceStatement(const MprTagTraceStatement& org, CloneMap* cloneMap)
    : MprStructuredStatement(org, cloneMap),
      tagGroup_(org.tagGroup_),
      T_tags(org.T_tags),
      baseFrameId_(org.baseFrameId_),
      offsetFrameId_(org.offsetFrameId_)
{
    if(cloneMap){
        positionList_ = org.positionList_->clone(*cloneMap);
    } else {
        positionList_ = org.positionList_->clone();
    }
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
    return MprStructuredStatement::read(program, archive);
}


bool MprTagTraceStatement::write(Mapping& archive) const
{
    return MprStructuredStatement::write(archive);
}


namespace {

struct StatementTypeRegistration {
    StatementTypeRegistration(){
        MprStatementRegistration()
            .registerAbstractType<MprPositionStatement, MprStatement>()
            .registerAbstractType<MprTagTraceStatement, MprStatement>()
            ;
    }
} registration;

}
