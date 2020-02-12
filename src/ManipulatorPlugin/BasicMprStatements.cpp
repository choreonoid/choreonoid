#include "BasicMprStatements.h"
#include "MprStatementRegistration.h"
#include "MprProgram.h"
#include "MprVariableSet.h"
#include "MprPositionList.h"
#include <cnoid/CloneMap>
#include <cnoid/ValueTree>
#include <fmt/format.h>

using namespace std;
using namespace cnoid;
using fmt::format;


MprEmptyStatement::MprEmptyStatement()
{

}


MprEmptyStatement::MprEmptyStatement(const MprEmptyStatement& org)
    : MprStatement(org)
{

}


Referenced* MprEmptyStatement::doClone(CloneMap*) const
{
    return new MprEmptyStatement(*this);
}


std::string MprEmptyStatement::label(int index) const
{
    return string();
}


bool MprEmptyStatement::read(MprProgram* program, const Mapping& archive)
{
    return true;
}


bool MprEmptyStatement::write(Mapping& archive) const
{
    return true;
}


MprDummyStatement::MprDummyStatement()
{

}


MprDummyStatement::MprDummyStatement(const MprDummyStatement& org)
    : MprEmptyStatement(org)
{

}


Referenced* MprDummyStatement::doClone(CloneMap*) const
{
    return new MprDummyStatement(*this);
}


std::string MprDummyStatement::label(int index) const
{
    if(index == 0){
        return "---";
    }
    return string();
}


bool MprDummyStatement::write(Mapping& archive) const
{
    return true;
}


MprCommentStatement::MprCommentStatement()
{

}


MprCommentStatement::MprCommentStatement(const MprCommentStatement& org)
    : MprStatement(org),
      comment_(org.comment_)
{

}


Referenced* MprCommentStatement::doClone(CloneMap*) const
{
    return new MprCommentStatement(*this);
}


std::string MprCommentStatement::label(int index) const
{
    if(index == 0){
        return format("# {}", comment_);
    }
    return string();
}


bool MprCommentStatement::read(MprProgram* program, const Mapping& archive)
{
    archive.read("comment", comment_);
    return true;
}
    

bool MprCommentStatement::write(Mapping& archive) const
{
    archive.write("comment", comment_);
    return true;
}


MprStructuredStatement::MprStructuredStatement()
{
    program_ = new MprProgram;
    program_->setHolderStatement(this);
}


MprStructuredStatement::MprStructuredStatement(const MprStructuredStatement& org, CloneMap* cloneMap)
    : MprStatement(org)
{
    if(cloneMap){
        program_ = cloneMap->getClone(org.program_);
    } else {
        program_ = org.program_->clone();
    }

    program_->setHolderStatement(this);
}


MprProgram* MprStructuredStatement::getLowerLevelProgram()
{
    return lowerLevelProgram();
}


bool MprStructuredStatement::read(MprProgram* program, const Mapping& archive)
{
    return program_->read(archive);
}


bool MprStructuredStatement::write(Mapping& archive) const
{
    return program_->write(archive);
}


MprConditionStatement::MprConditionStatement()
{

}


MprConditionStatement::MprConditionStatement(const MprConditionStatement& org, CloneMap* cloneMap)
    : MprStructuredStatement(org, cloneMap),
      condition_(org.condition_)
{

}


bool MprConditionStatement::read(MprProgram* program, const Mapping& archive)
{
    if(MprStructuredStatement::read(program, archive)){
        archive.read("condition", condition_);
        return true;
    }
    return false;
}


bool MprConditionStatement::write(Mapping& archive) const
{
    archive.write("condition", condition_, DOUBLE_QUOTED);
    return MprStructuredStatement::write(archive);
}


MprIfStatement::MprIfStatement()
{

}


MprIfStatement::MprIfStatement(const MprIfStatement& org, CloneMap* cloneMap)
    : MprConditionStatement(org, cloneMap)
{
    
}


Referenced* MprIfStatement::doClone(CloneMap* cloneMap) const
{
    return new MprIfStatement(*this, cloneMap);
}


std::string MprIfStatement::label(int index) const
{
    if(index == 0){
        return "IF";
    } else if(index == 1){
        return condition();
    }
    return string();
}


bool MprIfStatement::read(MprProgram* program, const Mapping& archive)
{
    return MprConditionStatement::read(program, archive);
}


bool MprIfStatement::write(Mapping& archive) const
{
    return MprConditionStatement::write(archive);
}


MprElseStatement::MprElseStatement()
{

}


MprElseStatement::MprElseStatement(const MprElseStatement& org, CloneMap* cloneMap)
    : MprStructuredStatement(org, cloneMap)
{

}


Referenced* MprElseStatement::doClone(CloneMap* cloneMap) const
{
    return new MprElseStatement(*this, cloneMap);
}


std::string MprElseStatement::label(int index) const
{
    if(index == 0){
        return "ELSE";
    }
    return string();
}


bool MprElseStatement::read(MprProgram* program, const Mapping& archive)
{
    return MprStructuredStatement::read(program, archive);
}


bool MprElseStatement::write(Mapping& archive) const
{
    return MprStructuredStatement::write(archive);
}


MprWhileStatement::MprWhileStatement()
{

}


MprWhileStatement::MprWhileStatement(const MprWhileStatement& org, CloneMap* cloneMap)
    : MprConditionStatement(org, cloneMap)
{

}


Referenced* MprWhileStatement::doClone(CloneMap* cloneMap) const
{
    return new MprWhileStatement(*this, cloneMap);
}


std::string MprWhileStatement::label(int index) const
{
    if(index == 0){
        return "WHILE";
    } else if(index == 1){
        return condition();
    }
    return string();
}


bool MprWhileStatement::read(MprProgram* program, const Mapping& archive)
{
    return MprConditionStatement::read(program, archive);
}


bool MprWhileStatement::write(Mapping& archive) const
{
    return MprConditionStatement::write(archive);
}


MprCallStatement::MprCallStatement()
{

}


MprCallStatement::MprCallStatement(const MprCallStatement& org)
    : MprStatement(org),
      programName_(org.programName_)
{

}


Referenced* MprCallStatement::doClone(CloneMap*) const
{
    return new MprCallStatement(*this);
}


std::string MprCallStatement::label(int index) const
{
    if(index == 0){
        return "Call";
    } else if(index == 1){
        return programName_;
    }
    return string();
}


bool MprCallStatement::read(MprProgram* program, const Mapping& archive)
{
    archive.read("program", programName_);
    return true;
}


bool MprCallStatement::write(Mapping& archive) const
{
    archive.write("program", programName_, DOUBLE_QUOTED);
    return true;
}


MprAssignStatement::MprAssignStatement()
{

}


MprAssignStatement::MprAssignStatement(const MprAssignStatement& org)
    : variableId_(org.variableId_),
      expression_(org.expression_)
{

}
      

Referenced* MprAssignStatement::doClone(CloneMap*) const
{
    return new MprAssignStatement(*this);
}


std::string MprAssignStatement::label(int index) const
{
    if(index == 0){
        return "Assign";

    } else if(index == 1){
        if(!variableId_.isValid()){
            return "..... = ";
        } else if(variableId_.isInt()){
            return format("var[{0}] =", variableId_.toInt());
        } else {
            return format("{0} = ", variableId_.label());
        }
    } else if(index == 2){
        if(expression_.empty()){
            return ".....";
        } else {
            return expression_;
        }
    }
    return string();
}


MprVariable* MprAssignStatement::variable(MprVariableSet* variables) const
{
    return variables->findOrCreateVariable(variableId_, 0);
}


bool MprAssignStatement::read(MprProgram* program, const Mapping& archive)
{
    variableId_.read(archive, "variable");
    archive.read("expression", expression_);
    return true;
}


bool MprAssignStatement::write(Mapping& archive) const
{
    variableId_.write(archive, "variable");
    archive.write("expression", expression_, SINGLE_QUOTED);
    return true;
}


MprSignalStatement::MprSignalStatement()
{
    signalIndex_ = 0;
    on_ = false;
}


MprSignalStatement::MprSignalStatement(const MprSignalStatement& org)
{
    signalIndex_ = org.signalIndex_;
    on_ = org.on_;
}


Referenced* MprSignalStatement::doClone(CloneMap*) const
{
    return new MprSignalStatement(*this);
}


std::string MprSignalStatement::label(int index) const
{
    if(index == 0){
        return "Set";
    } else if(index == 1){
        return format("Out[{0}]", signalIndex_);
    } else if(index == 2){
        return on_ ? "on" : "off";
    }
    return string();
}
        

bool MprSignalStatement::read(MprProgram* program, const Mapping& archive)
{
    archive.read("signalIndex", signalIndex_);
    archive.read("on", on_);
    return true;
}


bool MprSignalStatement::write(Mapping& archive) const
{
    archive.write("signalIndex", signalIndex_);
    archive.write("on", on_);
    return true;
}
    

MprDelayStatement::MprDelayStatement()
{
    time_ = 1.0;
}


MprDelayStatement::MprDelayStatement(const MprDelayStatement& org)
    : MprStatement(org)
{
    time_ = org.time_;
}


Referenced* MprDelayStatement::doClone(CloneMap*) const
{
    return new MprDelayStatement(*this);
}
    

std::string MprDelayStatement::label(int index) const
{
    if(index == 0){
        return "Delay";
    } else if(index == 1){
        return format("{0:.3}", time_);
    }
    return string();
}


bool MprDelayStatement::read(MprProgram* program, const Mapping& archive)
{
    archive.read("time", time_);
    return true;
}


bool MprDelayStatement::write(Mapping& archive) const
{
    archive.write("time", time_);
    return true;
}


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


const MprPosition* MprPositionStatement::position(const MprPositionList* positions) const
{
    return positions->findPosition(positionId_);
}


MprPosition* MprPositionStatement::position(MprPositionList* positions) const
{
    return positions->findPosition(positionId_);
}


std::string MprPositionStatement::positionLabel() const
{
    if(positionId_.isInt()){
        return format("P{0}", positionId_.toInt());
    } else {
        return positionId_.toString();
    }
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


namespace {

struct StatementTypeRegistration {
    StatementTypeRegistration(){
        MprStatementRegistration()
            .registerType<MprEmptyStatement, MprStatement>("Empty")
            .registerType<MprDummyStatement, MprEmptyStatement>("Dummy")
            .registerType<MprCommentStatement, MprStatement>("Comment")
            .registerAbstractType<MprStructuredStatement, MprStatement>()
            .registerAbstractType<MprConditionStatement, MprStructuredStatement>()
            .registerType<MprIfStatement, MprConditionStatement>("If")
            .registerType<MprElseStatement, MprStructuredStatement>("Else")
            .registerType<MprWhileStatement, MprConditionStatement>("While")
            .registerType<MprCallStatement, MprStatement>("Call")
            .registerType<MprAssignStatement, MprStatement>("Assign")
            .registerType<MprSignalStatement, MprStatement>("SetSignal")
            .registerType<MprDelayStatement, MprStatement>("Delay")
            .registerAbstractType<MprPositionStatement, MprStatement>()
            ;
    }
} registration;

}
