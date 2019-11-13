#include "BasicManipulatorStatements.h"
#include "ManipulatorProgram.h"
#include "ManipulatorVariableSet.h"
#include <cnoid/CloneMap>
#include <cnoid/ValueTree>
#include <fmt/format.h>

using namespace std;
using namespace cnoid;
using fmt::format;


EmptyStatement::EmptyStatement()
{

}


EmptyStatement::EmptyStatement(const EmptyStatement& org)
    : ManipulatorStatement(org)
{

}


Referenced* EmptyStatement::doClone(CloneMap*) const
{
    return new EmptyStatement(*this);
}


std::string EmptyStatement::label(int index) const
{
    return string();
}


bool EmptyStatement::read(ManipulatorProgram* program, const Mapping& archive)
{
    return true;
}


bool EmptyStatement::write(Mapping& archive) const
{
    archive.write("type", "Empty");
    return true;
}


DummyStatement::DummyStatement()
{

}


DummyStatement::DummyStatement(const DummyStatement& org)
    : EmptyStatement(org)
{

}


Referenced* DummyStatement::doClone(CloneMap*) const
{
    return new DummyStatement(*this);
}


std::string DummyStatement::label(int index) const
{
    if(index == 0){
        return "---";
    }
    return string();
}


bool DummyStatement::write(Mapping& archive) const
{
    archive.write("type", "Dummy");
    return true;
}


CommentStatement::CommentStatement()
{

}


CommentStatement::CommentStatement(const CommentStatement& org)
    : ManipulatorStatement(org),
      comment_(org.comment_)
{

}


Referenced* CommentStatement::doClone(CloneMap*) const
{
    return new CommentStatement(*this);
}


std::string CommentStatement::label(int index) const
{
    if(index == 0){
        return format("# {}", comment_);
    }
    return string();
}


bool CommentStatement::read(ManipulatorProgram* program, const Mapping& archive)
{
    archive.read("comment", comment_);
    return true;
}
    

bool CommentStatement::write(Mapping& archive) const
{
    archive.write("type", "Comment");
    archive.write("comment", comment_);
    return true;
}


StructuredStatement::StructuredStatement()
{
    program_ = new ManipulatorProgram;
    program_->setHolderStatement(this);
}


StructuredStatement::StructuredStatement(const StructuredStatement& org, CloneMap* cloneMap)
    : ManipulatorStatement(org)
{
    if(cloneMap){
        program_ = cloneMap->getClone(org.program_);
    } else {
        program_ = org.program_->clone();
    }

    program_->setHolderStatement(this);
}


bool StructuredStatement::read(ManipulatorProgram* program, const Mapping& archive)
{
    return program_->read(archive);
}


bool StructuredStatement::write(Mapping& archive) const
{
    return program_->write(archive);
}


ConditionalStatement::ConditionalStatement()
{

}


ConditionalStatement::ConditionalStatement(const ConditionalStatement& org, CloneMap* cloneMap)
    : StructuredStatement(org, cloneMap),
      condition_(org.condition_)
{

}


bool ConditionalStatement::read(ManipulatorProgram* program, const Mapping& archive)
{
    if(StructuredStatement::read(program, archive)){
        archive.read("condition", condition_);
        return true;
    }
    return false;
}


bool ConditionalStatement::write(Mapping& archive) const
{
    archive.write("condition", condition_, DOUBLE_QUOTED);
    return StructuredStatement::write(archive);
}


IfStatement::IfStatement()
{

}


IfStatement::IfStatement(const IfStatement& org, CloneMap* cloneMap)
    : ConditionalStatement(org, cloneMap)
{
    
}


Referenced* IfStatement::doClone(CloneMap* cloneMap) const
{
    return new IfStatement(*this, cloneMap);
}


std::string IfStatement::label(int index) const
{
    if(index == 0){
        return "IF";
    } else if(index == 1){
        return condition();
    }
    return string();
}


bool IfStatement::read(ManipulatorProgram* program, const Mapping& archive)
{
    return ConditionalStatement::read(program, archive);
}


bool IfStatement::write(Mapping& archive) const
{
    archive.write("type", "If");
    return ConditionalStatement::write(archive);
}


ElseStatement::ElseStatement()
{

}


ElseStatement::ElseStatement(const ElseStatement& org, CloneMap* cloneMap)
    : StructuredStatement(org, cloneMap)
{

}


Referenced* ElseStatement::doClone(CloneMap* cloneMap) const
{
    return new ElseStatement(*this, cloneMap);
}


std::string ElseStatement::label(int index) const
{
    if(index == 0){
        return "ELSE";
    }
    return string();
}


bool ElseStatement::read(ManipulatorProgram* program, const Mapping& archive)
{
    return StructuredStatement::read(program, archive);
}


bool ElseStatement::write(Mapping& archive) const
{
    archive.write("type", "Else");
    return StructuredStatement::write(archive);
}


WhileStatement::WhileStatement()
{

}


WhileStatement::WhileStatement(const WhileStatement& org, CloneMap* cloneMap)
    : ConditionalStatement(org, cloneMap)
{

}


Referenced* WhileStatement::doClone(CloneMap* cloneMap) const
{
    return new WhileStatement(*this, cloneMap);
}


std::string WhileStatement::label(int index) const
{
    if(index == 0){
        return "WHILE";
    } else if(index == 1){
        return condition();
    }
    return string();
}


bool WhileStatement::read(ManipulatorProgram* program, const Mapping& archive)
{
    return ConditionalStatement::read(program, archive);
}


bool WhileStatement::write(Mapping& archive) const
{
    archive.write("type", "While");
    return ConditionalStatement::write(archive);
}


CallStatement::CallStatement()
{

}


CallStatement::CallStatement(const CallStatement& org)
    : ManipulatorStatement(org),
      programName_(org.programName_)
{

}


Referenced* CallStatement::doClone(CloneMap*) const
{
    return new CallStatement(*this);
}


std::string CallStatement::label(int index) const
{
    if(index == 0){
        return "Call";
    } else if(index == 1){
        return programName_;
    }
    return string();
}


bool CallStatement::read(ManipulatorProgram* program, const Mapping& archive)
{
    archive.read("program", programName_);
    return true;
}


bool CallStatement::write(Mapping& archive) const
{
    archive.write("type", "Call");
    archive.write("program", programName_, DOUBLE_QUOTED);
    return true;
}


AssignStatement::AssignStatement()
{

}


AssignStatement::AssignStatement(const AssignStatement& org)
    : variableId_(org.variableId_),
      expression_(org.expression_)
{

}
      

Referenced* AssignStatement::doClone(CloneMap*) const
{
    return new AssignStatement(*this);
}


std::string AssignStatement::label(int index) const
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


ManipulatorVariable* AssignStatement::variable(ManipulatorVariableSet* variables) const
{
    return variables->findOrCreateVariable(variableId_, 0);
}


bool AssignStatement::read(ManipulatorProgram* program, const Mapping& archive)
{
    variableId_.read(archive, "variable");
    archive.read("expression", expression_);
    return true;
}


bool AssignStatement::write(Mapping& archive) const
{
    archive.write("type", "Assign");
    variableId_.write(archive, "variable");
    archive.write("expression", expression_, SINGLE_QUOTED);
    return true;
}


SetSignalStatement::SetSignalStatement()
{
    signalIndex_ = 0;
    on_ = false;
}


SetSignalStatement::SetSignalStatement(const SetSignalStatement& org)
{
    signalIndex_ = org.signalIndex_;
    on_ = org.on_;
}


Referenced* SetSignalStatement::doClone(CloneMap*) const
{
    return new SetSignalStatement(*this);
}


std::string SetSignalStatement::label(int index) const
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
        

bool SetSignalStatement::read(ManipulatorProgram* program, const Mapping& archive)
{
    archive.read("signalIndex", signalIndex_);
    archive.read("on", on_);
    return true;
}


bool SetSignalStatement::write(Mapping& archive) const
{
    archive.write("type", "SetSignal");
    archive.write("signalIndex", signalIndex_);
    archive.write("on", on_);
    return true;
}
    

DelayStatement::DelayStatement()
{
    time_ = 1.0;
}


DelayStatement::DelayStatement(const DelayStatement& org)
    : ManipulatorStatement(org)
{
    time_ = org.time_;
}


Referenced* DelayStatement::doClone(CloneMap*) const
{
    return new DelayStatement(*this);
}
    

std::string DelayStatement::label(int index) const
{
    if(index == 0){
        return "Delay";
    } else if(index == 1){
        return format("{0:.3}", time_);
    }
    return string();
}


bool DelayStatement::read(ManipulatorProgram* program, const Mapping& archive)
{
    archive.read("time", time_);
    return true;
}


bool DelayStatement::write(Mapping& archive) const
{
    archive.write("type", "Delay");
    archive.write("time", time_);
    return true;
}


namespace {

struct StatementTypeRegistration {
    StatementTypeRegistration(){
        ManipulatorStatement::registerType<EmptyStatement>("Empty");
        ManipulatorStatement::registerType<DummyStatement>("Dummy");
        ManipulatorStatement::registerType<CommentStatement>("Comment");
        ManipulatorStatement::registerType<IfStatement>("If");
        ManipulatorStatement::registerType<ElseStatement>("Else");
        ManipulatorStatement::registerType<WhileStatement>("While");
        ManipulatorStatement::registerType<CallStatement>("Call");
        ManipulatorStatement::registerType<AssignStatement>("Assign");
        ManipulatorStatement::registerType<SetSignalStatement>("SetSignal");
        ManipulatorStatement::registerType<DelayStatement>("Delay");
    }
} registration;

}
