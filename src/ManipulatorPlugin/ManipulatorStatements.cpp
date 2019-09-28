#include "ManipulatorStatements.h"
#include "ManipulatorProgram.h"
#include <cnoid/ValueTree>
#include <fmt/format.h>
#include <unordered_map>
#include <mutex>

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

unordered_map<string, ManipulatorStatement::FactoryFunction> factoryMap;
mutex factoryMutex;

}


void ManipulatorStatement::registerFactory(const char* type, FactoryFunction factory)
{
    lock_guard<mutex> lock(factoryMutex);
    factoryMap[type] = factory;
}


ManipulatorStatement* ManipulatorStatement::create(const std::string& type)
{
    lock_guard<mutex> lock(factoryMutex);
    auto iter = factoryMap.find(type);
    if(iter != factoryMap.end()){
        auto& factory = iter->second;
        return factory();
    }
    return nullptr;
}


ManipulatorStatement::ManipulatorStatement()
{

}


ManipulatorStatement::ManipulatorStatement(const ManipulatorStatement& org)
{

}


int ManipulatorStatement::labelSpan(int /* index */) const
{
    return 1;
}


EmptyStatement::EmptyStatement()
{

}


EmptyStatement::EmptyStatement(const EmptyStatement& org)
    : ManipulatorStatement(org)
{

}


ManipulatorStatement* EmptyStatement::clone(ManipulatorProgramCloneMap&)
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


CommentStatement::CommentStatement()
{

}


CommentStatement::CommentStatement(const CommentStatement& org)
    : ManipulatorStatement(org),
      comment_(org.comment_)
{

}


ManipulatorStatement* CommentStatement::clone(ManipulatorProgramCloneMap& cloneMap)
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


int CommentStatement::labelSpan(int index) const
{
    if(index == 0){
        return MaxNumLabels;
    }
    return 0;
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

}


StructuredStatement::StructuredStatement(const StructuredStatement& org)
    : ManipulatorStatement(org)
{

}


IfStatement::IfStatement()
{
    program_ = new ManipulatorProgram;
    program_->append(new EmptyStatement);
}


IfStatement::IfStatement(const IfStatement& org, ManipulatorProgramCloneMap& cloneMap)
    : StructuredStatement(org)
{
    program_ = cloneMap.getClone(org.program_);
}


ManipulatorStatement* IfStatement::clone(ManipulatorProgramCloneMap& cloneMap)
{
    return new IfStatement(*this, cloneMap);
}


std::string IfStatement::label(int index) const
{
    if(index == 0){
        return "IF";
    }
    return string();
}


ManipulatorProgram* IfStatement::getLowerLevelProgram() const
{
    return program_;
}
    

bool IfStatement::read(ManipulatorProgram* program, const Mapping& archive)
{
    return true;
}


bool IfStatement::write(Mapping& archive) const
{
    archive.write("type", "If");
    return true;
}


ElseStatement::ElseStatement()
{
    program_ = new ManipulatorProgram;
    program_->append(new EmptyStatement);
}


ElseStatement::ElseStatement(const ElseStatement& org, ManipulatorProgramCloneMap& cloneMap)
    : StructuredStatement(org)
{
    program_ = cloneMap.getClone(org.program_);
}


ManipulatorStatement* ElseStatement::clone(ManipulatorProgramCloneMap& cloneMap)
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


ManipulatorProgram* ElseStatement::getLowerLevelProgram() const
{
    return program_;
}
    

bool ElseStatement::read(ManipulatorProgram* program, const Mapping& archive)
{
    return true;
}


bool ElseStatement::write(Mapping& archive) const
{
    archive.write("type", "Else");
    return true;
}


WhileStatement::WhileStatement()
{
    program_ = new ManipulatorProgram;
    program_->append(new EmptyStatement);
}


WhileStatement::WhileStatement(const WhileStatement& org, ManipulatorProgramCloneMap& cloneMap)
    : StructuredStatement(org)
{
    program_ = cloneMap.getClone(org.program_);
}


ManipulatorStatement* WhileStatement::clone(ManipulatorProgramCloneMap& cloneMap)
{
    return new WhileStatement(*this, cloneMap);
}


std::string WhileStatement::label(int index) const
{
    if(index == 0){
        return "WHILE";
    }
    return string();
}


ManipulatorProgram* WhileStatement::getLowerLevelProgram() const
{
    return program_;
}
    

bool WhileStatement::read(ManipulatorProgram* program, const Mapping& archive)
{
    return true;
}


bool WhileStatement::write(Mapping& archive) const
{
    archive.write("type", "While");
    return true;
}


CallStatement::CallStatement()
{

}


CallStatement::CallStatement(const CallStatement& org, ManipulatorProgramCloneMap& cloneMap)
{
    program_ = cloneMap.getClone(org.program_);
}


ManipulatorStatement* CallStatement::clone(ManipulatorProgramCloneMap& cloneMap)
{
    return new CallStatement(*this, cloneMap);
}


std::string CallStatement::label(int index) const
{
    if(index == 0){
        return "Call";
    }
    return string();
}


bool CallStatement::read(ManipulatorProgram* program, const Mapping& archive)
{
    return true;
}


bool CallStatement::write(Mapping& archive) const
{
    archive.write("type", "Call");
    return true;
}


SetSignalStatement::SetSignalStatement()
{
    signalIndex_ = 0;
    on_ = false;
}


SetSignalStatement::SetSignalStatement(const SetSignalStatement& org, ManipulatorProgramCloneMap& cloneMap)
{
    signalIndex_ = org.signalIndex_;
    on_ = org.on_;
}


SetSignalStatement* SetSignalStatement::clone(ManipulatorProgramCloneMap& cloneMap)
{
    return new SetSignalStatement(*this, cloneMap);
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


ManipulatorStatement* DelayStatement::clone(ManipulatorProgramCloneMap&)
{
    return new DelayStatement(*this);
}
    

std::string DelayStatement::label(int index) const
{
    if(index == 0){
        return "Delay";
    } else if(index == 1){
        return std::to_string(time_);
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
        ManipulatorStatement::registerType<CommentStatement>("Comment");
        ManipulatorStatement::registerType<IfStatement>("If");
        ManipulatorStatement::registerType<ElseStatement>("Else");
        ManipulatorStatement::registerType<ElseStatement>("While");
        ManipulatorStatement::registerType<CallStatement>("Call");
        ManipulatorStatement::registerType<SetSignalStatement>("SetSignal");
        ManipulatorStatement::registerType<DelayStatement>("Delay");
    }
} registration;

}
