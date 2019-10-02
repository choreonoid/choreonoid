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


EmptyStatement::EmptyStatement()
{

}


EmptyStatement::EmptyStatement(const EmptyStatement& org)
    : ManipulatorStatement(org)
{

}


ManipulatorStatement* EmptyStatement::doClone(ManipulatorProgramCloneMap*) const
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


ManipulatorStatement* DummyStatement::doClone(ManipulatorProgramCloneMap*) const
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


bool DummyStatement::read(ManipulatorProgram* program, const Mapping& archive)
{
    return true;
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


ManipulatorStatement* CommentStatement::doClone(ManipulatorProgramCloneMap*) const
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
    program_->append(new DummyStatement, false);
}


StructuredStatement::StructuredStatement(const StructuredStatement& org, ManipulatorProgramCloneMap* cloneMap)
    : ManipulatorStatement(org)
{
    if(cloneMap){
        program_ = cloneMap->getClone(org.program_);
    } else {
        program_ = org.program_->clone();
    }
}


IfStatement::IfStatement()
{

}


IfStatement::IfStatement(const IfStatement& org, ManipulatorProgramCloneMap* cloneMap)
    : StructuredStatement(org, cloneMap)
{
    
}


ManipulatorStatement* IfStatement::doClone(ManipulatorProgramCloneMap* cloneMap) const
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

}


ElseStatement::ElseStatement(const ElseStatement& org, ManipulatorProgramCloneMap* cloneMap)
    : StructuredStatement(org, cloneMap)
{

}


ManipulatorStatement* ElseStatement::doClone(ManipulatorProgramCloneMap* cloneMap) const
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
    return true;
}


bool ElseStatement::write(Mapping& archive) const
{
    archive.write("type", "Else");
    return true;
}


WhileStatement::WhileStatement()
{

}


WhileStatement::WhileStatement(const WhileStatement& org, ManipulatorProgramCloneMap* cloneMap)
    : StructuredStatement(org, cloneMap)
{

}


ManipulatorStatement* WhileStatement::doClone(ManipulatorProgramCloneMap* cloneMap) const
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


CallStatement::CallStatement(const CallStatement& org, ManipulatorProgramCloneMap* cloneMap)
{
    if(cloneMap){
        program_ = cloneMap->getClone(org.program_);
    } else {
        program_ = org.program_;
    }
}


ManipulatorStatement* CallStatement::doClone(ManipulatorProgramCloneMap* cloneMap) const
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


SetSignalStatement::SetSignalStatement(const SetSignalStatement& org)
{
    signalIndex_ = org.signalIndex_;
    on_ = org.on_;
}


SetSignalStatement* SetSignalStatement::doClone(ManipulatorProgramCloneMap*) const
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


ManipulatorStatement* DelayStatement::doClone(ManipulatorProgramCloneMap*) const
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
        ManipulatorStatement::registerType<ElseStatement>("While");
        ManipulatorStatement::registerType<CallStatement>("Call");
        ManipulatorStatement::registerType<SetSignalStatement>("SetSignal");
        ManipulatorStatement::registerType<DelayStatement>("Delay");
    }
} registration;

}
