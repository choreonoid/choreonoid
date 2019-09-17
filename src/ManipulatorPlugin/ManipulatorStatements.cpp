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


IfStatement::IfStatement()
{

}


IfStatement::IfStatement(const IfStatement& org)
{

}


ManipulatorStatement* IfStatement::clone(ManipulatorProgramCloneMap& cloneMap)
{
    return new IfStatement(*this);
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


CallStatement::CallStatement()
{

}


CallStatement::CallStatement(const CallStatement& org, ManipulatorProgramCloneMap& cloneMap)
{
    subProgram = cloneMap.getClone(org.subProgram);
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
        ManipulatorStatement::registerType<IfStatement>("If");
        ManipulatorStatement::registerType<CallStatement>("Call");
        ManipulatorStatement::registerType<SetSignalStatement>("SetSignal");
        ManipulatorStatement::registerType<DelayStatement>("Delay");
    }
} registration;

}
