#include "ManipulatorStatements.h"
#include "ManipulatorProgram.h"
#include <cnoid/ValueTree>
#include <unordered_map>
#include <mutex>

using namespace std;
using namespace cnoid;

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


const char* IfStatement::label(int index) const
{
    if(index == 0){
        return "IF";
    }
    return nullptr;
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


const char* CallStatement::label(int index) const
{
    if(index == 0){
        return "Call";
    }
    return nullptr;
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

namespace {

struct StatementTypeRegistration {
    StatementTypeRegistration(){
        ManipulatorStatement::registerType<IfStatement>  ("If");
        ManipulatorStatement::registerType<CallStatement>("Call");
    }
} registration;

}
