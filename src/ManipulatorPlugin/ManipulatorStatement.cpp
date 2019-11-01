#include "ManipulatorStatement.h"
#include "ManipulatorProgram.h"
#include "BasicManipulatorStatements.h"
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


ManipulatorProgram* ManipulatorStatement::holderProgram() const
{
    return holderProgram_.lock();
}


ManipulatorProgram* ManipulatorStatement::topLevelProgram() const
{
    auto program = holderProgram();
    if(!program){
        return nullptr;
    }
    return program->topLevelProgram();
}


void ManipulatorStatement::notifyUpdate()
{
    if(auto holder = holderProgram()){
        holder->notifyStatementUpdate(this);
    }
}


