#include "ManipulatorStatementRegistration.h"
#include <unordered_map>
#include <mutex>
#include <typeindex>

using namespace std;
using namespace cnoid;

namespace {

typedef unordered_map<string, ManipulatorStatementRegistration::FactoryFunction> FactoryMap;
FactoryMap stdFactoryMap;
unordered_map<string, FactoryMap> moduleFactoryMap;

struct StatementTypeInfo {
    string fullTypeName;

    StatementTypeInfo(const string& moduleName, const string& typeName){
        if(moduleName.empty()){
            fullTypeName = typeName;
        } else {
            fullTypeName = moduleName + ":" + typeName;
        }
    }
};

typedef unordered_map<type_index, StatementTypeInfo> StatementTypeInfoMap;
StatementTypeInfoMap statementTypeInfoMap;

mutex factoryMutex;
string emptyString;

}

namespace cnoid {

class ManipulatorStatementRegistration::Impl
{
public:
    string moduleName;
    FactoryMap& factoryMap;
    Impl(const char* moduleName, FactoryMap& factoryMap)
        : moduleName(moduleName),
          factoryMap(factoryMap) { }
};

}


ManipulatorStatementRegistration::ManipulatorStatementRegistration()
{
    lock_guard<mutex> lock(factoryMutex);
    impl = new Impl("", stdFactoryMap);
}


ManipulatorStatementRegistration::ManipulatorStatementRegistration(const char* module)
{
    lock_guard<mutex> lock(factoryMutex);
    impl = new Impl(module, moduleFactoryMap[module]);
}


void ManipulatorStatementRegistration::registerFactory
(const char* typeName, const std::type_info& type, FactoryFunction factory)
{
    lock_guard<mutex> lock(factoryMutex);
    impl->factoryMap[typeName] = factory;
    statementTypeInfoMap.insert(
        StatementTypeInfoMap::value_type(type, StatementTypeInfo(impl->moduleName, typeName)));
}


static ManipulatorStatement* create(const std::string& type, FactoryMap& factoryMap, bool doSearchOtherMaps)
{
    ManipulatorStatement* statement = nullptr;
    
    auto iter = factoryMap.find(type);
    if(iter != factoryMap.end()){
        auto& factory = iter->second;
        statement = factory();

    } else if(doSearchOtherMaps){
        if(&factoryMap != &stdFactoryMap){
            statement = ::create(type, stdFactoryMap, false);
        }
        if(!statement){
            for(auto& kv : moduleFactoryMap){
                auto& anotherFactoryMap = kv.second;
                if(&factoryMap != &anotherFactoryMap){
                    if(statement = ::create(type, anotherFactoryMap, false)){
                        break;
                    }
                }
            }
        }
    }
        
    return statement;
}


ManipulatorStatement* ManipulatorStatementRegistration::create(const std::string& type)
{
    lock_guard<mutex> lock(factoryMutex);
    return ::create(type, stdFactoryMap, true);
}
    

ManipulatorStatement* ManipulatorStatementRegistration::create(const std::string& type, const std::string& module)
{
    lock_guard<mutex> lock(factoryMutex);
    if(module.empty()){
        return ::create(type, stdFactoryMap, true);
    } else {
        return ::create(type, moduleFactoryMap[module], true);
    }
}


const std::string& ManipulatorStatementRegistration::fullTypeName(const ManipulatorStatement* statement)
{
    lock_guard<mutex> lock(factoryMutex);
    auto iter = statementTypeInfoMap.find(typeid(*statement));
    if(iter != statementTypeInfoMap.end()){
        return iter->second.fullTypeName;
    }
    return emptyString;
}
