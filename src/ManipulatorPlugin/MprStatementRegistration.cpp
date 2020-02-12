#include "MprStatementRegistration.h"
#include <unordered_map>
#include <mutex>
#include <typeindex>

using namespace std;
using namespace cnoid;

namespace {

typedef unordered_map<string, MprStatementRegistration::FactoryFunction> FactoryMap;
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

class MprStatementRegistration::Impl
{
public:
    MprStatementClassRegistry& classRegistry;
    string moduleName;
    FactoryMap& factoryMap;
    Impl(const char* moduleName, FactoryMap& factoryMap)
        : classRegistry(MprStatementClassRegistry::instance()),
          moduleName(moduleName),
          factoryMap(factoryMap) { }
};

}


MprStatementRegistration::MprStatementRegistration()
{
    lock_guard<mutex> lock(factoryMutex);
    impl = new Impl("", stdFactoryMap);
}


MprStatementRegistration::MprStatementRegistration(const char* module)
{
    lock_guard<mutex> lock(factoryMutex);
    impl = new Impl(module, moduleFactoryMap[module]);
}


void MprStatementRegistration::registerFactory_
(const char* typeName, const std::type_info& type, const std::type_info& superType, FactoryFunction factory)
{
    impl->classRegistry.registerClassAsTypeInfo(type, superType);

    statementTypeInfoMap.insert(
        StatementTypeInfoMap::value_type(type, StatementTypeInfo(impl->moduleName, typeName)));
    
    if(factory){
        lock_guard<mutex> lock(factoryMutex);
        impl->factoryMap[typeName] = factory;
    }
}


static MprStatement* create(const std::string& type, FactoryMap& factoryMap, bool doSearchOtherMaps)
{
    MprStatement* statement = nullptr;
    
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


MprStatement* MprStatementRegistration::create(const std::string& type)
{
    lock_guard<mutex> lock(factoryMutex);
    return ::create(type, stdFactoryMap, true);
}
    

MprStatement* MprStatementRegistration::create(const std::string& type, const std::string& module)
{
    lock_guard<mutex> lock(factoryMutex);
    if(module.empty()){
        return ::create(type, stdFactoryMap, true);
    } else {
        return ::create(type, moduleFactoryMap[module], true);
    }
}


const std::string& MprStatementRegistration::fullTypeName(const MprStatement* statement)
{
    lock_guard<mutex> lock(factoryMutex);
    auto iter = statementTypeInfoMap.find(typeid(*statement));
    if(iter != statementTypeInfoMap.end()){
        return iter->second.fullTypeName;
    }
    return emptyString;
}
