#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_STATEMENT_REGISTRATION_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_STATEMENT_REGISTRATION_H

#include "MprStatement.h"
#include <typeinfo>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MprStatementRegistration
{
public:
    typedef MprStatement* (*FactoryFunction)();

    MprStatementRegistration();
    MprStatementRegistration(const char* module);

    template<class StatementType, class SuperType>
    MprStatementRegistration& registerType(const char* type){
        registerFactory_(type, typeid(StatementType), typeid(SuperType),
                        []() -> MprStatement* { return new StatementType; });
        return *this;
    }
    template<class StatementType, class SuperType>
    MprStatementRegistration& registerAbstractType(){
        registerFactory_("", typeid(StatementType), typeid(SuperType), nullptr);
        return *this;
    }

    static MprStatement* create(const std::string& type);
    static MprStatement* create(const std::string& type, const std::string& module);
    static const std::string& fullTypeName(const MprStatement* statement);

private:
void registerFactory_(
    const char* typeName, const std::type_info& type, const std::type_info& superType, FactoryFunction factory);
    class Impl;
    Impl* impl;
};

}
        
#endif

