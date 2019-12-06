#ifndef CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_STATEMENT_REGISTRATION_H
#define CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_STATEMENT_REGISTRATION_H

#include "ManipulatorStatement.h"
#include <typeinfo>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ManipulatorStatementRegistration
{
public:
    typedef ManipulatorStatement* (*FactoryFunction)();

    ManipulatorStatementRegistration();
    ManipulatorStatementRegistration(const char* module);

    template<class StatementType>
    ManipulatorStatementRegistration& registerType(const char* type){
        registerFactory(type, typeid(StatementType), []() -> ManipulatorStatement* { return new StatementType; });
        return *this;
    }

    static ManipulatorStatement* create(const std::string& type);
    static ManipulatorStatement* create(const std::string& type, const std::string& module);
    static const std::string& fullTypeName(const ManipulatorStatement* statement);

private:
    void registerFactory(const char* typeName, const std::type_info& type, FactoryFunction factory);
    class Impl;
    Impl* impl;
};

}
        
#endif

