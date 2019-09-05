#ifndef CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_STATEMENTS_H
#define CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_STATEMENTS_H

#include <cnoid/Referenced>
#include <cnoid/ManipulatorPosition>
#include "exportdecl.h"

namespace cnoid {

class ManipulatorProgram;
typedef ref_ptr<ManipulatorProgram> ManipulatorProgramPtr;
class ManipulatorProgramCloneMap;

class CNOID_EXPORT ManipulatorStatement : public Referenced
{
public:
    typedef ManipulatorStatement* (*FactoryFunction)();

    template<class StatementType>
    static void registerType(const char* type){
        registerFactory(type, []() -> ManipulatorStatement* { return new StatementType; });
    }

    static ManipulatorStatement* create(const std::string& type);

    virtual ManipulatorStatement* clone(ManipulatorProgramCloneMap& cloneMap) = 0;
    
    virtual const char* label(int index) const = 0;
    virtual bool read(ManipulatorProgram* program, const Mapping& archive) = 0;
    virtual bool write(Mapping& archive) const = 0;

protected:
    ManipulatorStatement();
    ManipulatorStatement(const ManipulatorStatement& org);
    
private:
    // Is this necessary?
    //weak_ref_ptr<ManipulatorProgram> ownerProgram_;

    static void registerFactory(const char* type, FactoryFunction factory);
};

typedef ref_ptr<ManipulatorStatement> ManipulatorStatementPtr;

class CNOID_EXPORT IfStatement : public ManipulatorStatement
{
public:
    IfStatement();
    virtual ManipulatorStatement* clone(ManipulatorProgramCloneMap& cloneMap) override;
    virtual const char* label(int index) const override;
    virtual bool read(ManipulatorProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const;

protected:
    IfStatement(const IfStatement& org);
};
typedef ref_ptr<IfStatement> IfStatementPtr;

class CNOID_EXPORT CallStatement : public ManipulatorStatement
{
public:
    CallStatement();
    virtual ManipulatorStatement* clone(ManipulatorProgramCloneMap& cloneMap) override;
    virtual const char* label(int index) const override;
    virtual bool read(ManipulatorProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const;

protected:
    CallStatement(const CallStatement& org, ManipulatorProgramCloneMap& cloneMap);

private:
    ManipulatorProgramPtr subProgram;
};
typedef ref_ptr<CallStatement> CallStatementPtr;

}

#endif
