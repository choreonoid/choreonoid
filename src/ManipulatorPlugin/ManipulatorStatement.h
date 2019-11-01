#ifndef CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_STATEMENT_H
#define CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_STATEMENT_H

#include <cnoid/CloneableReferenced>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class ManipulatorProgram;
typedef ref_ptr<ManipulatorProgram> ManipulatorProgramPtr;

class Mapping;

class CNOID_EXPORT ManipulatorStatement : public CloneableReferenced
{
public:
    typedef ManipulatorStatement* (*FactoryFunction)();

    template<class StatementType>
    static void registerType(const char* type){
        registerFactory(type, []() -> ManipulatorStatement* { return new StatementType; });
    }

    static ManipulatorStatement* create(const std::string& type);

    ManipulatorStatement* clone() const {
        return static_cast<ManipulatorStatement*>(doClone(nullptr));
    }
    ManipulatorStatement* clone(CloneMap& cloneMap) const {
        return static_cast<ManipulatorStatement*>(doClone(&cloneMap));
    }

    ManipulatorProgram* holderProgram() const;
    ManipulatorProgram* topLevelProgram() const;

    void notifyUpdate();

    virtual std::string label(int index) const = 0;
    virtual bool read(ManipulatorProgram* program, const Mapping& archive) = 0;
    virtual bool write(Mapping& archive) const = 0;

protected:
    ManipulatorStatement();
    ManipulatorStatement(const ManipulatorStatement& org);
    
private:
    static void registerFactory(const char* type, FactoryFunction factory);

    weak_ref_ptr<ManipulatorProgram> holderProgram_;

    friend class ManipulatorProgram;
};

typedef ref_ptr<ManipulatorStatement> ManipulatorStatementPtr;


}

#endif
