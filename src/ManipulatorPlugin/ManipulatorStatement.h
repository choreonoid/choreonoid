#ifndef CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_STATEMENT_H
#define CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_STATEMENT_H

#include <cnoid/CloneableReferenced>
#include <cnoid/HierarchicalClassRegistry>
#include <cnoid/PolymorphicFunctionSet>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class ManipulatorProgram;
typedef ref_ptr<ManipulatorProgram> ManipulatorProgramPtr;

class Mapping;

class CNOID_EXPORT ManipulatorStatement : public CloneableReferenced
{
public:
    int classId() const {
        if(classId_ < 0) validateClassId();
        return classId_;
    }
    
    ManipulatorStatement* clone() const {
        return static_cast<ManipulatorStatement*>(doClone(nullptr));
    }
    ManipulatorStatement* clone(CloneMap& cloneMap) const {
        return static_cast<ManipulatorStatement*>(doClone(&cloneMap));
    }

    ManipulatorProgram* holderProgram() const;
    ManipulatorProgram* topLevelProgram() const;

    virtual ManipulatorProgram* getLowerLevelProgram();

    void notifyUpdate();

    virtual std::string label(int index) const = 0;
    virtual bool read(ManipulatorProgram* program, const Mapping& archive) = 0;
    virtual bool write(Mapping& archive) const = 0;

protected:
    ManipulatorStatement();
    ManipulatorStatement(const ManipulatorStatement& org);
    
private:
    mutable int classId_;
    weak_ref_ptr<ManipulatorProgram> holderProgram_;

    void validateClassId() const;

    friend class ManipulatorProgram;
};

typedef ref_ptr<ManipulatorStatement> ManipulatorStatementPtr;


class CNOID_EXPORT ManipulatorStatementClassRegistry : public HierarchicalClassRegistry<ManipulatorStatement>
{
public:
    static ManipulatorStatementClassRegistry& instance();

private:
    ManipulatorStatementClassRegistry();
};


class CNOID_EXPORT PolymorphicManipulatorStatementFunctionSet : public PolymorphicFunctionSet<ManipulatorStatement>
{
public:
    PolymorphicManipulatorStatementFunctionSet();
};

typedef PolymorphicManipulatorStatementFunctionSet::Dispatcher ManipulatorStatementFunctionDispatcher;

}

#endif
