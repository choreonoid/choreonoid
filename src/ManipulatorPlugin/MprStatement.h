#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_STATEMENT_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_STATEMENT_H

#include <cnoid/CloneableReferenced>
#include <cnoid/HierarchicalClassRegistry>
#include <cnoid/PolymorphicFunctionSet>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class MprProgram;
typedef ref_ptr<MprProgram> MprProgramPtr;

class Mapping;

class CNOID_EXPORT MprStatement : public CloneableReferenced
{
public:
    int classId() const {
        if(classId_ < 0) validateClassId();
        return classId_;
    }
    
    MprStatement* clone() const {
        return static_cast<MprStatement*>(doClone(nullptr));
    }
    MprStatement* clone(CloneMap& cloneMap) const {
        return static_cast<MprStatement*>(doClone(&cloneMap));
    }

    MprProgram* holderProgram() const;
    MprProgram* topLevelProgram() const;

    virtual MprProgram* getLowerLevelProgram();

    void notifyUpdate();

    virtual std::string label(int index) const = 0;
    virtual bool read(MprProgram* program, const Mapping& archive) = 0;
    virtual bool write(Mapping& archive) const = 0;

protected:
    MprStatement();
    MprStatement(const MprStatement& org);
    
private:
    mutable int classId_;
    weak_ref_ptr<MprProgram> holderProgram_;

    void validateClassId() const;

    friend class MprProgram;
};

typedef ref_ptr<MprStatement> MprStatementPtr;


class CNOID_EXPORT MprStatementClassRegistry : public HierarchicalClassRegistry<MprStatement>
{
public:
    static MprStatementClassRegistry& instance();

private:
    MprStatementClassRegistry();
};


class CNOID_EXPORT PolymorphicMprStatementFunctionSet : public PolymorphicFunctionSet<MprStatement>
{
public:
    PolymorphicMprStatementFunctionSet();
};

typedef PolymorphicMprStatementFunctionSet::Dispatcher MprStatementFunctionDispatcher;

}

#endif
