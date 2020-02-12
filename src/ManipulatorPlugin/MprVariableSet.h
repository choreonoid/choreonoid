#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_VARIABLE_SET_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_VARIABLE_SET_H

#include "MprVariable.h"
#include <cnoid/CloneableReferenced>
#include <cnoid/Signal>
#include <vector>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MprVariableSet : public CloneableReferenced
{
public:
    MprVariableSet* clone(){
        return static_cast<MprVariableSet*>(doClone(nullptr));
    }
    MprVariableSet* clone(CloneMap& cloneMap){
        return static_cast<MprVariableSet*>(doClone(&cloneMap));
    }

    const std::string& name() const { return name_; }

    virtual void setName(const std::string& name);

    virtual int getNumVariables() const = 0;

    virtual MprVariable* getVariableAt(int index) const = 0;
    
    virtual MprVariable* findVariable(const GeneralId& id) const = 0;

    virtual MprVariable* findOrCreateVariable(
        const GeneralId& id, const MprVariable::Value& defaultValue) = 0;

    virtual std::vector<MprVariablePtr> getFindableVariableLists() const = 0;

    virtual bool containsVariableSet(const MprVariableSet* variableSet) const = 0;

    virtual SignalProxy<void(MprVariableSet* variableSet, MprVariable* variable)>
        sigVariableUpdated() = 0;

protected:
    MprVariableSet();
    MprVariableSet(const MprVariableSet& org);

private:
    std::string name_;
};

typedef ref_ptr<MprVariableSet> MprVariableSetPtr;

}

#endif
