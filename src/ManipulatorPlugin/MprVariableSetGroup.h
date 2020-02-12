#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_VARIABLE_SET_GROUP_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_VARIABLE_SET_GROUP_H

#include "MprVariableSet.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MprVariableSetGroup : public MprVariableSet
{
public:
    MprVariableSetGroup();
    MprVariableSetGroup(const MprVariableSetGroup& org);

    void clear();
    int numVariableSets() const;
    MprVariableSet* variableSet(int index) const;
    void addVariableSet(MprVariableSet* variableSet);

    virtual int getNumVariables() const override;
    virtual MprVariable* getVariableAt(int index) const override;
    virtual MprVariable* findVariable(const GeneralId& id) const override;
    virtual MprVariable* findOrCreateVariable(
        const GeneralId& id, const MprVariable::Value& defaultValue) override;
    virtual std::vector<MprVariablePtr> getFindableVariableLists() const override;
    virtual bool containsVariableSet(const MprVariableSet* variableSet) const override;
    virtual SignalProxy<void(MprVariableSet* variableSet, MprVariable* variable)>
        sigVariableUpdated() override;

protected:
    MprVariableSetGroup(const MprVariableSetGroup& org, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
    
private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<MprVariableSetGroup> MprVariableSetGroupPtr;

}

#endif
