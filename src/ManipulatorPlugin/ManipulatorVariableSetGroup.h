#ifndef CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_VARIABLE_SET_GROUP_H
#define CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_VARIABLE_SET_GROUP_H

#include "ManipulatorVariableSet.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ManipulatorVariableSetGroup : public ManipulatorVariableSet
{
public:
    ManipulatorVariableSetGroup();
    ManipulatorVariableSetGroup(const ManipulatorVariableSetGroup& org);

    void clear();
    int numVariableSets() const;
    ManipulatorVariableSet* variableSet(int index) const;
    void addVariableSet(ManipulatorVariableSet* variableSet);

    virtual int getNumVariables() const override;
    virtual ManipulatorVariable* getVariableAt(int index) const override;
    virtual ManipulatorVariable* findVariable(const GeneralId& id) const override;
    virtual ManipulatorVariable* findOrCreateVariable(
        const GeneralId& id, const ManipulatorVariable::Value& defaultValue) override;
    virtual std::vector<ManipulatorVariablePtr> getFindableVariableLists() const override;
    virtual bool containsVariableSet(const ManipulatorVariableSet* variableSet) const override;
    virtual SignalProxy<void(ManipulatorVariableSet* variableSet, ManipulatorVariable* variable)>
        sigVariableUpdated() override;

protected:
    ManipulatorVariableSetGroup(const ManipulatorVariableSetGroup& org, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
    
private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<ManipulatorVariableSetGroup> ManipulatorVariableSetGroupPtr;

}

#endif
