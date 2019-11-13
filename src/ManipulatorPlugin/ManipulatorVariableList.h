#ifndef CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_VARIABLE_LIST_H
#define CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_VARIABLE_LIST_H

#include "ManipulatorVariableSet.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ManipulatorVariableList : public ManipulatorVariableSet
{
public:
    ManipulatorVariableList();
    ManipulatorVariableList(const ManipulatorVariableList& org);
    ~ManipulatorVariableList();

    void setStringIdEnabled(bool on);
    bool isStringIdEnabled() const;
    
    void clear();
    int numVariables() const;
    ManipulatorVariable* variableAt(int index) const;
    int indexOf(ManipulatorVariable* variable) const;

    bool insert(int index, ManipulatorVariable* variable);
    bool append(ManipulatorVariable* variable);
    void removeAt(int index);

    /**
       @return true if the id is successfully changed. false if the id is not
       changed because anther coordinate frame with the same id is exists.
    */
    bool resetId(ManipulatorVariable* variable, const GeneralId& newId);

    void resetIdCounter();
    GeneralId createNextId(int prevId = -1);

    virtual int getNumVariables() const override;
    virtual ManipulatorVariable* getVariableAt(int index) const override;
    virtual ManipulatorVariable* findVariable(const GeneralId& id) const override;
    virtual ManipulatorVariable* findOrCreateVariable(
        const GeneralId& id, const ManipulatorVariable::Value& defaultValue) override;
    virtual std::vector<ManipulatorVariablePtr> getFindableVariableLists() const override;
    virtual bool containsVariableSet(const ManipulatorVariableSet* variableSet) const override;
    virtual SignalProxy<void(ManipulatorVariableSet* variableSet, ManipulatorVariable* variable)>
        sigVariableUpdated() override;
    
    void notifyVariableUpdate(ManipulatorVariable* variable);

    bool read(const Mapping& archive);
    bool write(Mapping& archive) const;    

protected:
    ManipulatorVariableList(const ManipulatorVariableList& org, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
    
private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<ManipulatorVariableList> ManipulatorVariableListPtr;

}

#endif
