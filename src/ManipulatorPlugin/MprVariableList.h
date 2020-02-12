#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_VARIABLE_LIST_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_VARIABLE_LIST_H

#include "MprVariableSet.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MprVariableList : public MprVariableSet
{
public:
    MprVariableList();
    MprVariableList(const MprVariableList& org);
    ~MprVariableList();

    void setStringIdEnabled(bool on);
    bool isStringIdEnabled() const;
    
    void clear();
    int numVariables() const;
    MprVariable* variableAt(int index) const;
    int indexOf(MprVariable* variable) const;

    bool insert(int index, MprVariable* variable);
    bool append(MprVariable* variable);
    void removeAt(int index);

    /**
       @return true if the id is successfully changed. false if the id is not
       changed because anther coordinate frame with the same id is exists.
    */
    bool resetId(MprVariable* variable, const GeneralId& newId);

    void resetIdCounter();
    GeneralId createNextId(int prevId = -1);

    virtual int getNumVariables() const override;
    virtual MprVariable* getVariableAt(int index) const override;
    virtual MprVariable* findVariable(const GeneralId& id) const override;
    virtual MprVariable* findOrCreateVariable(
        const GeneralId& id, const MprVariable::Value& defaultValue) override;
    virtual std::vector<MprVariablePtr> getFindableVariableLists() const override;
    virtual bool containsVariableSet(const MprVariableSet* variableSet) const override;
    virtual SignalProxy<void(MprVariableSet* variableSet, MprVariable* variable)>
        sigVariableUpdated() override;
    
    void notifyVariableUpdate(MprVariable* variable);

    bool read(const Mapping& archive);
    bool write(Mapping& archive) const;    

protected:
    MprVariableList(const MprVariableList& org, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
    
private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<MprVariableList> MprVariableListPtr;

}

#endif
