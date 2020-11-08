#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_VARIABLE_LIST_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_VARIABLE_LIST_H

#include "MprVariable.h"
#include <cnoid/ClonableReferenced>
#include <cnoid/Signal>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MprVariableList : public ClonableReferenced
{
public:
    enum VariableType {
        GeneralVariable, IntVariable, DoubleVariable, BoolVariable, StringVariable };

    MprVariableList();
    MprVariableList(VariableType variableType);
    MprVariableList(const MprVariableList& org);
    ~MprVariableList();

    MprVariableList* clone() const {
        return static_cast<MprVariableList*>(doClone(nullptr));
    }
    MprVariableList* clone(CloneMap& cloneMap) const {
        return static_cast<MprVariableList*>(doClone(&cloneMap));
    }

    MprVariableList& operator=(const MprVariableList& rhs) = delete;

    void setVariableType(VariableType type);
    void setGeneralVariableValueTypeUnchangeable(bool on);
    void setNumberIdEnabled(bool on);
    void setStringIdEnabled(bool on);

    VariableType variableType() const { return variableType_; }
    bool isGeneralVariableValueTypeUnchangeable() const {
        return isGeneralVariableValueTypeUnchangeable_;
    }
    bool isNumberIdEnabled() const { return isNumberIdEnabled_; }
    bool isStringIdEnabled() const { return isStringIdEnabled_; }

    void clear();
    int numVariables() const;
    MprVariable* variableAt(int index) const;
    int indexOf(MprVariable* variable) const;
    MprVariable* findVariable(const GeneralId& id) const;
    MprVariable::Value defaultValue() const;

    bool insert(int index, MprVariable* variable);
    bool append(MprVariable* variable);
    bool removeAt(int index);

    SignalProxy<void(int index)> sigVariableAdded();
    SignalProxy<void(int index, MprVariable* variable)> sigVariableRemoved();
    SignalProxy<void(int index, int flags)> sigVariableUpdated();

    /**
       @return true if the id is successfully changed. false if the id is not
       changed because anther coordinate frame with the same id is exists.
    */
    bool resetId(MprVariable* variable, const GeneralId& newId);

    void resetIdCounter();
    GeneralId createNextId(int prevId = -1);

    bool read(const Mapping& archive);
    bool write(Mapping& archive) const;    

protected:
    MprVariableList(const MprVariableList& org, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
    
private:
    // Called from the MprVariable implementation
    void notifyVariableUpdate(MprVariable* variable, int flags);

    class Impl;
    Impl* impl;

    VariableType variableType_;
    bool isGeneralVariableValueTypeUnchangeable_;
    bool isNumberIdEnabled_;
    bool isStringIdEnabled_;

    friend class MprVariable;
};

typedef ref_ptr<MprVariableList> MprVariableListPtr;

}

#endif
