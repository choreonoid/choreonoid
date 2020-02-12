#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_VARIABLE_LIST_ITEM_BASE_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_VARIABLE_LIST_ITEM_BASE_H

#include <cnoid/Item>
#include "exportdecl.h"

namespace cnoid {

class MprVariableList;

class CNOID_EXPORT MprVariableListItemBase : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);
    
    MprVariableListItemBase();
    MprVariableListItemBase(const MprVariableListItemBase& org);
    virtual ~MprVariableListItemBase();

    MprVariableList* variableList();
    const MprVariableList* variableList() const;

    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

protected:
    virtual Item* doDuplicate() const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<MprVariableListItemBase> MprVariableListItemBasePtr;

}

#endif
