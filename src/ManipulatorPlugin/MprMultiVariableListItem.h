#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_MULTI_VARIABLE_LIST_ITEM_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_MULTI_VARIABLE_LIST_ITEM_H

#include "MprVariableList.h"
#include <cnoid/Item>
#include "exportdecl.h"

namespace cnoid {

class MprVariableList;

class CNOID_EXPORT MprMultiVariableListItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);
    
    MprMultiVariableListItem();
    MprMultiVariableListItem(const MprMultiVariableListItem& org);
    virtual ~MprMultiVariableListItem();

    void clearVariableLists();
    void setNumVariableList(int n);
    void setVariableList(int index, MprVariableList* list);

    int numVariableLists() const;
    MprVariableList* variableListAt(int index);
    MprVariableList* findVariableList(MprVariableList::VariableType variableType);

    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

protected:
    virtual Item* doDuplicate() const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<MprMultiVariableListItem> MprMultiVariableListItemPtr;

}

#endif
