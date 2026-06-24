#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_GENERAL_VARIABLE_LIST_ITEM_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_GENERAL_VARIABLE_LIST_ITEM_H

#include "MprVariableList.h"
#include <cnoid/Item>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MprGeneralVariableListItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);
    
    MprGeneralVariableListItem();
    MprGeneralVariableListItem(const MprGeneralVariableListItem& org);
    virtual ~MprGeneralVariableListItem();

    void setStartingVariableIdNumber(int id);

    MprVariableList* variableList();
    void setVariableList(MprVariableList* list);

    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

protected:
    virtual Item* doCloneItem(CloneMap* cloneMap) const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<MprGeneralVariableListItem> MprGeneralVariableListItemPtr;

}

#endif
