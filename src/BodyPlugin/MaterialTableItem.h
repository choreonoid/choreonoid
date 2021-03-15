#ifndef CNOID_BODY_PLUGIN_MATERIAL_TABLE_ITEM_H
#define CNOID_BODY_PLUGIN_MATERIAL_TABLE_ITEM_H

#include <cnoid/Item>
#include <cnoid/MaterialTable>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MaterialTableItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);

    MaterialTableItem();
    MaterialTableItem(const MaterialTableItem& org);

    MaterialTable* materialTable() { return materialTable_; }

protected:
    virtual Item* doDuplicate() const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    MaterialTablePtr materialTable_;
};

}

#endif
