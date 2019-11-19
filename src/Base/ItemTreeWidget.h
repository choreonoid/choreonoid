#ifndef CNOID_BASE_ITEM_TREE_WIDGET_H
#define CNOID_BASE_ITEM_TREE_WIDGET_H

#include "TreeWidget.h"
#include "ItemList.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ItemTreeWidget : public TreeWidget
{
public:
    ItemTreeWidget();
    ~ItemTreeWidget();

    void addTopLevelItem(Item* item);

    bool storeState(Archive& archive) override;
    bool restoreState(const Archive& archive) override;

private:
    
};

}

#endif
