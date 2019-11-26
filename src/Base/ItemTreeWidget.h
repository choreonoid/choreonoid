#ifndef CNOID_BASE_ITEM_TREE_WIDGET_H
#define CNOID_BASE_ITEM_TREE_WIDGET_H

#include <QWidget>
#include <functional>
#include "exportdecl.h"

namespace cnoid {

class Item;
class RootItem;
class Archive;

class CNOID_EXPORT ItemTreeWidget : public QWidget
{
public:
    ItemTreeWidget(RootItem* rootItem, QWidget* parent = nullptr);
    ~ItemTreeWidget();

    void updateTreeWidgetItems();
    void setVisibleItemPredicate(std::function<bool(Item* item, bool isTopLevelItem)> pred);

    void setExpanded(Item* item, bool on = true);
    
    bool storeState(Archive& archive);
    bool restoreState(const Archive& archive);

    class Impl;

private:
    Impl* impl;
};

}

#endif
