#ifndef CNOID_BASE_ITEM_TREE_WIDGET_H
#define CNOID_BASE_ITEM_TREE_WIDGET_H

#include "ItemList.h"
#include "PolymorphicItemFunctionSet.h"
#include <QWidget>
#include <functional>
#include "exportdecl.h"

namespace cnoid {

class Item;
class RootItem;
class MenuManager;
class Archive;

class CNOID_EXPORT ItemTreeWidget : public QWidget
{
public:
    ItemTreeWidget(RootItem* rootItem, QWidget* parent = nullptr);
    ~ItemTreeWidget();

    void updateTreeWidgetItems();
    void setVisibleItemPredicate(std::function<bool(Item* item, bool isTopLevelItem)> pred);

    void setExpanded(Item* item, bool on = true);

    template<class ItemType>
    void setContextMenuFunctionFor(
        std::function<void(ItemType* item, MenuManager& menuManager, ItemFunctionDispatcher menuFunction)> func){
        setContextMenuFunctionFor(
            typeid(ItemType),
            [func](Item* item, MenuManager& menuManager, ItemFunctionDispatcher menuFunction){
                func(static_cast<ItemType*>(item), menuManager, menuFunction);
            });
    }

    ItemList<> selectedItems() const;
    void selectAllItems();
    void clearSelection();
    void setSelectedItemsChecked(bool on);
    void toggleSelectedItemChecks();
    void cutSelectedItems();
    void copySelectedItems();
    void copySelectedItemsWithSubTrees();
    void pasteItems();
    
    bool storeState(Archive& archive);
    bool restoreState(const Archive& archive);

    class Impl;

private:
    void setContextMenuFunctionFor(
        const std::type_info& type,
        std::function<void(Item* item, MenuManager& menuManager, ItemFunctionDispatcher menuFunction)> func);
    
    Impl* impl;
};

}

#endif
