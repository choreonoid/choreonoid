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

    RootItem* rootItem();

    void setDragDropEnabled(bool on);
    void setCheckColumnShown(bool on);
    void setVisibleItemPredicate(std::function<bool(Item* item, bool isTopLevelItem)> pred);

    template<class ItemType>
    void setContextMenuFunctionFor(
        std::function<void(ItemType* item, MenuManager& menuManager, ItemFunctionDispatcher menuFunction)> func){
        setContextMenuFunctionFor(
            typeid(ItemType),
            [func](Item* item, MenuManager& menuManager, ItemFunctionDispatcher menuFunction){
                func(static_cast<ItemType*>(item), menuManager, menuFunction);
            });
    }

    void updateTreeWidgetItems();
    void setExpanded(Item* item, bool on = true);

    ItemList<> selectedItems() const;
    template <class ItemType> ItemList<ItemType> selectedItems() {
        return selectedItems();
    }
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
