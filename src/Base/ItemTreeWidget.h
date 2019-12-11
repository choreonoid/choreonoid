#ifndef CNOID_BASE_ITEM_TREE_WIDGET_H
#define CNOID_BASE_ITEM_TREE_WIDGET_H

#include "ItemList.h"
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

    void setContextMenuFunction(std::function<void(MenuManager& menuManager)> func);

    template<class ItemType>
    void setContextMenuFunctionFor(std::function<void(ItemType* item, MenuManager& menuManager)> func){
        setContextMenuFunction(
            typeid(ItemType),
            [func](Item* item, MenuManager& menuManager){
                func(static_cast<ItemType*>(item), menuManager);
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
    void setContextMenuFunctionFor(const std::type_info& type, std::function<void(Item* item, MenuManager& menuManager)> func);
    
    Impl* impl;
};

}

#endif
