#ifndef CNOID_BASE_ITEM_TREE_WIDGET_H
#define CNOID_BASE_ITEM_TREE_WIDGET_H

#include "ItemList.h"
#include "PolymorphicItemFunctionSet.h"
#include <QWidget>
#include <functional>
#include "exportdecl.h"

class QTreeWidgetItem;

namespace cnoid {

class Item;
class MenuManager;
class Archive;

class CNOID_EXPORT ItemTreeWidget : public QWidget
{
public:
    class Impl;

    ItemTreeWidget(QWidget* parent = nullptr);
    ~ItemTreeWidget();

    RootItem* projectRootItem();
    Item* findRootItem();
    template<class ItemType>
    ItemType* findRootItem(){ return dynamic_cast<ItemType*>(findRootItem()); }
    Item* findOrCreateRootItem();
    template<class ItemType>
    ItemType* findOrCreateRootItem(){ return dynamic_cast<ItemType*>(findRootItem()); }
    void setRootItem(Item* item);
    void setRootItemUpdateFunction(std::function<Item*(bool doCreate)> func);
    void setRootItemVisible(bool on);
    bool isRootItemVisible() const;
    void setDragDropEnabled(bool on);
    void setCheckColumnShown(bool on);

    template<class ItemType>
    void customizeVisibility(std::function<bool(ItemType* item, bool isTopLevelItemCandidate)> func){
        customizeVisibility_(
            typeid(ItemType),
            [func](Item* item, bool isTopLevelItemCandidate){
                return func(static_cast<ItemType*>(item), isTopLevelItemCandidate);
            });
    }

    class CNOID_EXPORT Display {
    public:
        QBrush foreground() const;
        void setForeground(const QBrush& brush);
        QBrush background() const;
        void setBackground(const QBrush& brush);
        QFont font() const;
        void setFont(const QFont& font);
        QIcon icon() const;
        void setIcon(const QIcon& icon);
        void setToolTip(const std::string& toolTip);
        void setStatusTip(const std::string& statusTip);
        void setNameEditable(bool on);
        void setDisabled(bool on);
    private:
        QTreeWidgetItem* item;
        friend class Impl;
    };

    template<class ItemType>
    void customizeDisplay(std::function<void(ItemType* item, Display& display)> func){
        customizeDisplay_(
            typeid(ItemType),
            [func](Item* item, Display& display){
                return func(static_cast<ItemType*>(item), display);
            });
    }

    template<class ItemType>
    void customizePositionAcceptance(
        std::function<bool(ItemType* item, Item* parentItem)> func){
        customizePositionAcceptance_(
            typeid(ItemType),
            [func](Item* item, Item* parentItem){
                return func(static_cast<ItemType*>(item), parentItem);
            });
    }

    bool checkPositionAcceptance(Item* item, Item* parentItem) const;

    void customizeRootContextMenu(std::function<void(MenuManager& menuManager)> func);
    
    template<class ItemType>
    void customizeContextMenu(
        std::function<void(ItemType* item, MenuManager& menuManager, ItemFunctionDispatcher menuFunction)> func){
        customizeContextMenu_(
            typeid(ItemType),
            [func](Item* item, MenuManager& menuManager, ItemFunctionDispatcher menuFunction){
                func(static_cast<ItemType*>(item), menuManager, menuFunction);
            });
    }

    void updateTreeWidgetItems();
    void setExpanded(Item* item, bool on = true);
    void editItemName(Item* item);

    ItemList<> getItems() const;
    template <class ItemType> ItemList<ItemType> getItems() {
        return getItems();
    }

    void setSelectionSyncGroup(const std::string& id);

    SignalProxy<void(const ItemList<>&)> sigSelectionChanged();

    ItemList<> getSelectedItems() const;
    template <class ItemType> ItemList<ItemType> getSelectedItems() {
        return getSelectedItems();
    }

    bool selectOnly(Item* item);
    void selectAllItems();
    void clearSelection();
    void setSelectedItemsChecked(bool on);
    void toggleSelectedItemChecks();
    void cutSelectedItems();
    void copySelectedItems();
    void copySelectedItemsWithSubTrees();
    bool pasteItems(bool doCheckPositionAcceptance = true);
    bool checkCuttable(Item* item) const;
    bool checkCopiable(Item* item) const;
    bool checkPastable(Item* pasteParentItem) const;

    bool storeState(Archive& archive);
    bool restoreState(const Archive& archive);

private:
    void customizeVisibility_(
        const std::type_info& type, std::function<bool(Item* item, bool isTopLevelItemCandidate)> func);
    void customizeDisplay_(
        const std::type_info& type, std::function<void(Item* item, Display& display)> func);
    void customizePositionAcceptance_(
        const std::type_info& type, std::function<bool(Item* item, Item* parentItem)> func);
    void customizeContextMenu_(
        const std::type_info& type,
        std::function<void(Item* item, MenuManager& menuManager, ItemFunctionDispatcher menuFunction)> func);
    
    Impl* impl;
};

}

#endif
