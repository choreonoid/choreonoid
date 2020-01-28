#ifndef CNOID_BASE_ITEM_TREE_VIEW_H
#define CNOID_BASE_ITEM_TREE_VIEW_H

#include "View.h"
#include "RootItem.h"
#include "ItemTreeWidget.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ItemTreeView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);

    static ItemTreeView* instance();

     // deprecated
    static ItemTreeView* mainInstance() {
        return instance();
    }

    ItemTreeView();
    ~ItemTreeView();

    ItemTreeWidget* itemTreeWidget();

    template<class ItemType>
    void customizeContextMenu(
        std::function<void(ItemType* item, MenuManager& menuManager, ItemFunctionDispatcher menuFunction)> func){
        itemTreeWidget()->customizeContextMenu(func);
    }

    /*
       All the following functions are deprecated.
       Use the corresponding functions defined in the RootItem class.
    */
    template <class ItemType> ItemList<ItemType> selectedItems() const {
        return RootItem::instance()->selectedItems<ItemType>();
    }
    const ItemList<>& selectedItems() const {
        return RootItem::instance()->selectedItems();
    }
    template <class ItemType> ItemType* selectedItem(bool fromMultiItems = false) const {
        return selectedItems<ItemType>().toSingle(fromMultiItems);
    }
    template <class ItemType> ItemList<ItemType> selectedSubItems(const Item* topItem) const {
        return topItem->selectedDescendantItems<ItemType>();
    }
    template <class ItemType> ItemType* selectedSubItem(Item* topItem, bool fromMultiItems = false) const {
        return selectedSubItems<ItemType>(topItem).toSingle(fromMultiItems);
    }
    bool isItemSelected(const Item* item) const {
        return item->isSelected();
    }
    bool selectItem(Item* item, bool select = true) {
        item->setSelected(select);
        return true;
    }
    void selectAllItems() {
        RootItem::instance()->setSubTreeItemsSelected(true);
    }
    void clearSelection() {
        RootItem::instance()->setSubTreeItemsSelected(false);
    }

    enum CheckId { ID_ANY = Item::LogicalSumOfAllChecks };
        
    template <class ItemType> ItemList<ItemType> checkedItems(int checkId = Item::PrimaryCheck) const {
        return RootItem::instance()->checkedItems<ItemType>(checkId);
    }
    bool isItemChecked(const Item* item, int checkId = Item::PrimaryCheck) const {
        return item->isChecked(checkId);
    }
    bool checkItem(Item* item, bool on = true, int checkId = Item::PrimaryCheck) {
        item->setChecked(checkId, on);
        return true;
    }
    SignalProxy<void(const ItemList<>&)> sigSelectionChanged() const {
        return RootItem::instance()->sigSelectedItemsChanged();
    }
    SignalProxy<void(Item* item, bool isChecked)> sigCheckToggled(int checkId = Item::PrimaryCheck) const {
        return RootItem::instance()->sigCheckToggled(checkId);
    }
    SignalProxy<void(bool isChecked)> sigCheckToggled(Item* item, int checkId = Item::PrimaryCheck) const {
        return item->sigCheckToggled(checkId);
    }

protected:
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;

private:
    class Impl;
    Impl* impl;
};

}

#endif
