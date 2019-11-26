#ifndef CNOID_BASE_NEW_ITEM_TREE_VIEW_H
#define CNOID_BASE_NEW_ITEM_TREE_VIEW_H

#include "View.h"
#include "RootItem.h"

namespace cnoid {

class NewItemTreeView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);

    static NewItemTreeView* instance();

     // deprecated
    static NewItemTreeView* mainInstance() {
        return instance();
    }

    NewItemTreeView();
    ~NewItemTreeView();

    void setExpanded(Item* item, bool on = true);

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
        return topItem->selectedDescendants<ItemType>();
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
    template <class ItemType> inline ItemList<ItemType> checkedItems(int checkId = Item::PrimaryCheck) const {
        return RootItem::instance()->checkedItems<ItemType>();
    }
    bool isItemChecked(const Item* item, int checkId = Item::PrimaryCheck) const {
        return item->isChecked(checkId);
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
