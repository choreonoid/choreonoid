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

    /**
       This function may return nullptr if ItemTreeView is disabled by not including it in the whitelist
       and the instance of it is not being used from anywhere.
    */
    static ItemTreeView* findInstance();

     // deprecated
    static ItemTreeView* mainInstance() {
        return instance();
    }

    ItemTreeView();
    ~ItemTreeView();

    ItemTreeWidget* itemTreeWidget();

    template<class ItemType>
    static void customizeContextMenu(
        std::function<void(ItemType* item, MenuManager& menuManager, ItemFunctionDispatcher menuFunction)> func){
        if(auto view = findInstance()){
            view->itemTreeWidget()->customizeContextMenu(func);
        }
    }

    void setExpanded(Item* item, bool on = true);

    template <class ItemType>
    [[deprecated("Use RootItem::selectedItems")]]
    ItemList<ItemType> selectedItems() const {
        return RootItem::instance()->selectedItems<ItemType>();
    }
    [[deprecated("Use RootItem::selectedItems")]]
    const ItemList<>& selectedItems() const {
        return RootItem::instance()->selectedItems();
    }
    template <class ItemType>
    [[deprecated]]
    ItemType* selectedItem(bool fromMultiItems = false) const {
        return selectedItems<ItemType>().toSingle(fromMultiItems);
    }
    template <class ItemType>
    [[deprecated("Use Item::selectedDescendantItems")]]
    ItemList<ItemType> selectedSubItems(const Item* topItem) const {
        return topItem->selectedDescendantItems<ItemType>();
    }
    template <class ItemType>
    [[deprecated]]
    ItemType* selectedSubItem(Item* topItem, bool fromMultiItems = false) const {
        return selectedSubItems<ItemType>(topItem).toSingle(fromMultiItems);
    }
    [[deprecated("Use Item::isSelected")]]
    bool isItemSelected(const Item* item) const {
        return item->isSelected();
    }
    [[deprecated("Use Item::setSelected")]]
    bool selectItem(Item* item, bool on = true) {
        item->setSelected(on);
        return true;
    }
    [[deprecated("Use Item::setSubTreeItemsSelected for the root item")]]
    void selectAllItems() {
        RootItem::instance()->setSubTreeItemsSelected(true);
    }
    [[deprecated("Use Item::setSubTreeItemsSelected for the root item")]]
    void clearSelection() {
        RootItem::instance()->setSubTreeItemsSelected(false);
    }

    enum CheckId { ID_ANY = Item::LogicalSumOfAllChecks };
        
    template <class ItemType>
    [[deprecated("Use RootItem::checkedItems")]]
    ItemList<ItemType> checkedItems(int checkId = Item::PrimaryCheck) const {
        return RootItem::instance()->checkedItems<ItemType>(checkId);
    }
    [[deprecated("Use Item::isChecked")]]
    bool isItemChecked(const Item* item, int checkId = Item::PrimaryCheck) const {
        return item->isChecked(checkId);
    }
    [[deprecated("Use Item::setChecked")]]
    bool checkItem(Item* item, bool on = true, int checkId = Item::PrimaryCheck) {
        item->setChecked(checkId, on);
        return true;
    }
    [[deprecated("Use RootItem::sigSelectedItemsChanged")]]
    SignalProxy<void(const ItemList<>&)> sigSelectionChanged() const {
        return RootItem::instance()->sigSelectedItemsChanged();
    }
    [[deprecated("Use RootItem::sigCheckToggled")]]
    SignalProxy<void(Item* item, bool isChecked)> sigCheckToggled(int checkId = Item::PrimaryCheck) const {
        return RootItem::instance()->sigCheckToggled(checkId);
    }
    [[deprecated("Use Item::sigCheckToggled")]]
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
