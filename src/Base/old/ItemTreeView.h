/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_ITEM_TREE_VIEW_H
#define CNOID_BASE_ITEM_TREE_VIEW_H

#include "View.h"
#include "ItemList.h"
#include "exportdecl.h"

namespace cnoid {

class ItemTreeViewImpl;

class CNOID_EXPORT ItemTreeView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
    static ItemTreeView* instance();
    static ItemTreeView* mainInstance(); // deprecated

    ItemTreeView();
    ~ItemTreeView();

    Item* rootItem();
    
    /**
       This function returns the specific type items that are selected in the ItemTreeView
    */
    template <class ItemType> ItemList<ItemType> selectedItems() {
        return allSelectedItems();
    }

    const ItemList<>& selectedItems() {
        return allSelectedItems();
    }

        
    template <class ItemType> ItemType* selectedItem(bool fromMultiItems = false) {
        return selectedItems<ItemType>().toSingle(fromMultiItems);
    }

    /**
       This functions returns the specific type items that are selected in the sub tree of the topItem.
       The topItem itself is not included in the return value list.
    */
    template <class ItemType> ItemList<ItemType> selectedSubItems(Item* topItem) {
        ItemList<> items;
        extractSelectedItemsOfSubTree(topItem, items);
        return items;
    }

    template <class ItemType> ItemType* selectedSubItem(Item* topItem, bool fromMultiItems = false) {
        return selectedSubItems<ItemType>(topItem).toSingle(fromMultiItems);
    }
        
    bool isItemSelected(Item* item);
    bool selectItem(Item* item, bool select = true);
    void unselectItem(Item* item);
    
    void selectAllItems();
    void clearSelection();

    /**
       @return The ID of the check column.
    */
    int addCheckColumn();
    void setCheckColumnToolTip(int id, const QString& whatsThis);
    void updateCheckColumnToolTip(int id);
    void showCheckColumn(int id, bool on = true);
    void storeCheckColumnState(int id, Archive& archive);
    bool restoreCheckColumnState(int id, const Archive& archive);
    void releaseCheckColumn(int id);

    enum { ID_ANY = -1 };

    /**
       This functions returns the specific type items that are checked in the ItemTreeView
    */
    template <class ItemType> inline ItemList<ItemType> checkedItems(int id = 0) {
        return allCheckedItems(id);
    }

    bool isItemChecked(Item* item, int id = 0);
    bool checkItem(Item* item, bool check = true, int id = 0);

    void expandItem(Item* item, bool expanded = true);

    /**
       The signal that is emitted when the item selection state is changed.
    */
    SignalProxy<void(const ItemList<>&)> sigSelectionChanged();

    /**
       The signal that is emitted when the item selection state or the tree structure is changed.
    */
    SignalProxy<void(const ItemList<>&)> sigSelectionOrTreeChanged();
    
    SignalProxy<void(Item* item, bool isChecked)> sigCheckToggled(int id = 0);

    SignalProxy<void(bool isChecked)> sigCheckToggled(Item* item, int id = 0);

    void cutSelectedItems();

protected:
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;

private:
    ItemTreeViewImpl* impl;

    ItemList<>& allSelectedItems();
    ItemList<>& allCheckedItems(int id);
    void extractSelectedItemsOfSubTree(Item* topItem, ItemList<>& items);
};

}

#endif
