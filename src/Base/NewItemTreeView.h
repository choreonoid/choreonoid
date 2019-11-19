#ifndef CNOID_BASE_NEW_ITEM_TREE_VIEW_H
#define CNOID_BASE_NEW_ITEM_TREE_VIEW_H

#include "View.h"
#include "RootItem.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT NewItemTreeView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
    static NewItemTreeView* instance();
    static NewItemTreeView* mainInstance(); // deprecated

    NewItemTreeView();
    ~NewItemTreeView();

    Item* rootItem();
    void expandItem(Item* item, bool expanded = true);
    void cutSelectedItems();

    /*
       The following functions and signals on the selection state and check state are deprecated.
       The management of those statements are moved to the Item and RootItem classes.
    */
    template <class ItemType> ItemList<ItemType> selectedItems() {
        return RootItem::selectedItems();
    }
    const ItemList<>& selectedItems() {
        return RootItem::selectedItems();
    }
    template <class ItemType> ItemType* selectedItem(bool fromMultiItems = false) {
        return selectedItems<ItemType>().toSingle(fromMultiItems);
    }
    template <class ItemType> ItemList<ItemType> selectedSubItems(Item* subTreeRoot) {
        return RootItem::selectedSubTreeItems<ItemType>(subTreeRoot);
    }
    template <class ItemType> ItemType* selectedSubItem(Item* subTreeRoot, bool fromMultiItems = false) {
        return selectedSubItems<ItemType>(subTreeRoot).toSingle(fromMultiItems);
    }
    bool isItemSelected(Item* item){
        return item->isSelected();
    }
    bool selectItem(Item* item, bool on = true){
        item->setSelected(on);
        return true;
    }
    void unselectItem(Item* item){
        item->setSelected(false);
    }
    void selectAllItems(){
        RootItem::selectAllItemSelected(true);
    }
    void clearSelection(){
        RootItem::setAllItemSelected(false);
    }
    int addCheckColumn(){
        return RootItem::addCheckState();
    }
    void setCheckColumnToolTip(int id, const QString& whatsThis){
        RootItem::setCheckStateDescription(id, whatsThis.toStdString());
    }
    void storeCheckColumnState(int id, Archive& archive){
        RootItem::storeCheckStates(id, archive);
    }
    bool restoreCheckColumnState(int id, const Archive& archive){
        return RootItem::restoreCheckStates(id, archive);
    }
    void releaseCheckColumn(int id){
        return RootItem::releaseCheckColumn(id);
    }
    enum { ID_ANY = -1 };
    template <class ItemType> inline ItemList<ItemType> checkedItems(int id = 0) {
        return RootItem::checkedItems<ItemType>(id);
    }
    bool isItemChecked(Item* item, int id = 0){
        return item->isChecked(id);
    }
    bool checkItem(Item* item, bool on = true, int id = 0){
        item->setChecked(id, on);
        return true;
    }
    SignalProxy<void(const ItemList<>&)> sigSelectionChanged(){
        return RootItem::sigSelectionChanged();
    }
    SignalProxy<void(Item* item, bool on)> sigCheckToggled(int id = 0){
        return RootItem::sigCheckToggled(id);
    }
    SignalProxy<void(bool on)> sigCheckToggled(Item* item, int id = 0){
        return item->sigCheckToggled(id);
    }

    //! \todo remove the followings or make their replacements
    void updateCheckColumnToolTip(int id);
    void showCheckColumn(int id, bool on = true);
    SignalProxy<void(const ItemList<>&)> sigSelectionOrTreeChanged();

protected:
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;

private:
    class Impl;
    Impl* impl;
};

}

#endif
