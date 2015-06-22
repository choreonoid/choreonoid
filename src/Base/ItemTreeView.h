/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_ITEM_TREE_VIEW_H
#define CNOID_BASE_ITEM_TREE_VIEW_H

#include "ItemList.h"
#include <cnoid/View>
#include <QAbstractItemModel>
#include "exportdecl.h"

namespace cnoid {

class RootItem;
class ItemTreeViewImpl;

/**
   @if jp
   アイテムツリーを表示するウィンドウ
   @endif
*/
class CNOID_EXPORT ItemTreeView : public View
{
    Q_OBJECT

public:
    static void initializeClass(ExtensionManager* ext);
    static ItemTreeView* instance();
    static ItemTreeView* mainInstance(); // obsolete

    ItemTreeView();
    ItemTreeView(RootItem* rootItem, bool showRoot = false);
    ~ItemTreeView();

    RootItem* rootItem();

    void showRoot(bool show);

    /**
       @if jp
       選択状態になっているアイテムのうち、指定した型に適合するものを取得する。
       @endif
    */
    template <class ItemType> inline ItemList<ItemType> selectedItems() {
        return allSelectedItems();
    }

    const ItemList<>& selectedItems() {
        return allSelectedItems();
    }

        
    template <class ItemType> inline ItemType* selectedItem(bool fromMultiItems = false) {
        return selectedItems<ItemType>().toSingle(fromMultiItems);
    }

    /**
       @if jp
       topItem 以下のサブツリーにおける選択状態アイテムのリストを得る。
       topItem は選択されていてもリストには含まれない。
       @endif
    */
    template <class ItemType> inline ItemList<ItemType> selectedSubItems(ItemPtr topItem) {
        ItemList<> items;
        extractSelectedItemsOfSubTree(topItem, items);
        return items;
    }

    template <class ItemType> inline ItemType* selectedSubItem(ItemPtr topItem, bool fromMultiItems = false) {
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

    /**
       @if jp
       チェック状態になっているアイテムのうち、指定した型に適合するものを取得する。
       @endif
    */
    template <class ItemType> inline ItemList<ItemType> checkedItems(int id = 0) {
        return allCheckedItems(id);
    }

    bool isItemChecked(ItemPtr item, int id = 0);
    bool checkItem(ItemPtr item, bool check = true, int id = 0);

    /**
       @if jp
       アイテムの選択状態が変化したときに発行されるシグナル。
       @endif
    */
    SignalProxy<void(const ItemList<>&)> sigSelectionChanged();

    /**
       @if jp
       アイテムの選択状態が変化したか、ツリーの構造が変化したときに発行されるシグナル。
       アイテム間の親子関係もみるようなハンドラはこのシグナルと接続するとよい。
       @endif
    */
    SignalProxy<void(const ItemList<>&)> sigSelectionOrTreeChanged();

    SignalProxy<void(Item* item, bool isChecked)> sigCheckToggled(int id = 0);

    SignalProxy<void(bool isChecked)> sigCheckToggled(Item* targetItem, int id = 0);

    void cutSelectedItems();

protected:

    virtual bool storeState(Archive& archive);
    virtual bool restoreState(const Archive& archive);

private:

    ItemTreeViewImpl* impl;

    void construct(RootItem* rootItem, bool showRoot);
    ItemList<>& allSelectedItems();
    ItemList<>& allCheckedItems(int id);
    void extractSelectedItemsOfSubTree(ItemPtr topItem, ItemList<>& items);

private Q_SLOTS:
    void onRowsAboutToBeRemoved(const QModelIndex& parent, int start, int end);
    void onRowsInserted(const QModelIndex& parent, int start, int end);
};

}

#endif
