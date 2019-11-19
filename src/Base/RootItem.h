/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_ROOT_ITEM_H
#define CNOID_BASE_ROOT_ITEM_H

#include "Item.h"
#include "exportdecl.h"

namespace cnoid {

class Project;
class Item;
class RootItemImpl;

/**
   The class of the item that is the root of the item tree structure
*/
class CNOID_EXPORT RootItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);
    static RootItem* instance();
    static RootItem* mainInstance(); // deprecated

    RootItem();
    RootItem(const RootItem& org);
    virtual ~RootItem();

    SignalProxy<void(RootItem* rootItem)> sigDestroyed();
    SignalProxy<void(Item* item)> sigSubTreeAdded();
    SignalProxy<void(Item* item)> sigItemAdded();
    SignalProxy<void(Item* item)> sigSubTreeMoved();
    SignalProxy<void(Item* item)> sigItemMoved();
    SignalProxy<void(Item* item, bool isMoving)> sigSubTreeRemoving();
    SignalProxy<void(Item* item, bool isMoving)> sigSubTreeRemoved();
    SignalProxy<void()> sigTreeChanged();
    SignalProxy<void(Item* assigned, Item* srcItem)> sigItemAssigned();

    void setAllItemSelected(bool on);

    template <class ItemType> ItemList<ItemType> selectedItems() {
        return getSelectedItems();
    }

    const ItemList<>& selectedItems() {
        return getSelectedItems();
    }
    
    template <class ItemType> ItemList<ItemType> selectedSubTreeItems(Item* subTreeRoot) {
        return getSelectedSubTreeItems(subTreeRoot);
    }

    SignalProxy<void(const ItemList<>& selectedItems)> sigSelectionChanged();

    //! \return The state id of the new check state.
    int addCheckState();
    void setCheckStateDescription(int stateId, const std::string& description);
    void releaseCheckState(int stateId);
    void storeCheckStates(int stateId, Archive& archive){
    bool restoreCheckStates(int stateId, const Archive& archive);

    template <class ItemType> ItemList<ItemType> checkedItems(int stateId = EnabledState) {
        return getCheckedItems(stateId);
    }

    SignalProxy<void(Item* item, bool on)> sigCheckToggled(int stateId = EnabledState);
    
protected:

    virtual Item* doDuplicate() const;
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

private:

    void initializeInstance();

    friend class Item;

    void notifyEventOnSubTreeAdded(Item* item);
    void notifyEventOnSubTreeMoved(Item* item);
    void notifyEventOnSubTreeRemoving(Item* item, bool isMoving);
    void notifyEventOnSubTreeRemoved(Item* item, bool isMoving);

    void emitSigItemAssinged(Item* assigned, Item* srcItem);

    friend class RootItemImpl;
    RootItemImpl* impl;
};

typedef ref_ptr<RootItem> RootItemPtr;

}

#endif
