/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_ROOT_ITEM_H
#define CNOID_BASE_ROOT_ITEM_H

#include "Item.h"
#include "exportdecl.h"

namespace cnoid {

/**
   The class of the item that is the root of the item tree structure
*/
class CNOID_EXPORT RootItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);
    static RootItem* instance();

    [[deprecated("Use RootItem::instance()")]]
    static RootItem* mainInstance() { return instance(); }

    RootItem();
    virtual ~RootItem();

    /**
       The copy constructor is disabled and the duplication function
       is not implemented becasue RootItem is a singleton item class.
    */
    RootItem(const RootItem& org) = delete;
    // virtual Item* doDuplicate() const override;
    
    SignalProxy<void(RootItem* rootItem)> sigDestroyed();
    SignalProxy<void(Item* item)> sigSubTreeAdded();
    SignalProxy<void(Item* item)> sigItemAdded();
    SignalProxy<void(Item* item)> sigSubTreeMoved();
    SignalProxy<void(Item* item)> sigItemMoved();
    SignalProxy<void(Item* item, bool isMoving)> sigSubTreeRemoving();
    SignalProxy<void(Item* item, bool isMoving)> sigSubTreeRemoved();
    SignalProxy<void(Item* assigned, const Item* srcItem)> sigItemAssigned();
    SignalProxy<void(Item* item, const std::string& oldName)> sigItemNameChanged();

    [[deprecated("Use Item::sigSubTreeChanged()")]]
    SignalProxy<void()> sigTreeChanged();

    Item* currentItem();
    
    template <class ItemType> ItemList<ItemType> selectedItems() {
        return getSelectedItems();
    }

    const ItemList<>& selectedItems() {
        return getSelectedItems();
    }

    void selectItem(Item* item);

    SignalProxy<void(Item* item, bool on)> sigSelectionChanged();

    SignalProxy<void(const ItemList<>& selectedItems)> sigSelectedItemsChanged();
    void beginItemSelectionChanges();
    void endItemSelectionChanges();

    //! \return The state id of the new check state.
    int addCheckEntry(const std::string& description);
    int numCheckEntries() const;
    const std::string& checkEntryDescription(int checkId) const;
    void releaseCheckEntry(int checkId);

    SignalProxy<void(int checkId)> sigCheckEntryAdded();
    SignalProxy<void(int checkId)> sigCheckEntryReleased();
    
    bool storeCheckStates(int checkId, Archive& archive, const std::string& key);
    bool restoreCheckStates(int checkId, const Archive& archive, const std::string& key);

    //! \note Item::LogicalSumOfAllChecks is not supported
    template <class ItemType> ItemList<ItemType> checkedItems(int checkId = PrimaryCheck) {
        return getCheckedItems(checkId);
    }

    const ItemList<>& checkedItems() {
        return getSelectedItems();
    }

    SignalProxy<void(Item* item, bool on)> sigCheckToggled(int checkId = PrimaryCheck);

    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

    /**
       The root item is initialized to be consistent when the entire project tree is loaded.
    */
    virtual void setConsistentWithArchive(bool isConsistent) override;

    /**
       For the root item, the consistency with the archive means that the structure of
       the entire project item tree is consistent with that archived in the project file.
    */
    virtual bool checkConsistencyWithArchive() override;

private:
    friend class Item;
    class Impl;
    Impl* impl;

    // The following functions are called from the implementation of the Item class
    void notifyEventOnSubTreeAdded(Item* item, std::vector<Item*>& orgSubTreeItems);
    void notifyEventOnSubTreeMoved(Item* item, std::vector<Item*>& orgSubTreeItems);
    void notifyEventOnSubTreeRemoving(Item* item, bool isMoving);
    void notifyEventOnSubTreeRemoved(Item* item, bool isMoving);
    void emitSigItemAssinged(Item* assigned, const Item* srcItem);
    void emitSigItemNameChanged(Item* item, const std::string& oldName);
    void emitSigSelectionChanged(Item* item, bool on, bool isCurrent);
    void requestToEmitSigSelectedItemsChanged();
    void emitSigCheckToggled(Item* item, int checkId, bool on);

    const ItemList<>& getSelectedItems();
    const ItemList<>& getCheckedItems(int checkId);
};

typedef ref_ptr<RootItem> RootItemPtr;

}

#endif
