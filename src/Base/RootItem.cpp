/**
   @author Shin'ichiro Nakaoka
*/

#include "RootItem.h"
#include "ExtensionManager.h"
#include "ItemManager.h"
#include "LazyCaller.h"
#include "LazySignal.h"
#include "Archive.h"
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

const bool TRACE_FUNCTIONS = false;

class CheckEntry
{
public:
    Signal<void(Item* item, bool on)> sigCheckToggled;
    ItemList<> checkedItems;
    bool needToUpdateCheckedItemList;
    string description;

    CheckEntry(const string& description);
    CheckEntry(const CheckEntry& org);
    void emitSigCheckToggled();
};

}

namespace cnoid {

class RootItem::Impl
{
public:
    RootItem* self;
	
    ItemList<> selectedItems;
    ItemPtr focusedItem;
    bool needToUpdateSelectedItems;
    LazyCaller emitSigSelectedItemsChangedLater;
    Signal<void(Item* item, bool on)> sigSelectionChanged;
    Signal<void(const ItemList<>& selectedItems)> sigSelectedItemsChanged;

    vector<shared_ptr<CheckEntry>> checkEntries;
    Signal<void(Item* item, bool on)> sigLogicalSumOfAllChecksToggled;
    Signal<void(Item* item, bool on)> sigCheckToggledDummy;
    Signal<void(int checkId)> sigCheckEntryAdded;
    Signal<void(int checkId)> sigCheckEntryReleased;

    string emptyString;
    ItemList<> dummyItemList;

    Signal<void(RootItem* rootItem)> sigDestroyed;
    Signal<void(Item* item)> sigSubTreeAdded;
    Signal<void(Item* item)> sigItemAdded;
    Signal<void(Item* item)> sigSubTreeMoved;
    Signal<void(Item* item)> sigItemMoved;
    Signal<void(Item* item, bool isMoving)> sigSubTreeRemoving;
    Signal<void(Item* item, bool isMoving)> sigSubTreeRemoved;
    LazySignal<Signal<void()>> sigTreeChanged;
    Signal<void(Item* assigned, Item* srcItem)> sigItemAssigned;

    Impl(RootItem* self);
    Impl(RootItem* self, const Impl& org);
    void doCommonInitialization();
    void emitSigItemAddedForItemTree(Item* item);
    void emitSigItemMovedForItemTree(Item* item);
    bool updateSelectedItemsIter(Item* item);
    void updateCheckedItemsIter(Item* item, int checkId, ItemList<>& checkedItems);
};

}


CheckEntry::CheckEntry(const string& description)
    : needToUpdateCheckedItemList(false),
      description(description)
{

}


CheckEntry::CheckEntry(const CheckEntry& org)
    : needToUpdateCheckedItemList(false),
      description(org.description)
{

}

#include "MenuManager.h"
#include "ItemClassRegistry.h"
#include <fmt/format.h>
static void putItemTreeWithPolymorphicIds()
{
    auto& registry = ItemClassRegistry::instance();
    cout << "Number of item classes: " << registry.numRegisteredClasses() << endl;
    for(auto& item : RootItem::instance()->descendantItems()){
        int id = item->classId();
        cout << fmt::format("{}: id {} : {}",
                            item->name(), id, registry.superClassId(id)) << endl;
    }
}
    

void RootItem::initializeClass(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        ext->itemManager().registerClass<RootItem>(N_("RootItem"));
        ext->manage(RootItemPtr(mainInstance()));
        initialized = true;

        // debug
        ext->menuManager().setPath("/Options").addItem("Polymorphic id test")->sigTriggered().connect(
            [](){ putItemTreeWithPolymorphicIds(); });
    }
}


RootItem* RootItem::instance()
{
    static RootItem* rootItem = new RootItem();
    rootItem->setName("Root");
    return rootItem;
}


RootItem::RootItem()
{
    impl = new Impl(this);
    addCheckEntry(_("Primary check"));
}


RootItem::Impl::Impl(RootItem* self)
    : self(self)
{
    doCommonInitialization();
}


RootItem::RootItem(const RootItem& org)
    : Item(org)
{
    impl = new Impl(this, *org.impl);
}


RootItem::Impl::Impl(RootItem* self, const Impl& org)
{
    checkEntries.reserve(org.checkEntries.size());
    for(auto& entry : org.checkEntries){
        checkEntries.push_back(make_shared<CheckEntry>(*entry));
    }

    doCommonInitialization();
}


void RootItem::Impl::doCommonInitialization()
{
    needToUpdateSelectedItems = false;
    
    emitSigSelectedItemsChangedLater.setFunction(
        [&](){ sigSelectedItemsChanged(self->getSelectedItems()); });
}


RootItem::~RootItem()
{
    // This is also written in Item::~Item(), but this should be also written here
    // to notify detaching from the root even when the root is destroyed
    Item* child = childItem();
    while(child){
        Item* next = child->nextItem();
        child->detachFromParentItem();
        child = next;
    }

    impl->sigDestroyed(this);

    delete impl;
}


Item* RootItem::doDuplicate() const
{
    return new RootItem(*this);
}


SignalProxy<void(RootItem* rootItem)> RootItem::sigDestroyed()
{
    return impl->sigDestroyed;
}


SignalProxy<void(Item* item)> RootItem::sigSubTreeAdded()
{
    return impl->sigSubTreeAdded;
}


SignalProxy<void(Item* item)> RootItem::sigItemAdded()
{
    return impl->sigItemAdded;
}


SignalProxy<void(Item* item)> RootItem::sigSubTreeMoved()
{
    return impl->sigSubTreeMoved;
}


SignalProxy<void(Item* item)> RootItem::sigItemMoved()
{
    return impl->sigItemMoved;
}


/**
   @if jp
   The signal that is emitted just before an item belonging to the path from
   the root item is being removed.
   
   @param isMoving It is true if the item is moving and again belongs to the
   path from the root item.
   
   @todo This signal should be replaced with the itemRemoved signal and deprecated.
*/
SignalProxy<void(Item* item, bool isMoving)> RootItem::sigSubTreeRemoving()
{
    return impl->sigSubTreeRemoving;
}


/**
   @if jp
   The signal that is emitted when an item belonging to the item tree from the
   root item is removed.
   
   @param isMoving It is true if the item is moving and again belongs to the
   path from the root item.
*/
SignalProxy<void(Item* item, bool isMoving)> RootItem::sigSubTreeRemoved()
{
    return impl->sigSubTreeRemoved;
}


/**
   @if jp
   The signal that is emitted when the structure of the item tree changes,
   such as adding and deleting items.

   Unlike sigItemAdded or sigItemRemoving, it is emitted only once for a series of
   operations performed at once. To be precise, it is emitted after the events in
   the queue are processed in the event loop of the framework.   
   
   @todo "Once all at once" is probably not being protected at the time of project
   loading etc, so improve this point.
*/
SignalProxy<void()> RootItem::sigTreeChanged()
{
    return impl->sigTreeChanged.signal();
}


/**
   This signal is emitted when Item::asign() is called.
*/
SignalProxy<void(Item* assigned, Item* srcItem)> RootItem::sigItemAssigned()
{
    return impl->sigItemAssigned;
}


void RootItem::notifyEventOnSubTreeAdded(Item* item)
{
    if(TRACE_FUNCTIONS){
        cout << "RootItem::notifyEventOnItemAdded()" << endl;
    }

    impl->sigSubTreeAdded(item);
    impl->emitSigItemAddedForItemTree(item);
    impl->sigTreeChanged.request();
}


void RootItem::Impl::emitSigItemAddedForItemTree(Item* item)
{
    sigItemAdded(item);
    for(Item* child = item->childItem(); child; child = child->nextItem()){
        emitSigItemAddedForItemTree(child);
    }
}


void RootItem::notifyEventOnSubTreeMoved(Item* item)
{
    if(TRACE_FUNCTIONS){
        cout << "RootItem::notifyEventOnItemMoved()" << endl;
    }

    impl->sigSubTreeMoved(item);
    impl->emitSigItemMovedForItemTree(item);
    impl->sigTreeChanged.request();
}


void RootItem::Impl::emitSigItemMovedForItemTree(Item* item)
{
    sigItemMoved(item);
    for(Item* child = item->childItem(); child; child = child->nextItem()){
        emitSigItemMovedForItemTree(child);
    }
}


void RootItem::notifyEventOnSubTreeRemoving(Item* item, bool isMoving)
{
    impl->sigSubTreeRemoving(item, isMoving);
    impl->sigTreeChanged.request();
}


void RootItem::notifyEventOnSubTreeRemoved(Item* item, bool isMoving)
{
    impl->sigSubTreeRemoved(item, isMoving);
    impl->sigTreeChanged.request();
}


void RootItem::emitSigItemAssinged(Item* assigned, Item* srcItem)
{
    impl->sigItemAssigned(assigned, srcItem);
}


Item* RootItem::focusedItem()
{
    auto& selected = getSelectedItems();
    if(impl->focusedItem){
        return impl->focusedItem;
    }
    return selected.toSingle(true);
}


const ItemList<>& RootItem::getSelectedItems()
{
    if(impl->needToUpdateSelectedItems){
        impl->selectedItems.clear();
        bool isFocusedItemIncluded = impl->updateSelectedItemsIter(this);
        if(!isFocusedItemIncluded){
            impl->focusedItem = nullptr;
        }
        impl->needToUpdateSelectedItems = false;
    }
    return impl->selectedItems;
}


//! \return true if the selected items include the focused item
bool RootItem::Impl::updateSelectedItemsIter(Item* item)
{
    bool isFocusedItemIncluded = false;
    
    if(item->isSelected()){
        selectedItems.push_back(item);
        if(item == focusedItem){
            isFocusedItemIncluded = true;
        }
    }
    for(Item* child = item->childItem(); child; child = child->nextItem()){
        if(updateSelectedItemsIter(child)){
            isFocusedItemIncluded = true;
        }
    }

    return isFocusedItemIncluded;
}
    

void RootItem::emitSigSelectionChanged(Item* item, bool on, bool isFocused)
{
    if(isFocused){
        if(on){
            impl->focusedItem = item;
        } else {
            impl->focusedItem = nullptr;
        }
    }
    impl->needToUpdateSelectedItems = true;
    impl->sigSelectionChanged(item, on);
}


void RootItem::emitSigSelectedItemsChangedLater()
{
    impl->needToUpdateSelectedItems = true;
    impl->emitSigSelectedItemsChangedLater();
}


void RootItem::flushSigSelectedItemsChanged()
{
    if(impl->emitSigSelectedItemsChangedLater.isPending()){
        impl->emitSigSelectedItemsChangedLater.flush();
    }
}


SignalProxy<void(Item* item, bool on)> RootItem::sigSelectionChanged()
{
    return impl->sigSelectionChanged;
}


SignalProxy<void(const ItemList<>& selectedItems)> RootItem::sigSelectedItemsChanged()
{
    return impl->sigSelectedItemsChanged;
}


int RootItem::addCheckEntry(const std::string& description)
{
    const int n = impl->checkEntries.size();
    int checkId;
    for(checkId = 0; checkId < n; ++checkId){
        if(!impl->checkEntries[checkId]){
            break;
        }
    }
    if(checkId >= n){
        impl->checkEntries.resize(checkId + 1);
    }
    impl->checkEntries[checkId] = make_shared<CheckEntry>(description);

    impl->sigCheckEntryAdded(checkId);
    
    return checkId;
}


int RootItem::numCheckEntries() const
{
    return impl->checkEntries.size();
}


const std::string& RootItem::checkEntryDescription(int checkId) const
{
    if(checkId < impl->checkEntries.size()){
        if(auto checkEntry = impl->checkEntries[checkId]){
            return checkEntry->description;
        }
    }
    return impl->emptyString;
}


void RootItem::releaseCheckEntry(int checkId)
{
    int n = impl->checkEntries.size();
    if(checkId < n){
        if(checkId == n - 1){
            impl->checkEntries.resize(n - 1);
        } else {
            impl->checkEntries[checkId] = nullptr;
        }
        impl->sigCheckEntryReleased(checkId);
    }
}


SignalProxy<void(int checkId)> RootItem::sigCheckEntryAdded()
{
    return impl->sigCheckEntryAdded;
}


SignalProxy<void(int checkId)> RootItem::sigCheckEntryReleased()
{
    return impl->sigCheckEntryReleased;
}


bool RootItem::storeCheckStates(int checkId, Archive& archive, const std::string& key)
{
    auto& items = getCheckedItems(checkId);
    Listing& idseq = *archive.createFlowStyleListing(key);
    for(auto& item : items){
        if(auto itemId = archive.getItemId(item)){
            idseq.append(itemId);
        }
    }
    return true;
}


bool RootItem::restoreCheckStates(int checkId, const Archive& archive, const std::string& key)
{
    bool completed = false;
    const Listing& idseq = *archive.findListing(key);
    if(idseq.isValid()){
        completed = true;
        for(int i=0; i < idseq.size(); ++i){
            auto itemId = idseq.at(i);
            if(!itemId){
                completed = false;
            } else {
                if(auto item = archive.findItem(itemId)){
                    item->setChecked(checkId, true);
                } else {
                    completed = false;
                }
            }
        }
    }
    return completed;
}


const ItemList<>& RootItem::getCheckedItems(int checkId)
{
    if(checkId < impl->checkEntries.size()){
        if(auto checkEntry = impl->checkEntries[checkId]){
            auto& checkedItems = checkEntry->checkedItems;
            if(checkEntry->needToUpdateCheckedItemList){
                checkedItems.clear();
                impl->updateCheckedItemsIter(this, checkId, checkedItems);
                checkEntry->needToUpdateCheckedItemList = false;
            }
            return checkedItems;
        }
    }
    return impl->dummyItemList;
}


void RootItem::Impl::updateCheckedItemsIter(Item* item, int checkId, ItemList<>& checkedItems)
{
    if(item->isChecked(checkId)){
        checkedItems.push_back(item);
    }
    for(Item* child = item->childItem(); child; child = child->nextItem()){
        updateCheckedItemsIter(child, checkId, checkedItems);
    }
}


void RootItem::emitSigCheckToggled(Item* item, int checkId, bool on)
{
    if(checkId == Item::LogicalSumOfAllChecks){
        impl->sigLogicalSumOfAllChecksToggled(item, on);

    } else if(checkId >= 0 && checkId < impl->checkEntries.size()){
        if(auto checkEntry = impl->checkEntries[checkId]){
            checkEntry->needToUpdateCheckedItemList = true;
            checkEntry->sigCheckToggled(item, on);
        }
    }
}


SignalProxy<void(Item* item, bool on)> RootItem::sigCheckToggled(int checkId)
{
    if(checkId == Item::LogicalSumOfAllChecks){
        return impl->sigLogicalSumOfAllChecksToggled;

    } else if(checkId >= 0 && checkId < impl->checkEntries.size()){
        if(auto checkEntry = impl->checkEntries[checkId]){
            return checkEntry->sigCheckToggled;
        }
    }
    return impl->sigCheckToggledDummy;
}


bool RootItem::store(Archive& archive)
{
    return true;
}


bool RootItem::restore(const Archive& archive)
{
    return true;
}
