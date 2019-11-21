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

class CheckState
{
public:
    ItemList<> checkedItems;
    bool needToUpdateCheckedItemList;
    Signal<void(Item* item, bool on)> sigCheckToggled;
    string description;

    CheckState(const string& description);
    CheckState(const CheckState& org);
    void emitSigCheckToggled();
};

}

namespace cnoid {

class RootItem::Impl
{
public:
    RootItem* self;
	
    ItemList<> selectedItems;
    bool needToUpdateSelectedItems;
    LazyCaller emitSigSelectionChangedLater;
    Signal<void(const ItemList<>& selectedItems)> sigSelectionChanged;

    vector<shared_ptr<CheckState>> checkStates;
    Signal<void(int checkId)> sigCheckStateAdded;
    Signal<void(int checkId)> sigCheckStateReleased;
    Signal<void(Item* item, bool on)> sigCheckToggledDummy;

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
    void updateSelectedItemsIter(Item* item);
    void updateCheckedItemsIter(Item* item, int checkId, ItemList<>& checkedItems);
};

}


CheckState::CheckState(const string& description)
    : needToUpdateCheckedItemList(false),
      description(description)
{

}


CheckState::CheckState(const CheckState& org)
    : needToUpdateCheckedItemList(false),
      description(org.description)
{

}
    

void RootItem::initializeClass(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        ext->itemManager().registerClass<RootItem>(N_("RootItem"));
        ext->manage(RootItemPtr(mainInstance()));
        initialized = true;
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
    addCheckState(_("Primary check state"));
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
    checkStates.reserve(org.checkStates.size());
    for(auto& state : org.checkStates){
        checkStates.push_back(make_shared<CheckState>(*state));
    }

    doCommonInitialization();
}


void RootItem::Impl::doCommonInitialization()
{
    needToUpdateSelectedItems = false;
    
    emitSigSelectionChangedLater.setFunction(
        [&](){ sigSelectionChanged(self->getSelectedItems()); });
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


const ItemList<>& RootItem::getSelectedItems()
{
    if(impl->needToUpdateSelectedItems){
        impl->selectedItems.clear();
        impl->updateSelectedItemsIter(this);
        impl->needToUpdateSelectedItems = false;
    }
    return impl->selectedItems;
}


void RootItem::Impl::updateSelectedItemsIter(Item* item)
{
    if(item->isSelected()){
        selectedItems.push_back(item);
    }
    for(Item* child = item->childItem(); child; child = child->nextItem()){
        updateSelectedItemsIter(child);
    }
}
    

void RootItem::notifyEventOnItemSelectionChanged(Item* item, bool on)
{
    impl->needToUpdateSelectedItems = true;
    impl->emitSigSelectionChangedLater();
}


SignalProxy<void(const ItemList<>& selectedItems)> RootItem::sigSelectionChanged()
{
    return impl->sigSelectionChanged;
}


int RootItem::addCheckState(const std::string& description)
{
    const int n = impl->checkStates.size();
    int checkId;
    for(checkId = 0; checkId < n; ++checkId){
        if(!impl->checkStates[checkId]){
            break;
        }
    }
    if(checkId >= n){
        impl->checkStates.resize(checkId + 1);
    }
    impl->checkStates[checkId] = make_shared<CheckState>(description);

    impl->sigCheckStateAdded(checkId);
    
    return checkId;
}


int RootItem::numCheckStates() const
{
    return impl->checkStates.size();
}


std::string RootItem::checkStateDescription(int checkId) const
{
    if(checkId < impl->checkStates.size()){
        if(auto checkState = impl->checkStates[checkId]){
            return checkState->description;
        }
    }
    return string();
}


void RootItem::releaseCheckState(int checkId)
{
    int n = impl->checkStates.size();
    if(checkId < n){
        if(checkId == n - 1){
            impl->checkStates.resize(n - 1);
        } else {
            impl->checkStates[checkId] = nullptr;
        }
        impl->sigCheckStateReleased(checkId);
    }
}


bool RootItem::storeCheckStates(int checkId, Archive& archive, const std::string& key)
{
    ItemList<>* pCheckedItems;
    if(checkId >= impl->checkStates.size()){
        return false;
    }
    auto& items = impl->checkStates[checkId]->checkedItems;
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
    if(checkId < impl->checkStates.size()){
        if(auto checkState = impl->checkStates[checkId]){
            auto& checkedItems = checkState->checkedItems;
            if(checkState->needToUpdateCheckedItemList){
                checkedItems.clear();
                impl->updateCheckedItemsIter(this, checkId, checkedItems);
                checkState->needToUpdateCheckedItemList = false;
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


void RootItem::notifyEventOnItemCheckToggled(Item* item, int checkId, bool on)
{
    if(checkId < impl->checkStates.size()){
        if(auto checkState = impl->checkStates[checkId]){
            checkState->needToUpdateCheckedItemList = true;
            checkState->sigCheckToggled(item, on);
        }
    }
}


SignalProxy<void(Item* item, bool on)> RootItem::sigCheckToggled(int checkId)
{
    if(checkId < impl->checkStates.size()){
        if(auto checkState = impl->checkStates[checkId]){
            return checkState->sigCheckToggled;
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
