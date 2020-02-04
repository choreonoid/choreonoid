#include "TargetItemPicker.h"
#include "View.h"
#include "RootItem.h"
#include "Archive.h"
#include <cnoid/ConnectionSet>

using namespace std;
using namespace cnoid;

namespace cnoid {

class TargetItemPickerBase::Impl
{
public:
    TargetItemPickerBase* self;
    ItemPtr targetItem;
    ItemPtr preferredTargetItem; // Use this as the target when no candidate item is selected
    ItemList<> tmpSelectedItems;
    ScopedConnection targetItemConnection;
    bool isActive;
    bool isBeforeAnyItemChangeNotification;
    View* view;
    ScopedConnectionSet viewConnections;
    RootItem* rootItem;
    ScopedConnection itemSelectionChangeConnection;
    ScopedConnection itemAddedConnection;

    Impl(TargetItemPickerBase* self, View* view);
    void activate(bool doUpdate);
    void deactivate();
    void onSelectedItemsChanged(const ItemList<>& items);
    void setTargetItem(Item* item, bool doNotify, bool updateEvenIfEmpty);
    void onItemAddedWhenNoTargetItemSpecified(Item* item);
    void onTargetItemDisconnectedFromRoot(Item* item);
    void restoreTargetItem(const Archive& archive, const std::string& key);    
};

}


TargetItemPickerBase::TargetItemPickerBase(View* view)
{
    impl = new Impl(this, view);
}


TargetItemPickerBase::Impl::Impl(TargetItemPickerBase* self, View* view)
    : self(self),
      view(view)
{
    isActive = false;
    isBeforeAnyItemChangeNotification = true;
    
    rootItem = RootItem::instance();

    if(view){
        viewConnections.add(
            view->sigActivated().connect([&](){ activate(true); }));
        viewConnections.add(
            view->sigDeactivated().connect([&](){ deactivate(); }));

    } else {
        activate(false);
    }
}


TargetItemPickerBase::~TargetItemPickerBase()
{
    delete impl;
}


void TargetItemPickerBase::Impl::activate(bool doUpdate)
{
    itemSelectionChangeConnection.reset(
        rootItem->sigSelectedItemsChanged().connect(
            [&](const ItemList<>& items){ onSelectedItemsChanged(items); }));

    isActive = true;

    if(doUpdate){
        onSelectedItemsChanged(rootItem->selectedItems());
    }
}


void TargetItemPickerBase::Impl::deactivate()
{
    itemSelectionChangeConnection.disconnect();
    itemAddedConnection.disconnect();

    isActive = false;

    self->onDeactivated();

    if(targetItem){
        setTargetItem(nullptr, true, true);
    }
}


void TargetItemPickerBase::Impl::onSelectedItemsChanged(const ItemList<>& items)
{
    tmpSelectedItems = items;
    auto candidate = self->extractTargetItemCandidates(
        tmpSelectedItems, rootItem->focusedItem(), true);
    setTargetItem(candidate, true, false);
    tmpSelectedItems.clear();
}


Item* TargetItemPickerBase::getTargetItem()
{
    return impl->targetItem;
}


void TargetItemPickerBase::Impl::setTargetItem(Item* item, bool doNotify, bool updateEvenIfEmpty)
{
    if((item != targetItem && (item || updateEvenIfEmpty)) || (item && isBeforeAnyItemChangeNotification)){
        targetItemConnection.disconnect();
        targetItem = item;
        if(targetItem){
            auto itemToCheck = targetItem;
            targetItemConnection.reset(
                itemToCheck->sigDisconnectedFromRoot().connect(
                    [this, itemToCheck](){ onTargetItemDisconnectedFromRoot(itemToCheck); }));
            itemAddedConnection.disconnect();
        }
        if(doNotify){
            isBeforeAnyItemChangeNotification = false;
            self->onTargetItemSpecified(targetItem, true);
        }
    } else if(item || updateEvenIfEmpty){
        self->onTargetItemSpecified(targetItem, false);
    }

    if(!targetItem && !updateEvenIfEmpty){
        auto items = rootItem->descendantItems();
        auto candidate = self->extractTargetItemCandidates(items, preferredTargetItem, false);
        if(candidate == preferredTargetItem){
            preferredTargetItem.reset();
        }
        if(candidate){
            setTargetItem(candidate, true, false);
            return;
        }

        if(!targetItem && !itemAddedConnection.connected()){
            itemAddedConnection.reset(
                rootItem->sigItemAdded().connect(
                    [&](Item* item){ onItemAddedWhenNoTargetItemSpecified(item); }));
        }
    }
}


void TargetItemPickerBase::Impl::onItemAddedWhenNoTargetItemSpecified(Item* item)
{
    if(!targetItem){
        ItemList<> items;
        items.push_back(item);
        if(auto candidate = self->extractTargetItemCandidates(items, nullptr, false)){
            setTargetItem(candidate, true, false);
        }
    }
    if(targetItem){
        itemAddedConnection.disconnect();
    }
}


void TargetItemPickerBase::Impl::onTargetItemDisconnectedFromRoot(Item* item)
{
    if(item == targetItem){
        setTargetItem(nullptr, true, true);
    }
}


void TargetItemPickerBase::clearTargetItem()
{
    if(impl->targetItem){
        impl->setTargetItem(nullptr, false, true);
    }
}


void TargetItemPickerBase::storeTargetItem(Archive& archive, const std::string& key)
{
    if(impl->targetItem){
        archive.writeItemId(key, impl->targetItem);
    }
}


void TargetItemPickerBase::restoreTargetItem(const Archive& archive, const std::string& key)
{
    auto item = archive.findItem<Item>(key);
    if(impl->isActive){
        impl->setTargetItem(item, true, false);
    } else {
        impl->preferredTargetItem = item;
    }
}


void TargetItemPickerBase::restoreTargetItemLater(const Archive& archive, const std::string& key)
{
    archive.addPostProcess([&, key](){ restoreTargetItem(archive, key); });
}
