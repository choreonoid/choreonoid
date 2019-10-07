#include "TargetItemPicker.h"
#include "ItemTreeView.h"
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
    ScopedConnection targetItemConnection;
    bool isBeforeAnyItemChangeNotification;
    View* view;
    ScopedConnectionSet viewConnections;
    ItemTreeView* itemTreeView;
    ScopedConnection itemSelectionChangeConnection;
    ScopedConnection itemAddedConnection;

    Impl(TargetItemPickerBase* self, View* view);
    void onViewActivated();
    void onViewDeactivate();
    void onItemSelectionChanged();
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
    isBeforeAnyItemChangeNotification = true;
    
    itemTreeView = ItemTreeView::instance();

    viewConnections.add(
        view->sigActivated().connect([&](){ onViewActivated(); }));
    viewConnections.add(
        view->sigDeactivated().connect([&](){ onViewDeactivate(); }));
}


TargetItemPickerBase::~TargetItemPickerBase()
{
    delete impl;
}


void TargetItemPickerBase::Impl::onViewActivated()
{
    itemSelectionChangeConnection.reset(
        itemTreeView->sigSelectionChanged().connect(
            [&](const ItemList<>&){ onItemSelectionChanged(); }));
                
    onItemSelectionChanged();
}


void TargetItemPickerBase::Impl::onViewDeactivate()
{
    itemSelectionChangeConnection.disconnect();
    itemAddedConnection.disconnect();
}
        

void TargetItemPickerBase::Impl::onItemSelectionChanged()
{
    auto selectedItems = itemTreeView->selectedItems();
    self->extractTargetItemCandidates(selectedItems);
    setTargetItem(selectedItems.toSingle(true), true, false);
}


Item* TargetItemPickerBase::getTargetItem()
{
    return impl->targetItem;
}


void TargetItemPickerBase::Impl::setTargetItem(Item* item, bool doNotify, bool updateEvenIfEmpty)
{
    if((item != targetItem && (item || updateEvenIfEmpty)) || isBeforeAnyItemChangeNotification){
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
            self->onTargetItemChanged(targetItem);
        }
    }

    if(!targetItem){
        ItemList<> items;
        items.extractChildItems(RootItem::instance());
        self->extractTargetItemCandidates(items);
        if(auto candidate = items.toSingle(true)){
            setTargetItem(candidate, true, false);
            return;
        }

        if(!targetItem && !itemAddedConnection.connected()){
        itemAddedConnection.reset(
            RootItem::instance()->sigItemAdded().connect(
                [&](Item* item){ onItemAddedWhenNoTargetItemSpecified(item); }));
        }
    }
}


void TargetItemPickerBase::Impl::onItemAddedWhenNoTargetItemSpecified(Item* item)
{
    if(!targetItem){
        ItemList<> items;
        items.push_back(item);
        self->extractTargetItemCandidates(items);
        if(auto candidate = items.toSingle(true)){
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
    

void TargetItemPickerBase::storeTargetItem(Archive& archive, const std::string& key)
{
    if(impl->targetItem){
        archive.writeItemId(key, impl->targetItem);
    }
}


void TargetItemPickerBase::restoreTargetItemLater(const Archive& archive, const std::string& key)
{
    archive.addPostProcess([&, key](){ impl->restoreTargetItem(archive, key); });
}


void TargetItemPickerBase::Impl::restoreTargetItem(const Archive& archive, const std::string& key)
{
    auto item = archive.findItem<Item>(key);
    setTargetItem(item, true, false);
}
