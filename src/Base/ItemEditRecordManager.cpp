#include "ItemEditRecordManager.h"
#include "ExtensionManager.h"
#include "UnifiedEditHistory.h"
#include "ItemEditRecordManager.h"
#include "EditRecord.h"
#include "Item.h"
#include "RootItem.h"
#include "MessageView.h"
#include <cnoid/ConnectionSet>
#include <fmt/format.h>
#include <unordered_map>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

ItemEditRecordManager::Impl* manager;

class ItemTreeEditRecord : public EditRecord
{
public:
    enum Action { AddAction, MoveAction, RemoveAction };
    Action action;
    std::string forwardLabel;
    std::string reverseLabel;
    ItemPtr item;
    weak_ref_ptr<Item> parentItemRef;
    weak_ref_ptr<Item> nextItemRef;
    weak_ref_ptr<Item> oldParentItemRef;
    weak_ref_ptr<Item> oldNextItemRef;

    ItemTreeEditRecord(Item* item);
    ItemTreeEditRecord(const ItemTreeEditRecord& org);

    virtual EditRecord* clone() const override;
    virtual std::string label() const override;
    virtual bool undo() override;
    virtual bool redo() override;
    
    void setItemAddition();
    bool undoAddition();
    bool redoAddition();
    void setItemRemoval(Item* oldParentItem, Item* oldNextItem);
    bool undoRemoval();
    bool redoRemoval();
    void setItemMove(Item* oldParentItem, Item* oldNextItem);
    bool undoMove();
    bool redoMove();
};

class ItemNameEditRecord : public EditRecord
{
public:
    ItemPtr item;
    string oldName;
    string newName;

    ItemNameEditRecord(Item* item, const string& oldName);
    ItemNameEditRecord(const ItemNameEditRecord& org);
    virtual EditRecord* clone() const override;
    virtual std::string label() const override;
    virtual bool undo() override;
    virtual bool redo() override;
};

}

namespace cnoid {

class ItemEditRecordManager::Impl
{
public:
    UnifiedEditHistory* unifiedEditHistory;
    ItemPtr movingItem;
    ItemPtr oldParentItem;
    ItemPtr oldNextItem;
    ScopedConnectionSet rootItemConnections;
    unordered_map<ItemPtr, ScopedConnectionSet> itemConnectionSetMap;

    Impl();
    static bool isTargetItem(Item* item);
    void onSubTreeAdded(Item* item);
    void manageSubTree(Item* item);
    void onSubTreeRemoving(Item* item, Item* oldParentItem, Item* oldNextItem, bool isMoving);
    void onSubTreeMoved(Item* item);
    void releaseSubTree(Item* item);
    void onItemNameChanged(Item* item, const string& oldName);
};

}


void ItemEditRecordManager::initializeClass(ExtensionManager* ext)
{
    static ItemEditRecordManager* instance = new ItemEditRecordManager;
    ext->manage(instance);
    manager = instance->impl;
}


ItemEditRecordManager::ItemEditRecordManager()
{
    impl = new Impl;
}


ItemEditRecordManager::~ItemEditRecordManager()
{
    delete impl;
}


ItemEditRecordManager::Impl::Impl()
{
    unifiedEditHistory = UnifiedEditHistory::instance();
    
    auto rootItem = RootItem::instance();

    rootItemConnections.add(
        rootItem->sigSubTreeAdded().connect(
            [&](Item* item){ onSubTreeAdded(item); }));
    
    rootItemConnections.add(
        rootItem->sigSubTreeRemoving().connect(
            [&](Item* item, bool isMoving){
                onSubTreeRemoving(item, item->parentItem(), item->nextItem(), isMoving); }));

    rootItemConnections.add(
        rootItem->sigSubTreeMoved().connect(
            [&](Item* item){ onSubTreeMoved(item); }));
}


bool ItemEditRecordManager::Impl::isTargetItem(Item* item)
{
    if(item->hasAttribute(Item::ExcludedFromUnifiedEditHistory)){
        return false;
    }
    if(item->isSubItem() && !item->hasAttribute(Item::IncludedInUnifiedEditHistory)){
        return false;
    }
    return true;
}


void ItemEditRecordManager::Impl::onSubTreeAdded(Item* item)
{
    if(isTargetItem(item)){
        auto record = new ItemTreeEditRecord(item);
        record->setItemAddition();
        unifiedEditHistory->addRecord(record);
        manageSubTree(item);
    }
}


void ItemEditRecordManager::Impl::manageSubTree(Item* item)
{
    auto& connections = itemConnectionSetMap[item];
    if(connections.empty()){
        connections.add(
            item->sigNameChanged().connect(
                [this, item](const std::string& oldName){
                    onItemNameChanged(item, oldName);
                }));
    }
    for(auto child = item->childItem(); child; child = child->nextItem()){
        if(!child->isSubItem()){
            manageSubTree(child);
        }
    }
}


void ItemEditRecordManager::Impl::onSubTreeRemoving(Item* item, Item* oldParentItem, Item* oldNextItem, bool isMoving)
{
    if(isTargetItem(item)){
        if(isMoving){
            movingItem = item;
            this->oldParentItem = oldParentItem;
            this->oldNextItem = oldNextItem;
        } else {
            auto record = new ItemTreeEditRecord(item);
            record->setItemRemoval(oldParentItem, oldNextItem);
            unifiedEditHistory->addRecord(record);
            releaseSubTree(item);
        }
    }
}


void ItemEditRecordManager::Impl::onSubTreeMoved(Item* item)
{
    if(isTargetItem(item)){
        if(item == movingItem){
            auto record = new ItemTreeEditRecord(item);
            record->setItemMove(oldParentItem, oldNextItem);
            unifiedEditHistory->addRecord(record);
        } else {
            UnifiedEditHistory::instance()->clear();
            MessageView::instance()->notify(
                format(_("The edit history has been cleared because inconsistent item move operation on \"{0}\" was carried out."),
                       item->displayName()),
                MessageView::Error);
        }
        movingItem.reset();
        oldParentItem.reset();
        oldNextItem.reset();
    }
}


void ItemEditRecordManager::Impl::releaseSubTree(Item* item)
{
    itemConnectionSetMap.erase(item);
    for(auto child = item->childItem(); child; child = child->nextItem()){
        releaseSubTree(child);
    }
}


void ItemEditRecordManager::Impl::onItemNameChanged(Item* item, const string& oldName)
{
    if(item->name() != oldName){ // Is this check necessary?
        unifiedEditHistory->addRecord(new ItemNameEditRecord(item, oldName));
    }
}


namespace {

ItemTreeEditRecord::ItemTreeEditRecord(Item* item)
    : EditRecord(item),
      item(item)
{

}


ItemTreeEditRecord::ItemTreeEditRecord(const ItemTreeEditRecord& org)
    : EditRecord(org),
      action(org.action),
      forwardLabel(org.forwardLabel),
      reverseLabel(org.reverseLabel),
      item(org.item),
      parentItemRef(org.parentItemRef),
      nextItemRef(org.nextItemRef),
      oldParentItemRef(org.oldParentItemRef),
      oldNextItemRef(org.oldNextItemRef)
{

}


EditRecord* ItemTreeEditRecord::clone() const
{
    return new ItemTreeEditRecord(*this);
}


void ItemTreeEditRecord::setItemAddition()
{
    action = AddAction;
    auto parentItem = item->parentItem();
    parentItemRef = parentItem;
    nextItemRef = item->nextItem();

    forwardLabel =
        format(_("Add \"{0}\" to \"{1}\""),
               item->displayName(), parentItem->displayName());
    reverseLabel =
        format(_("Remove \"{0}\" from \"{1}\""),
               item->displayName(), parentItem->displayName());
}


bool ItemTreeEditRecord::undoAddition()
{
    if(auto parentItem = parentItemRef.lock()){
        if(item->parentItem() == parentItem){
            manager->rootItemConnections.block();
            item->removeFromParentItem();
            manager->rootItemConnections.unblock();
            return true;
        }
    }
    return false;
}


bool ItemTreeEditRecord::redoAddition()
{
    auto parentItem = parentItemRef.lock();
    auto nextItem = nextItemRef.lock();
        
    if(parentItem && parentItem->isConnectedToRoot()){
        if(!nextItem || nextItem->parentItem() == parentItem){
            manager->rootItemConnections.block();
            bool inserted = parentItem->insertChild(nextItem, item, false);
            manager->rootItemConnections.unblock();
            return inserted;
        }
    }
    return false;
}


void ItemTreeEditRecord::setItemRemoval(Item* oldParentItem, Item* oldNextItem)
{
    action = RemoveAction;
    oldParentItemRef = oldParentItem;
    oldNextItemRef = oldNextItem;

    forwardLabel =
        format(_("Remove \"{0}\" from \"{1}\""),
               item->displayName(), oldParentItem->displayName());
    reverseLabel =
        format(_("Add \"{0}\" to \"{1}\""),
               item->displayName(), oldParentItem->displayName());
}


bool ItemTreeEditRecord::undoRemoval()
{
    auto oldParentItem = oldParentItemRef.lock();
    auto oldNextItem = oldNextItemRef.lock();

    if(oldParentItem && oldParentItem->isConnectedToRoot()){
        if(!oldNextItem || oldNextItem->parentItem() == oldParentItem){
            manager->rootItemConnections.block();
            bool inserted = oldParentItem->insertChild(oldNextItem, item, false);
            manager->rootItemConnections.unblock();
            return inserted;
        }
    }
    
    return false;
}


bool ItemTreeEditRecord::redoRemoval()
{
    if(auto oldParentItem = oldParentItemRef.lock()){
        if(item->parentItem() == oldParentItem){
            manager->rootItemConnections.block();
            item->removeFromParentItem();
            manager->rootItemConnections.unblock();
            return true;
        }
    }
    return false;
}


void ItemTreeEditRecord::setItemMove(Item* oldParentItem, Item* oldNextItem)
{
    action = MoveAction;
    auto parentItem = item->parentItem();
    parentItemRef = parentItem;
    nextItemRef = item->nextItem();
    oldParentItemRef = oldParentItem;
    oldNextItemRef = oldNextItem;

    forwardLabel =
        format(_("Move \"{0}\" from \"{1}\" to \"{2}\""),
               item->displayName(), oldParentItem->displayName(), parentItem->displayName());
    reverseLabel =
        format(_("Move \"{0}\" from \"{1}\" to \"{2}\""),
               item->displayName(), parentItem->displayName(), oldParentItem->displayName());
}


bool ItemTreeEditRecord::undoMove()
{
    auto parentItem = parentItemRef.lock();
    auto oldParentItem = oldParentItemRef.lock();
    auto oldNextItem = oldNextItemRef.lock();

    if(parentItem && oldParentItem){
        if(item->parentItem() == parentItem && oldParentItem->isConnectedToRoot()){
            if(!oldNextItem || oldNextItem->parentItem() == oldParentItem){
                manager->rootItemConnections.block();
                bool inserted = oldParentItem->insertChild(oldNextItem, item, false);
                manager->rootItemConnections.unblock();
                return inserted;
            }
        }
    }
    return false;
}


bool ItemTreeEditRecord::redoMove()
{
    auto parentItem = parentItemRef.lock();
    auto nextItem = nextItemRef.lock();
    auto oldParentItem = oldParentItemRef.lock();

    if(oldParentItem && item->parentItem() == oldParentItem && parentItem && parentItem->isConnectedToRoot()){
        if(!nextItem || nextItem->parentItem() == parentItem){
            manager->rootItemConnections.block();
            bool inserted = parentItem->insertChild(nextItem, item, false);
            manager->rootItemConnections.unblock();
            return inserted;
        }
    }
    return false;
}


std::string ItemTreeEditRecord::label() const
{
    return !isReverse() ? forwardLabel : reverseLabel;
}


bool ItemTreeEditRecord::undo()
{
    switch(action){
    case AddAction:    return undoAddition();
    case MoveAction:   return undoMove();
    case RemoveAction: return undoRemoval();
    default: break;
    }
    return false;
}


bool ItemTreeEditRecord::redo()
{
    switch(action){
    case AddAction:    return redoAddition();
    case MoveAction:   return redoMove();
    case RemoveAction: return redoRemoval();
    default: break;
    }
    return false;
}


ItemNameEditRecord::ItemNameEditRecord(Item* item, const string& oldName)
    : EditRecord(item),
      item(item),
      oldName(oldName),
      newName(item->name())
{

}


ItemNameEditRecord::ItemNameEditRecord(const ItemNameEditRecord& org)
    : EditRecord(org),
      item(org.item),
      oldName(org.oldName),
      newName(org.newName)
{

}


EditRecord* ItemNameEditRecord::clone() const
{
    return new ItemNameEditRecord(*this);
}
    

std::string ItemNameEditRecord::label() const
{
    if(!isReverse()){
        return format(_("Rename \"{0}\" to \"{1}\""), oldName, newName);
    } else {
        return format(_("Rename \"{0}\" to \"{1}\""), newName, oldName);
    }
}


bool ItemNameEditRecord::undo()
{
    auto block = manager->itemConnectionSetMap[item].scopedBlock();
    return item->setName(oldName);
}
    
    
bool ItemNameEditRecord::redo()
{
    auto block = manager->itemConnectionSetMap[item].scopedBlock();
    return item->setName(newName);
}

}
