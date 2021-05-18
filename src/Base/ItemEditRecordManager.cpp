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
    ItemPtr parentItem;
    ItemPtr nextItem;
    ItemPtr oldParentItem;
    ItemPtr oldNextItem;

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
    UnifiedEditHistory* history;
    ItemPtr movingItem;
    ItemPtr oldParentItem;
    ItemPtr oldNextItem;
    ScopedConnectionSet rootItemConnections;
    unordered_map<ItemPtr, ScopedConnectionSet> itemConnectionSetMap;

    Impl();
    void onSubTreeAdded(Item* item);
    void onSubTreeRemoving(Item* item, Item* oldParentItem, Item* oldNextItem, bool isMoving);
    void onSubTreeMoved(Item* item);
    void manageSubTree(Item* item);
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
    history = UnifiedEditHistory::instance();

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


void ItemEditRecordManager::Impl::onSubTreeAdded(Item* item)
{
    auto record = new ItemTreeEditRecord(item);
    record->setItemAddition();
    history->addRecord(record);
    manageSubTree(item);
}


void ItemEditRecordManager::Impl::onSubTreeRemoving(Item* item, Item* oldParentItem, Item* oldNextItem, bool isMoving)
{
    if(isMoving){
        movingItem = item;
        this->oldParentItem = oldParentItem;
        this->oldNextItem = oldNextItem;
    } else {
        auto record = new ItemTreeEditRecord(item);
        record->setItemRemoval(oldParentItem, oldNextItem);
        history->addRecord(record);
        releaseSubTree(item);
    }
}


void ItemEditRecordManager::Impl::onSubTreeMoved(Item* item)
{
    if(item == movingItem){
        auto record = new ItemTreeEditRecord(item);
        record->setItemMove(oldParentItem, oldNextItem);
        history->addRecord(record);
    } else {
        history->clear();
        MessageView::instance()->notify(
            format(_("The edit history has been cleared because inconsistent item move operation on \"{0}\" was carried out."),
                   item->displayName()),
            MessageView::Error);
    }
    movingItem.reset();
    oldParentItem.reset();
    oldNextItem.reset();
}


void ItemEditRecordManager::Impl::manageSubTree(Item* item)
{
    if(!item->isSubItem()){
        auto& connections = itemConnectionSetMap[item];
        if(connections.empty()){
            connections.add(
                item->sigNameChanged().connect(
                    [this, item](const std::string& oldName){
                        onItemNameChanged(item, oldName);
                    }));
        }
        for(auto child = item->childItem(); child; child = child->nextItem()){
            manageSubTree(child);
        }
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
    history->addRecord(new ItemNameEditRecord(item, oldName));
}


namespace {

ItemTreeEditRecord::ItemTreeEditRecord(Item* item)
    : item(item)
{

}


ItemTreeEditRecord::ItemTreeEditRecord(const ItemTreeEditRecord& org)
    : EditRecord(org),
      action(org.action),
      forwardLabel(org.forwardLabel),
      reverseLabel(org.reverseLabel),
      item(org.item),
      parentItem(org.parentItem),
      nextItem(org.nextItem),
      oldParentItem(org.oldParentItem),
      oldNextItem(org.oldNextItem)
{

}


EditRecord* ItemTreeEditRecord::clone() const
{
    return new ItemTreeEditRecord(*this);
}


void ItemTreeEditRecord::setItemAddition()
{
    action = AddAction;
    parentItem = item->parentItem();
    nextItem = item->nextItem();

    forwardLabel =
        format(_("Add \"{0}\" to \"{1}\""),
               item->displayName(), parentItem->displayName());
    reverseLabel =
        format(_("Remove \"{0}\" from \"{1}\""),
               item->displayName(), parentItem->displayName());
}


bool ItemTreeEditRecord::undoAddition()
{
    if(item->parentItem() == parentItem){
        manager->rootItemConnections.block();
        item->removeFromParentItem();
        manager->rootItemConnections.unblock();
        return true;
    }
    return false;
}


bool ItemTreeEditRecord::redoAddition()
{
    if(parentItem->isConnectedToRoot()){
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
    this->oldParentItem = oldParentItem;
    this->oldNextItem = oldNextItem;

    forwardLabel =
        format(_("Remove \"{0}\" from \"{1}\""),
               item->displayName(), oldParentItem->displayName());
    reverseLabel =
        format(_("Add \"{0}\" to \"{1}\""),
               item->displayName(), oldParentItem->displayName());
}


bool ItemTreeEditRecord::undoRemoval()
{
    if(oldParentItem->isConnectedToRoot()){
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
    if(item->parentItem() == oldParentItem){
        manager->rootItemConnections.block();
        item->removeFromParentItem();
        manager->rootItemConnections.unblock();
        return true;
    }
    return false;
}


void ItemTreeEditRecord::setItemMove(Item* oldParentItem, Item* oldNextItem)
{
    action = MoveAction;
    parentItem = item->parentItem();
    nextItem = item->nextItem();
    this->oldParentItem = oldParentItem;
    this->oldNextItem = oldNextItem;

    forwardLabel =
        format(_("Move \"{0}\" from \"{1}\" to \"{2}\""),
               item->displayName(), oldParentItem->displayName(), parentItem->displayName());
    reverseLabel =
        format(_("Move \"{0}\" from \"{1}\" to \"{2}\""),
               item->displayName(), parentItem->displayName(), oldParentItem->displayName());
}


bool ItemTreeEditRecord::undoMove()
{
    if(item->parentItem() == parentItem && oldParentItem->isConnectedToRoot()){
        if(!oldNextItem || oldNextItem->parentItem() == oldParentItem){
            manager->rootItemConnections.block();
            bool inserted = oldParentItem->insertChild(oldNextItem, item, false);
            manager->rootItemConnections.unblock();
            return inserted;
        }
    }
    return false;
}


bool ItemTreeEditRecord::redoMove()
{
    if(item->parentItem() == oldParentItem && parentItem->isConnectedToRoot()){
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
    : item(item),
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
