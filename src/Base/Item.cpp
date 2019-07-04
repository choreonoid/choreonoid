/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "Item.h"
#include "RootItem.h"
#include "ItemPath.h"
#include "ItemManager.h"
#include <cnoid/stdx/filesystem>
#include <chrono>
#include <typeinfo>
#include <unordered_set>
#include "gettext.h"

using namespace std;
using namespace cnoid;
namespace filesystem = cnoid::stdx::filesystem;

//tmp
#include <iostream>

namespace {
const bool TRACE_FUNCTIONS = false;

unordered_set<Item*> itemsToEmitSigSubTreeChanged;

int recursiveTreeChangeCounter = 0;
unordered_set<Item*> itemsBeingAddedOrRemoved;

}


Item::Item()
{
    attributes = 0;
    init();
}


Item::Item(const Item& org) :
    name_(org.name_),
    attributes(org.attributes)
{
    init();

    if(attributes[LOAD_ONLY]){
        filePath_ = org.filePath_;
        fileFormat_ = org.fileFormat_;
    }
}


void Item::init()
{
    parent_ = 0;
    lastChild_ = 0;
    prevItem_ = 0;

    numChildren_ = 0;

    attributes.reset(SUB_ITEM);
    attributes.reset(TEMPORAL);

    isConsistentWithFile_ = false;
    fileModificationTime_ = 0;
}


// The assignment operator is disabled
Item& Item::operator=(const Item& rhs)
{
    return *this;
}


Item::~Item()
{
    if(TRACE_FUNCTIONS){
        cout << "Item::~Item() of " << name_ << endl;
    }
    
    sigSubTreeChanged_.disconnect_all_slots();

    Item* child = childItem();
    while(child){
        Item* next = child->nextItem();
        child->detachFromParentItem();
        child = next;
    }
}


void Item::setName(const std::string& name)
{
    if(name != name_){
        string oldName(name_);
        name_ = name;
        sigNameChanged_(oldName);
    }
}


bool Item::addChildItem(Item* item, bool isManualOperation)
{
    return doInsertChildItem(item, 0, isManualOperation);
}


/**
   This function adds a sub item to the item.
   The sub item is an item that is a required element of the main item,
   and the sub item cannot be removed from the main item.
*/
bool Item::addSubItem(Item* item)
{
    item->attributes.set(SUB_ITEM);
    return addChildItem(item, false);
}


bool Item::insertChildItem(Item* item, Item* nextItem, bool isManualOperation)
{
    return doInsertChildItem(item, nextItem, isManualOperation);
}


bool Item::insertSubItem(Item* item, Item* nextItem)
{
    item->attributes.set(SUB_ITEM);
    return doInsertChildItem(item, nextItem, false);
}


bool Item::doInsertChildItem(ItemPtr item, Item* newNextItem, bool isManualOperation)
{
    if(!this->onChildItemAboutToBeAdded(item, isManualOperation)){
        return false; // rejected
    }

    ++recursiveTreeChangeCounter;
    itemsBeingAddedOrRemoved.insert(item);
    
    if(!item->attributes[SUB_ITEM]){
        attributes.reset(TEMPORAL);
    }
    bool isMoving = false;
    RootItem* rootItem = findRootItem();
    
    if(item->parent_){
        RootItem* srcRootItem = item->parent_->findRootItem();
        if(srcRootItem){
            if(srcRootItem == rootItem){
                isMoving = true;
            }
        }
        item->detachFromParentItemSub(isMoving);
    }
        
    item->parent_ = this;

    if(newNextItem && (newNextItem->parent_ == this)){
        item->nextItem_ = newNextItem;
        Item* prevItem = newNextItem->prevItem_;
        if(prevItem){
            prevItem->nextItem_ = item;
            item->prevItem_ = prevItem;
        } else {
            firstChild_ = item;
            item->prevItem_ = 0;
        }
        newNextItem->prevItem_ = item;

    } else if(lastChild_){
        lastChild_->nextItem_ = item;
        item->prevItem_ = lastChild_;
        item->nextItem_ = 0;
        lastChild_ = item;
    } else {
        firstChild_ = item;
        lastChild_ = item;
    }

    ++numChildren_;

    if(rootItem){
        if(isMoving){
            rootItem->notifyEventOnSubTreeMoved(item);
        } else {
            rootItem->notifyEventOnSubTreeAdded(item);
        }
    }

    if(rootItem){
        if(!isMoving){
            item->callFuncOnConnectedToRoot();
        }
        if(itemsBeingAddedOrRemoved.find(this) == itemsBeingAddedOrRemoved.end()){
            item->callSlotsOnPositionChanged();
        }
    }

    addToItemsToEmitSigSubTreeChanged();

    --recursiveTreeChangeCounter;

    if(recursiveTreeChangeCounter == 0){
        emitSigSubTreeChanged();
        itemsBeingAddedOrRemoved.clear();
    }

    return true;
}

/**
   This function is called when a child item is about to added to this item.
   
   \return false if the item cannot be accepted as a child item
   \note The childItem is not actually connected to the item when this function is called.
*/
bool Item::onChildItemAboutToBeAdded(Item* childItem, bool isManualOperation)
{
    return true;
}


void Item::callSlotsOnPositionChanged()
{
    onPositionChanged();
    sigPositionChanged_();

    for(Item* child = childItem(); child; child = child->nextItem()){
        child->callSlotsOnPositionChanged();
    }
}


void Item::callFuncOnConnectedToRoot()
{
    onConnectedToRoot();
    for(Item* child = childItem(); child; child = child->nextItem()){
        child->callFuncOnConnectedToRoot();
    }
}


void Item::addToItemsToEmitSigSubTreeChanged()
{
    Item* item = this;
    do {
        itemsToEmitSigSubTreeChanged.insert(item);
        item = item->parentItem();
    } while(item);
}


void Item::emitSigSubTreeChanged()
{
    vector<Item*> items(itemsToEmitSigSubTreeChanged.size());
    std::copy(itemsToEmitSigSubTreeChanged.begin(), itemsToEmitSigSubTreeChanged.end(), items.begin());
    std::sort(items.begin(), items.end(), [](Item* lhs, Item* rhs){ return !rhs->isOwnedBy(lhs); });
    
    for(auto item : items){
        item->sigSubTreeChanged_();
    }
    
    itemsToEmitSigSubTreeChanged.clear();
}


bool Item::isSubItem() const
{
    return attributes[SUB_ITEM];
}


/**
   If this is true, the item is not automatically saved or overwritten
   when a project is saved. For example, a motion item which is produced as a
   simulation result may be an temporal item because a user may not want to
   save the result. If a user manually save the item, the item becomes a
   non-temporal item. Or if a child item is manually attached to a temporal
   item, the item becomes non-temporal one, too.
*/
bool Item::isTemporal() const
{
    return attributes[TEMPORAL];
}


void Item::setTemporal(bool on)
{
    attributes.set(TEMPORAL, on);
}


void Item::detachFromParentItem()
{
    ItemPtr self = this;
    detachFromParentItemSub(false);
}


void Item::detachFromParentItemSub(bool isMoving)
{
    if(!parent_){
        return;
    }

    ++recursiveTreeChangeCounter;
    itemsBeingAddedOrRemoved.insert(this);
    
    RootItem* rootItem = findRootItem();
  
    if(rootItem){
        rootItem->notifyEventOnSubTreeRemoving(this, isMoving);
    }

    parent_->addToItemsToEmitSigSubTreeChanged();
    if(prevItem_){
        prevItem_->nextItem_ = nextItem_;
    } else {
        parent_->firstChild_ = nextItem_;
    }
    if(nextItem_){
        nextItem_->prevItem_ = prevItem_;
    } else {
        parent_->lastChild_ = prevItem_;
    }
    
    --parent_->numChildren_;
    parent_ = 0;
    prevItem_ = 0;
    nextItem_ = 0;

    attributes.reset(SUB_ITEM);

    if(rootItem){
        rootItem->notifyEventOnSubTreeRemoved(this, isMoving);
        if(!isMoving){
            if(itemsBeingAddedOrRemoved.find(parent_) == itemsBeingAddedOrRemoved.end()){
                callSlotsOnPositionChanged(); // sigPositionChanged is also emitted
            }
            emitSigDetachedFromRootForSubTree();
        }
    }

    --recursiveTreeChangeCounter;

    if(recursiveTreeChangeCounter == 0){
        if(!isMoving){
            emitSigSubTreeChanged();
        }
        itemsBeingAddedOrRemoved.clear();
    }
}


void Item::emitSigDetachedFromRootForSubTree()
{
    for(Item* child = childItem(); child; child = child->nextItem()){
        child->emitSigDetachedFromRootForSubTree();
    }
    sigDetachedFromRoot_();

    onDisconnectedFromRoot();
}


void Item::onConnectedToRoot()
{

}


void Item::onDisconnectedFromRoot()
{
    if(TRACE_FUNCTIONS){
        cout << "Item::onDisconnectedFromRoot() of " << name_ << endl;
    }
}


void Item::onPositionChanged()
{

}


static Item* findItemSub(Item* current, ItemPath::iterator it, ItemPath::iterator end)
{
    if(it == end){
        return current;
    }
    Item* item = 0;
    for(Item* child = current->childItem(); child; child = child->nextItem()){
        if(child->name() == *it){
            item = findItemSub(child, ++it, end);
            if(item){
                break;
            }
        }
    }
    if(!item){
        for(Item* child = current->childItem(); child; child = child->nextItem()){
            item = findItemSub(child, it, end);
            if(item){
                break;
            }
        }
    }
    return item;
}


Item* Item::find(const std::string& path)
{
    return RootItem::instance()->findItem(path);
}


Item* Item::findItem(const std::string& path) const
{
    ItemPath ipath(path);
    return findItemSub(const_cast<Item*>(this), ipath.begin(), ipath.end());
}


static Item* findChildItemSub(Item* current, ItemPath::iterator it, ItemPath::iterator end)
{
    if(it == end){
        return current;
    }
    Item* item = 0;
    for(Item* child = current->childItem(); child; child = child->nextItem()){
        if(child->name() == *it){
            item = findChildItemSub(child, ++it, end);
            if(item){
                break;
            }
        }
    }
    return item;
}


Item* Item::findChildItem(const std::string& path) const
{
    ItemPath ipath(path);
    return findChildItemSub(const_cast<Item*>(this), ipath.begin(), ipath.end());
}


static Item* findSubItemSub(Item* current, ItemPath::iterator it, ItemPath::iterator end)
{
    if(it == end){
        return current;
    }
    Item* item = 0;
    for(Item* child = current->childItem(); child; child = child->nextItem()){
        if(child->name() == *it && child->isSubItem()){
            item = findSubItemSub(child, ++it, end);
            if(item){
                break;
            }
        }
    }
    return item;
}


Item* Item::findSubItem(const std::string& path) const
{
    ItemPath ipath(path);
    return findSubItemSub(const_cast<Item*>(this), ipath.begin(), ipath.end());
}


Item* Item::rootItem()
{
    return RootItem::instance();
}


RootItem* Item::findRootItem() const
{
    Item* current = const_cast<Item*>(this);
    while(current->parent_){
        current = current->parent_;
    }
    return dynamic_cast<RootItem*>(current);
}


bool Item::isConnectedToRoot() const
{
    return findRootItem() != nullptr;
}


/**
   @return When the item is embeded one,
   this function returs the first parent item which is not an embeded one.
   Otherwise the item itself is returned.
*/
Item* Item::headItem() const
{
    Item* head = const_cast<Item*>(this);
    while(head->isSubItem()){
        if(head->parent_){
            head = head->parent_;
        } else {
            break;
        }
    }
    return head;
}


bool Item::isOwnedBy(Item* item) const
{
    Item* current = const_cast<Item*>(this);
    while(current->parent_){
        current = current->parent_;
        if(current == item){
            return true;
        }
    }
    return false;
}


bool Item::traverse(std::function<bool(Item*)> function)
{
    return traverse(this, function);
}


bool Item::traverse(Item* item, const std::function<bool(Item*)>& function)
{
    if(function(item)){
        return true;
    }
    for(Item* child = item->childItem(); child; child = child->nextItem()){
        if(traverse(child, function)){
            return true;
        }
    }
    return false;
}


void Item::notifyUpdate()
{
    sigUpdated_();
}


Item* Item::duplicate() const
{
    Item* duplicated = doDuplicate();
    if(duplicated && (typeid(*duplicated) != typeid(*this))){
        delete duplicated;
        duplicated = 0;
    }
    return duplicated;
}


/**
   This function creates a copy of the item including its sub tree items.
*/
Item* Item::duplicateAll() const
{
    return duplicateAllSub(0);
}


Item* Item::duplicateAllSub(Item* duplicated) const
{
    if(!duplicated){
        duplicated = this->duplicate();
    }
    
    if(duplicated){
        for(Item* child = childItem(); child; child = child->nextItem()){
            Item* duplicatedChildItem;
            if(child->isSubItem()){
                duplicatedChildItem = duplicated->findChildItem(child->name());
                if(duplicatedChildItem){
                    child->duplicateAllSub(duplicatedChildItem);
                }
            } else {
                duplicatedChildItem = child->duplicateAllSub(0);
                if(duplicatedChildItem){
                    duplicated->addChildItem(duplicatedChildItem);
                }
            }
        }
    }

    return duplicated;
}


/**
   Override this function to allow duplication of an instance.
*/
Item* Item::doDuplicate() const
{
    return 0;
}


/**
   Copy item properties as much as possible like the assignment operator
*/
void Item::assign(Item* srcItem)
{
    doAssign(srcItem);
    RootItem* rootItem = findRootItem();
    if(rootItem){
        rootItem->emitSigItemAssinged(this, srcItem);
    }
}

/**
   Implement the code to copy properties like the assingment operator
*/
void Item::doAssign(Item* srcItem)
{
    
}


/**
   This function loads the data of the item from a file by using a pre-registered loading function.
   
   To make this function available, a loading function has to be registered to an ItemManager
   in advance by calling the addLoader() or addLoaderAndSaver() function.  Otherwise,
   this function cannot be used.
   Note that this function should not be overloaded or overridden in the derived classes.
*/
bool Item::load(const std::string& filename, const std::string& format)
{
    return ItemManager::load(this, filename, parentItem(), format);
}


/**
   @param parentItem specify this when the item is newly created one and will be attached to a parent item
   if loading succeeds.
*/
bool Item::load(const std::string& filename, Item* parent, const std::string& format)
{
    return ItemManager::load(this, filename, parent, format);
}


/**
   This function saves the data of the item to a file by using a pre-registered saving function.
   
   To make this function available, a saving function has to be registered to an ItemManager
   in advance by calling the addSaver() or addLoaderAndSaver() function.  Otherwise,
   this function cannot be used.
   Note that this function should not be overloaded or overridden in the derived classes.
*/
bool Item::save(const std::string& filename, const std::string& format)
{
    return ItemManager::save(this, filename, format);
}


/**
   This function save the data of the item to the file from which the data of the item has been loaded.
   
   If the data has not been loaded from a file, a file save dialog opens and user specifies a file.
*/
bool Item::overwrite(bool forceOverwrite, const std::string& format)
{
    return ItemManager::overwrite(this, forceOverwrite, format);
}


const std::string& Item::filePath() const
{
    return filePath_;
}


const std::string& Item::fileFormat() const
{
    return fileFormat_;
}


#ifdef CNOID_BACKWARD_COMPATIBILITY
const std::string& Item::lastAccessedFilePath() const
{
    return filePath_;
}


const std::string& Item::lastAccessedFileFormatId() const
{
    return fileFormat_;
}
#endif


std::time_t Item::fileModificationTime() const
{
    return fileModificationTime_;
}


bool Item::isConsistentWithFile() const
{
    return isConsistentWithFile_;
}


void Item::setConsistentWithFile(bool isConsistent)
{
    isConsistentWithFile_ = isConsistent;
}


void Item::suggestFileUpdate()
{
    isConsistentWithFile_ = false;
}


void Item::updateFileInformation(const std::string& filename, const std::string& format)
{
    filesystem::path fpath(filename);
    if(filesystem::exists(fpath)){
        fileModificationTime_ = filesystem::last_write_time_to_time_t(fpath);
        isConsistentWithFile_ = true;
    } else {
        fileModificationTime_ = 0;
        isConsistentWithFile_ = false;
    }        
    filePath_ = filename;
    fileFormat_ = format;
}


/**
   Use this function to disable the implicit overwrite next time
*/
void Item::clearFileInformation()
{
    filePath_.clear();
    fileFormat_.clear();
    isConsistentWithFile_ = true;
}


void Item::putProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Name"), name_,
                [&](const string& name){
                    if(!name.empty()){
                        setName(name);
                        return true;
                    }
                    return false;
                });

    std::string moduleName, className;
    ItemManager::getClassIdentifier(this, moduleName, className);
    putProperty(_("Class"), className);
    
    doPutProperties(putProperty);

    if(!filePath_.empty()){
        putProperty(_("File"), FilePathProperty(filePath_));
    }

    putProperty(_("Num children"), numChildren_);
    putProperty(_("Sub item?"), isSubItem());
    putProperty(_("Temporal"), isTemporal());
    putProperty(_("Refs"), refCount());
}


/**
   Override this function to put properties of the item.
   @note Please call doPutProperties() of the parent class in this function.
   For example, when your class directly inherits the Item class,
   call Item::doPutProperties(putProperty).
*/
void Item::doPutProperties(PutPropertyFunction& putProperty)
{

}


bool Item::store(Archive& archive)
{
    return true;
}


bool Item::restore(const Archive& archive)
{
    return true;
}
