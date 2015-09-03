/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "Item.h"
#include "RootItem.h"
#include "ItemPath.h"
#include "ItemManager.h"
#include "MessageView.h"
#include <typeinfo>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include "gettext.h"

using namespace std;
namespace filesystem = boost::filesystem;
using namespace cnoid;

//tmp
#include <iostream>

namespace {
const bool TRACE_FUNCTIONS = false;

list<Item*> itemsToEmitSigSubTreeChanged;
}

namespace cnoid {
Signal<void(const char* type_info_name)> Item::sigClassUnregistered_;
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
    firstChild_ = 0;
    lastChild_ = 0;
    prevItem_ = 0;
    nextItem_ = 0;

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


/**
   @if jp
   アイテムの名前を設定／変更する。   

   名前が変わると、sigNameChanged シグナルが発行される。   
   @endif
*/
void Item::setName(const std::string& name)
{
    if(name != name_){
        string oldName(name_);
        name_ = name;
        sigNameChanged_(oldName);
    }
}


/**
   @if jp
   本アイテム以下のツリーに新たにアイテムを追加する。   
   
   @param item 追加するアイテム
   @endif
*/
bool Item::addChildItem(Item* item, bool isManualOperation)
{
    return doInsertChildItem(item, 0, isManualOperation);
}


/**
   @if jp
   アイテムがその不可欠な構成要素として小アイテムを持つ場合、   
   addChildItemではなく本関数を用いて小アイテムを追加しておくことで、   
   システムはその状況を把握可能となる。   
   この関数によって追加されたアイテムは isSubItem() が true となる。   
   @endif
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


bool Item::doInsertChildItem(Item* item, Item* nextItem, bool isManualOperation)
{
    if(!this->onChildItemAboutToBeAdded(item, isManualOperation)){
        return false; // rejected
    }

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

    if(nextItem && (nextItem->parent_ == this)){
        Item* prevItem = nextItem->prevItem_;
        if(prevItem){
            prevItem->nextItem_ = item;
            item->prevItem_ = prevItem;
        } else {
            firstChild_ = item;
            item->prevItem_ = 0;
        }
        nextItem->prevItem_ = item;
        item->nextItem_ = nextItem;

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
    item->addRef();

    if(rootItem){
        if(!isMoving){
            item->callFuncOnConnectedToRoot();
            // This must be before rootItem->notifyEventOnSubTreeAdded().
            item->callSlotsOnPositionChanged();
            rootItem->notifyEventOnSubTreeAdded(item);
        } else {
            // This must be before rootItem->notifyEventOnSubTreeAdded().
            item->callSlotsOnPositionChanged();
            rootItem->notifyEventOnSubTreeMoved(item);
        }
    }

    addToItemsToEmitSigSubTreeChanged();
    emitSigSubTreeChanged();

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
    for(ItemPtr child = childItem(); child; child = child->nextItem()){
        child->callSlotsOnPositionChanged();
    }
}


void Item::callFuncOnConnectedToRoot()
{
    onConnectedToRoot();
    for(ItemPtr child = childItem(); child; child = child->nextItem()){
        child->callFuncOnConnectedToRoot();
    }
}


void Item::addToItemsToEmitSigSubTreeChanged()
{
    list<Item*>::iterator pos = itemsToEmitSigSubTreeChanged.begin();
    addToItemsToEmitSigSubTreeChangedSub(pos);
}


void Item::addToItemsToEmitSigSubTreeChangedSub(list<Item*>::iterator& pos)
{
    if(parent_){
        parent_->addToItemsToEmitSigSubTreeChangedSub(pos);
    }
    pos = std::find(pos, itemsToEmitSigSubTreeChanged.end(), this);
    itemsToEmitSigSubTreeChanged.insert(pos, this);
}


void Item::emitSigSubTreeChanged()
{
    list<Item*>::reverse_iterator p;
    for(p = itemsToEmitSigSubTreeChanged.rbegin(); p != itemsToEmitSigSubTreeChanged.rend(); ++p){
        (*p)->sigSubTreeChanged_();
    }
    itemsToEmitSigSubTreeChanged.clear();
}


bool Item::isSubItem() const
{
    return attributes[SUB_ITEM];
}


/*
int Item::subItemIndex() const
{
    if(isSubItem() && parentItem()){
        int index = 0;
        for(Item* sibling = parent->childItem(); sibling = sibling->nextItem(); sibling){
            if(sibling == this){
                return index;
            }
            if(sibling->isSubItem()){
                ++index;
            }
        }
    }
    return -1;
}


Item* Item::subItem(int subItemIndex)
{
    int index = 0;
    for(Item* child = parent->childItem(); child = sibling->nextItem(); child){
        if(child->isSubItem()){
            if(index == subItemIndex){
                return child;
            }
            ++index;
        }
    }
    return 0;
}
*/
            

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


/**
   @if jp
   アイテムを親アイテムから切り離す。
   @return ルートアイテムのツリー内から切り離される場合は、そのルートアイテムを返す。
   @endif
*/
void Item::detachFromParentItem()
{
    ItemPtr self = this;
    detachFromParentItemSub(false);
}


void Item::detachFromParentItemSub(bool isMoving)
{
    RootItem* rootItem = findRootItem();
  
    if(rootItem){
        rootItem->notifyEventOnSubTreeRemoving(this, isMoving);
    }


    if(parent_){
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
    
        prevItem_ = 0;
        nextItem_ = 0;

        --parent_->numChildren_;
        this->releaseRef();
        parent_ = 0;
    }
    attributes.reset(SUB_ITEM);

    if(rootItem){
        rootItem->notifyEventOnSubTreeRemoved(this, isMoving);
        if(!isMoving){
            callSlotsOnPositionChanged(); // sigPositionChanged is also emitted
            emitSigDetachedFromRootForSubTree();
        }
    }
    if(!isMoving){
        emitSigSubTreeChanged();
    }
}


void Item::emitSigDetachedFromRootForSubTree()
{
    for(ItemPtr child = childItem(); child; child = child->nextItem()){
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


RootItem* Item::findRootItem() const
{
    Item* current = const_cast<Item*>(this);
    while(current->parent_){
        current = current->parent_;
    }
    return dynamic_cast<RootItem*>(current);
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
    while(current = current->parent_){
        if(current == item){
            return true;
        }
    }
    return false;
}


bool Item::traverse(boost::function<bool(Item*)> function)
{
    return traverse(this, function);
}


bool Item::traverse(Item* item, const boost::function<bool(Item*)>& function)
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


/**
   @if jp
   アイテムのコピーを生成する。   
   小アイテムについては isFixedToParentItem() が true のときはコピーされるが、   
   false のときはコピーされない。   
   @endif
*/
ItemPtr Item::duplicate() const
{
    ItemPtr duplicated = doDuplicate();
    if(duplicated && (typeid(*duplicated) != typeid(*this))){
        duplicated = 0;
    }
    return duplicated;
}


/**
   @if jp
   小アイテム（サブツリー）も含めたアイテムのコピーを生成する。   
   @endif
*/
ItemPtr Item::duplicateAll() const
{
    return duplicateAllSub(0);
}


ItemPtr Item::duplicateAllSub(ItemPtr duplicated) const
{
    if(!duplicated){
        duplicated = this->duplicate();
    }
    
    if(duplicated){
        for(ItemPtr child = childItem(); child; child = child->nextItem()){
            ItemPtr duplicatedChildItem;
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
ItemPtr Item::doDuplicate() const
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


void Item::updateFileInformation(const std::string& filename, const std::string& format)
{
    filesystem::path fpath(filename);
    if(filesystem::exists(fpath)){
        filePath_ = filename;
        fileFormat_ = format;
        fileModificationTime_ = filesystem::last_write_time(fpath);
        isConsistentWithFile_ = true;
    } else {
        filePath_.clear();
        fileFormat_.clear();
        fileModificationTime_ = 0;
        isConsistentWithFile_ = false;
    }
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


const Referenced* Item::customData(int id) const
{
    if(id >= (int)extraData.size()){
        return 0;
    }
    return extraData[id].get();
}


Referenced* Item::customData(int id)
{
    if(id >= (int)extraData.size()){
        return 0;
    }
    return extraData[id].get();
}


void Item::setCustomData(int id, ReferencedPtr data)
{
    if(id >= (int)extraData.size()){
        extraData.resize(id + 1, 0);
    }
    extraData[id] = data;
}


void Item::clearCustomData(int id)
{
    if(customData(id)){
        extraData[id] = 0;
    }
}


namespace {
bool onNamePropertyChanged(Item* item, const string& name)
{
    if(!name.empty()){
        item->setName(name);
    }
    return !name.empty();
}
}


void Item::putProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Name"), name_, boost::bind(onNamePropertyChanged, this, _1));

    std::string moduleName, className;
    ItemManager::getClassIdentifier(this, moduleName, className);
    putProperty(_("Class"), className);
    
    doPutProperties(putProperty);

    if(!filePath_.empty()){
        putProperty(_("File"), filePath_);
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
