/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "Item.h"
#include "ItemAddon.h"
#include "RootItem.h"
#include "ItemPath.h"
#include "ItemManager.h"
#include "ItemClassRegistry.h"
#include "PutPropertyFunction.h"
#include "LazyCaller.h"
#include <cnoid/ValueTree>
#include <cnoid/UTF8>
#include <cnoid/stdx/filesystem>
#include <typeinfo>
#include <bitset>
#include <vector>
#include <map>
#include <unordered_set>
#include <unordered_map>
#include <typeindex>
#include <iostream> // for debug
#include "gettext.h"

using namespace std;
using namespace cnoid;
namespace filesystem = cnoid::stdx::filesystem;

namespace {

const bool TRACE_FUNCTIONS = false;

/**
   \note The element type of the following container must be a raw pointer
   because a newly created item that is not contained in a smart pointer
   may be set to the container and such an item is deleted when the container
   is cleared if its element type is the smart pointer.
*/
unordered_set<Item*> itemsToEmitSigSubTreeChanged;

vector<Item*> tmpItemArray;

int recursiveTreeChangeCounter = 0;
bool isAnyItemInSubTreesBeingAddedOrRemovedSelected = false;

std::map<ItemPtr, ItemPtr> replacementToOriginalItemMap;
std::map<ItemPtr, ItemPtr> originalToReplacementItemMap;

LazyCaller clearItemReplacementMapsLater(
    [](){
        replacementToOriginalItemMap.clear();
        originalToReplacementItemMap.clear();
    },
    LazyCaller::MinimumPriority);
        

bool checkIfAnyItemInSubTreeSelected(Item* item)
{
    if(item->isSelected()){
        return true;
    }
    for(auto child = item->childItem(); child; child = child->nextItem()){
        return checkIfAnyItemInSubTreeSelected(child);
    }
    return false;
}

}

namespace cnoid {

class Item::Impl
{
public:
    Item* self;
    bitset<NumAttributes> attributes;
    vector<bool> checkStates;
    std::function<std::string(const Item* item)> displayNameModifier;
    
    std::unordered_map<std::type_index, ItemAddonPtr> addonMap;

    Signal<void(const std::string& oldName)> sigNameChanged;
    Signal<void()> sigDisconnectedFromRoot;
    Signal<void()> sigUpdated;
    Signal<void()> sigTreePathChanged;
    Signal<void()> sigTreePositionChanged;
    Signal<void(Item* topItem, Item* prevTopParentItem)> sigTreePositionChanged2;
    Signal<void()> sigSubTreeChanged;
    Signal<void(bool on)> sigSelectionChanged;
    Signal<void(int checkId, bool on)> sigAnyCheckToggled;
    Signal<void(bool on)> sigLogicalSumOfAllChecksToggled;
    map<int, Signal<void(bool on)>> checkIdToSignalMap;

    // for file overwriting management, mainly accessed by ItemManager::Impl
    bool isConsistentWithFile;
    std::string filePath;
    std::string fileFormat;
    MappingPtr fileOptions;
    std::time_t fileModificationTime;

    Impl(Item* self);
    Impl(Item* self, const Impl& org);
    void initialize();
    ~Impl();
    void notifyNameChange(const std::string& oldName);
    void setSelected(bool on, bool forceToNotify, bool doEmitSigSelectedItemsChangedLater);
    bool setSubTreeItemsSelectedIter(Item* item, bool on);
    int countDescendantItems(const Item* item) const;
    Item* findItem(const std::function<bool(Item* item)>& pred, bool isRecursive) const;
    Item* findItem(
        ItemPath::iterator iter, ItemPath::iterator end,  const std::function<bool(Item* item)>& pred,
        bool isRecursive) const;
    void getDescendantItemsIter(
        const Item* parentItem, ItemList<>& io_items, const std::function<bool(Item* item)>& pred,
        bool isRecursive) const;
    void getSelectedDescendantItemsIter(
        const Item* parentItem, ItemList<>& io_items, std::function<bool(Item* item)> pred) const;
    Item* duplicateSubTreeIter(Item* duplicated, Item* duplicatedParent) const;
    bool doInsertChildItem(ItemPtr item, Item* newNextItem, bool isManualOperation);
    void justInsertChildItem(Item* newNextItem, Item* item);
    bool checkNewTreePositionAcceptance(
        Item* newParentItem, Item* newNextItem, bool isManualOperation, vector<function<void()>>& callbacksWhenAdded);
    bool checkNewTreePositionAcceptanceIter(bool isManualOperation, vector<function<void()>>& callbacksWhenAdded);
    void collectSubTreeItems(vector<Item*>& items, Item* item);
    void callFuncOnConnectedToRoot();
    void justRemoveSelfFromParent();
    void doRemoveFromParentItem(bool isMoving, bool isParentBeingDeleted);
    void notifySubTreeItemsOfTreePositionChange(Item* prevParentItem, Item* prevNextSibling);
    void notifySubTreeItemsOfTreePositionChangeIter(
        Item* topItem, Item* prevTopParentItem, Item* topPathChangedItem, bool isPathChanged = false);
    void addToItemsToEmitSigSubTreeChanged();
    static void emitSigSubTreeChanged();
    void emitSigDisconnectedFromRootForSubTree();
    bool traverse(Item* item, const std::function<bool(Item*)>& pred);
};

}


Item::Item()
{
    impl = new Impl(this);
}


Item::Item(const std::string& name)
    : name_(name)
{
    impl = new Impl(this);
}


Item::Impl::Impl(Item* self)
    : self(self)
{
    initialize();
}


Item::Item(const Item& org)
    : name_(org.name_)
{
    impl = new Impl(this, *org.impl);
}


Item::Impl::Impl(Item* self, const Impl& org)
    : self(self),
      attributes(org.attributes)
{
    initialize();

    if(attributes[LoadOnly]){
        filePath = org.filePath;
        fileFormat = org.fileFormat;
        fileOptions = org.fileOptions;
        fileModificationTime = org.fileModificationTime;
    }
}


void Item::Impl::initialize()
{
    self->classId_ = -1;
    
    self->parent_ = nullptr;
    self->prevItem_ = nullptr;
    self->lastChild_ = nullptr;
    self->numChildren_ = 0;
    self->isSelected_ = false;

    attributes.reset(SubItem);
    attributes.reset(Temporal);

    isConsistentWithFile = false;
    fileModificationTime = 0;
}


Item::~Item()
{
    if(TRACE_FUNCTIONS){
        cout << "Item::~Item() of " << name_ << endl;
    }

    ItemPtr child = lastChild_;
    while(child){
        Item* prev = child->prevItem_;
        child->impl->doRemoveFromParentItem(false, true);
        child = prev;
    }

    delete impl;
}


Item::Impl::~Impl()
{
    auto p = addonMap.begin();
    while(p != addonMap.end()){
        auto& addon = p->second;
        addon->setOwnerItem(nullptr);
        p = addonMap.erase(p);
    }
}


void Item::validateClassId() const
{
    classId_ = ItemClassRegistry::instance().classId(this);
}


Item* Item::createNewInstance() const
{
    return ItemManager::createItem(classId());
}


void Item::assign(Item* srcItem)
{
    doAssign(srcItem);
    
    RootItem* rootItem = findRootItem();
    if(rootItem){
        rootItem->emitSigItemAssinged(this, srcItem);
    }
}


void Item::doAssign(Item* srcItem)
{
    
}


Item* Item::duplicate(Item* duplicatedParentItem) const
{
    Item* duplicated = doDuplicate(duplicatedParentItem);
    if(!duplicated){
        duplicated = doDuplicate();
    }
    if(duplicated && (typeid(*duplicated) != typeid(*this))){
        delete duplicated;
        duplicated = nullptr;
    }
    return duplicated;
}


Item* Item::doDuplicate() const
{
    return nullptr;
}


Item* Item::doDuplicate(Item* /* duplicatedParentItem */) const
{
    return nullptr;
}


Item* Item::duplicateSubTree() const
{
    return impl->duplicateSubTreeIter(nullptr, nullptr);
}


Item* Item::Impl::duplicateSubTreeIter(Item* duplicated, Item* duplicatedParent) const
{
    if(!duplicated){
        duplicated = self->duplicate(duplicatedParent);
    }
    
    if(duplicated){
        for(Item* child = self->childItem(); child; child = child->nextItem()){
            Item* duplicatedChildItem;
            if(child->isSubItem()){
                duplicatedChildItem = duplicated->findChildItem(child->name());
                if(duplicatedChildItem){
                    child->impl->duplicateSubTreeIter(duplicatedChildItem, duplicated);
                }
            } else {
                duplicatedChildItem = child->impl->duplicateSubTreeIter(nullptr, duplicated);
                if(duplicatedChildItem){
                    duplicated->addChildItem(duplicatedChildItem);
                }
            }
        }
    }

    return duplicated;
}


bool Item::setName(const std::string& name)
{
    if(name != name_){
        string oldName(name_);
        name_ = name;
        impl->notifyNameChange(oldName);
    }
    return true;
}


std::string Item::displayName() const
{
    if(impl->displayNameModifier){
        return impl->displayNameModifier(this);
    }
    return name_;
}


void Item::setDisplayNameModifier(std::function<std::string(const Item* item)> modifier)
{
    impl->displayNameModifier = modifier;
    impl->notifyNameChange(name_);
}


SignalProxy<void(const std::string& oldName)> Item::sigNameChanged()
{
    return impl->sigNameChanged;
}


void Item::Impl::notifyNameChange(const std::string& oldName)
{
    sigNameChanged(oldName);
    if(auto root = self->findRootItem()){
        root->emitSigItemNameChanged(self, oldName);
    }
}


void Item::notifyNameChange()
{
    impl->notifyNameChange(name_);
}


bool Item::hasAttribute(Attribute attribute) const
{
    return impl->attributes[attribute];
}


void Item::setAttribute(Attribute attribute)
{
    impl->attributes.set(attribute);
}


void Item::unsetAttribute(Attribute attribute)
{
    impl->attributes.reset(attribute);
}


bool Item::isSubItem() const
{
    return impl->attributes[SubItem];
}


void Item::setSubItemAttributes()
{
    impl->attributes.set(SubItem);
    impl->attributes.set(Attached);
}


bool Item::isTemporal() const
{
    return impl->attributes[Temporal];
}


void Item::setTemporal(bool on)
{
    impl->attributes.set(Temporal, on);
}


void Item::setSelected(bool on, bool isCurrent)
{
    impl->setSelected(on, isCurrent, true);
}


void Item::Impl::setSelected(bool on, bool isCurrent, bool doEmitSigSelectedItemsChangedLater)
{
    if(on != self->isSelected_ || (isCurrent && on)){
        self->isSelected_ = on;
        sigSelectionChanged(on);
        if(auto rootItem = self->findRootItem()){
            rootItem->emitSigSelectionChanged(self, on, isCurrent);
            if(doEmitSigSelectedItemsChangedLater){
                rootItem->requestToEmitSigSelectedItemsChanged();
            }
        }
    }
}    


void Item::setSubTreeItemsSelected(bool on)
{
    bool changed = impl->setSubTreeItemsSelectedIter(this, on);
    if(changed){
        if(auto rootItem = findRootItem()){
            rootItem->requestToEmitSigSelectedItemsChanged();
        }
    }
}


bool Item::Impl::setSubTreeItemsSelectedIter(Item* item, bool on)
{
    bool changed = false;
    if(on != item->isSelected()){
        item->impl->setSelected(on, false, false);
        changed = true;
    }
    for(Item* child = item->childItem(); child; child = child->nextItem()){
        if(setSubTreeItemsSelectedIter(child, on)){
            changed = true;
        }
    }
    return changed;
}


bool Item::isChecked(int checkId) const
{
    if(checkId == LogicalSumOfAllChecks){
        for(const auto& checked : impl->checkStates){
            if(checked){
                return true;
            }
        }
        return false;

    } else if(checkId < impl->checkStates.size()){
        return impl->checkStates[checkId];
    }
    
    return false;
}


void Item::setChecked(bool on)
{
    setChecked(PrimaryCheck, on);
}


void Item::setChecked(int checkId, bool on)
{
    if(checkId < 0){
        return;
    }
    
    RootItem* root = nullptr;

    if(checkId >= impl->checkStates.size()){
        root = findRootItem();
        auto tmpRoot = root ? root : RootItem::instance();
        if(checkId >= tmpRoot->numCheckEntries()){
            return;
        }
        impl->checkStates.resize(checkId + 1, false);
    }

    bool current = impl->checkStates[checkId];
    
    if(on != current){

        if(!root){
            root = findRootItem();
        }

        bool doEmitSigLogicalSumOfAllChecksToggled = false;
        bool wasAnyChecked = false;
        for(size_t i=0; i < impl->checkStates.size(); ++i){
            if(impl->checkStates[i]){
                wasAnyChecked = true;
                break;
            }
        }
        if(on != wasAnyChecked){
            doEmitSigLogicalSumOfAllChecksToggled = true;
        }
        
        impl->checkStates[checkId] = on;
        impl->checkIdToSignalMap[checkId](on);
        impl->sigAnyCheckToggled(checkId, on);

        if(doEmitSigLogicalSumOfAllChecksToggled){
            impl->sigLogicalSumOfAllChecksToggled(on);
        }
            
        if(root){
            root->emitSigCheckToggled(this, checkId, on);
            if(doEmitSigLogicalSumOfAllChecksToggled){
                root->emitSigCheckToggled(this, LogicalSumOfAllChecks, on);
            }
        }
    }
}


int Item::numCheckStates() const
{
    return impl->checkStates.size();
}


int Item::countDescendantItems() const
{
    return impl->countDescendantItems(this);
}


int Item::Impl::countDescendantItems(const Item* item) const
{
    int n = item->numChildren();
    for(Item* child = item->childItem(); child; child = child->nextItem()){
        n += countDescendantItems(child);
    }
    return n;
}
        

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


/*
Item* Item::rootItem()
{
    return RootItem::instance();
}
*/


RootItem* Item::findRootItem() const
{
    return dynamic_cast<RootItem*>(localRootItem());
}


bool Item::isConnectedToRoot() const
{
    return findRootItem() != nullptr;
}


Item* Item::localRootItem() const
{
    Item* current = const_cast<Item*>(this);
    while(current->parent_){
        current = current->parent_;
    }
    return current;
}


bool Item::addChildItem(Item* item, bool isManualOperation)
{
    return impl->doInsertChildItem(item, nullptr, isManualOperation);
}


bool Item::insertChild(Item* position, Item* item, bool isManualOperation)
{
    return impl->doInsertChildItem(item, position, isManualOperation);
}


bool Item::insertChildItem(Item* item, Item* nextItem, bool isManualOperation)
{
    return impl->doInsertChildItem(item, nextItem, isManualOperation);
}


bool Item::addSubItem(Item* item)
{
    item->setSubItemAttributes();
    return impl->doInsertChildItem(item, nullptr, false);
}


bool Item::insertSubItem(Item* item, Item* nextItem)
{
    item->setSubItemAttributes();
    return impl->doInsertChildItem(item, nextItem, false);
}


bool Item::Impl::doInsertChildItem(ItemPtr item, Item* newNextItem, bool isManualOperation)
{
    RootItem* rootItem = self->findRootItem();
    Item* prevParentItem = item->parentItem();
    Item* prevNextSibling = nullptr;
    
    if(prevParentItem){
        prevNextSibling = item->nextItem();
        if(prevParentItem == self && prevNextSibling == newNextItem){
            return true; // try to insert in the same position as now
        }
    }

    if(!self->onChildItemAboutToBeAdded(item, isManualOperation)){
        return false; // rejected
    }

    vector<function<void()>> callbacksWhenAdded;
    if(!item->impl->checkNewTreePositionAcceptance(
           self, newNextItem, isManualOperation, callbacksWhenAdded)){
        return false;
    }

    bool isMoving = false;

    if(prevParentItem){
        if(auto srcRootItem = prevParentItem->findRootItem()){
            if(srcRootItem == rootItem){
                isMoving = true;
            }
        }
        item->impl->doRemoveFromParentItem(isMoving, false);
    }

    ++recursiveTreeChangeCounter;

    if(checkIfAnyItemInSubTreeSelected(item)){
        isAnyItemInSubTreesBeingAddedOrRemovedSelected = true;
    }
    
    if(self->isTemporal()){
        if(isManualOperation && !item->impl->attributes[SubItem]){
            self->setTemporal(false);
        }
    }

    justInsertChildItem(newNextItem, item);


    for(auto& callback : callbacksWhenAdded){
        callback();
    }

    item->onAddedToParent();

    if(rootItem){
        /*
          The items contained in the original sub tree. New items may be added to the sub tree
          in the following event handler functions. For example, SubProjectItem loads its own
          items in its onConnectedToRoot function. For those items, emitting the RootItem::sigItemAdded
          signal twice must be avoided and the following variable is used for it.
        */
        vector<Item*> orgSubTreeItems;
        collectSubTreeItems(orgSubTreeItems, item);
        
        /*
           The order to process the following notifications was modified on February 4, 2020.
           (The revision just before this modification is 2401bde85eb50340ea5ea1e915d1355fa6b85582.)
           The order of calling notifySubTreeItemsOfTreePositionChange was changed to before
           notifyEventOnSubTreeAdded or notifyEventOnSubTreeMoved because Item::onPositionChanged
           is frequently used for initializing an item by itself, and the initialization outside
           the item should be processed after it. (There is a case where the condition is required.)
           Keep in mind that this change may affect what worked well before.
           Note that this order was originally used before revision a970f786c2f7021c336e3c61ad76b01c9a967e6c
           committed on October 31, 2018, and the reason why the order was changed at that time is not clear.
        */
        if(!isMoving){
            item->impl->callFuncOnConnectedToRoot();
        }

        item->impl->notifySubTreeItemsOfTreePositionChange(prevParentItem, prevNextSibling);

        if(isMoving){
            rootItem->notifyEventOnSubTreeMoved(item, orgSubTreeItems);

        } else {
            rootItem->notifyEventOnSubTreeAdded(item, orgSubTreeItems);

            for(auto& subTreeItem : orgSubTreeItems){
                if(subTreeItem->isSelected()){
                    rootItem->emitSigSelectionChanged(subTreeItem, true, false);
                }
                auto& checks = subTreeItem->impl->checkStates;
                int n = checks.size();
                for(int checkId = 0; checkId < n; ++checkId){
                    if(checks[checkId]){
                        rootItem->emitSigCheckToggled(subTreeItem, checkId, true);
                    }
                }
            }
        }
    }

    addToItemsToEmitSigSubTreeChanged();

    --recursiveTreeChangeCounter;

    if(recursiveTreeChangeCounter == 0){
        emitSigSubTreeChanged();
        if(rootItem && isAnyItemInSubTreesBeingAddedOrRemovedSelected){
            rootItem->requestToEmitSigSelectedItemsChanged();
        }
        isAnyItemInSubTreesBeingAddedOrRemovedSelected = false;
    }

    return true;
}


void Item::Impl::justInsertChildItem(Item* newNextItem, Item* item)
{
    item->parent_ = self;

    if(newNextItem && (newNextItem->parent_ == self)){
        item->nextItem_ = newNextItem;
        Item* prevItem = newNextItem->prevItem_;
        if(prevItem){
            prevItem->nextItem_ = item;
            item->prevItem_ = prevItem;
        } else {
            self->firstChild_ = item;
            item->prevItem_ = nullptr;
        }
        newNextItem->prevItem_ = item;

    } else if(self->lastChild_){
        self->lastChild_->nextItem_ = item;
        item->prevItem_ = self->lastChild_;
        item->nextItem_ = nullptr;
        self->lastChild_ = item;
    } else {
        self->firstChild_ = item;
        self->lastChild_ = item;
    }

    ++self->numChildren_;
    
}    


bool Item::Impl::checkNewTreePositionAcceptance
(Item* newParentItem, Item* newNextItem, bool isManualOperation, vector<function<void()>>& callbacksWhenAdded)
{
    auto currentParentItem = self->parent_;
    auto currentNextItem = self->nextItem_;

    if(currentParentItem){
        justRemoveSelfFromParent();
    }
    newParentItem->impl->justInsertChildItem(newNextItem, self);

    bool accepted = checkNewTreePositionAcceptanceIter(isManualOperation, callbacksWhenAdded);

    justRemoveSelfFromParent();
    if(currentParentItem){
        currentParentItem->impl->justInsertChildItem(currentNextItem, self);
    }

    return accepted;
}


bool Item::Impl::checkNewTreePositionAcceptanceIter
(bool isManualOperation, vector<function<void()>>& callbacksWhenAdded)
{
    function<void()> callback;
    if(!self->onNewTreePositionCheck(isManualOperation, callback)){
        return false;
    }
    if(callback){
        callbacksWhenAdded.push_back(callback);
    }
    for(auto child = self->childItem(); child; child = child->nextItem()){
        if(!child->impl->checkNewTreePositionAcceptanceIter(isManualOperation, callbacksWhenAdded)){
            return false;
        }
    }
    return true;
}


bool Item::onNewTreePositionCheck(bool isManualOperation, std::function<void()>& out_callbackWhenAdded)
{
    return true;
}


void Item::onAddedToParent()
{

}


bool Item::onChildItemAboutToBeAdded(Item* childItem, bool isManualOperation)
{
    return true;
}


void Item::Impl::collectSubTreeItems(vector<Item*>& items, Item* item)
{
    items.push_back(item);
    for(Item* child = item->childItem(); child; child = child->nextItem()){
        collectSubTreeItems(items, child);
    }
}


void Item::Impl::callFuncOnConnectedToRoot()
{
    self->onConnectedToRoot();
    for(Item* child = self->childItem(); child; child = child->nextItem()){
        child->impl->callFuncOnConnectedToRoot();
    }
}


void Item::Impl::justRemoveSelfFromParent()
{
    if(self->prevItem_){
        self->prevItem_->nextItem_ = self->nextItem_;
    } else {
        self->parent_->firstChild_ = self->nextItem_;
    }
    if(self->nextItem_){
        self->nextItem_->prevItem_ = self->prevItem_;
    } else {
        self->parent_->lastChild_ = self->prevItem_;
    }
    --self->parent_->numChildren_;

    self->parent_ = nullptr;
    self->prevItem_ = nullptr;
    self->nextItem_ = nullptr;
}


void Item::removeFromParentItem()
{
    ItemPtr self = this;
    impl->doRemoveFromParentItem(false, false);
}


void Item::Impl::doRemoveFromParentItem(bool isMoving, bool isParentBeingDeleted)
{
    Item* prevParent = self->parentItem();
    if(!prevParent){
        return;
    }
    Item* prevNextSibling = self->nextItem();

    RootItem* rootItem = nullptr;
    
    ++recursiveTreeChangeCounter;

    if(!isParentBeingDeleted){
        if(checkIfAnyItemInSubTreeSelected(self)){
            isAnyItemInSubTreesBeingAddedOrRemovedSelected = true;
        }
        // Clear all the selection of the sub tree to remove
        setSubTreeItemsSelectedIter(self, false);

        rootItem = self->findRootItem();
        if(rootItem){
            rootItem->notifyEventOnSubTreeRemoving(self, isMoving);
        }
        self->parent_->impl->addToItemsToEmitSigSubTreeChanged();
    }

    justRemoveSelfFromParent();

    attributes.reset(SubItem);

    self->onRemovedFromParent(prevParent, isParentBeingDeleted);

    if(rootItem){
        rootItem->notifyEventOnSubTreeRemoved(self, isMoving);
        if(!isMoving){
            notifySubTreeItemsOfTreePositionChange(prevParent, prevNextSibling);
            emitSigDisconnectedFromRootForSubTree();
        }
    }

    --recursiveTreeChangeCounter;

    if(recursiveTreeChangeCounter == 0){
        if(!isMoving){
            emitSigSubTreeChanged();
        }
        if(rootItem && isAnyItemInSubTreesBeingAddedOrRemovedSelected){
            rootItem->requestToEmitSigSelectedItemsChanged();
        }
        isAnyItemInSubTreesBeingAddedOrRemovedSelected = false;
    }
}


void Item::onRemovedFromParent(Item* /* parentItem */, bool /* isParentBeingDeleted */)
{

}


void Item::clearChildren()
{
    while(childItem()){
        childItem()->removeFromParentItem();
    }
}


void Item::Impl::notifySubTreeItemsOfTreePositionChange(Item* prevParentItem, Item* prevNextSibling)
{
    Item* newParentItem = self->parentItem();

    if(prevParentItem == newParentItem){
        if(prevNextSibling == self->nextItem()){
            return; // position has not been changed
        }
        std::unordered_set<Item*> changedItems(prevParentItem->numChildren());
        changedItems.insert(self);
        // Younger siblings at the previous position
        for(Item* sibling = prevNextSibling; sibling; sibling = sibling->nextItem()){
            changedItems.insert(sibling);
        }
        // Younger sibling at the new position
        for(Item* sibling = self->nextItem(); sibling; sibling = sibling->nextItem()){
            changedItems.insert(sibling);
        }
        for(Item* sibling = newParentItem->childItem(); sibling; sibling = sibling->nextItem()){
            if(changedItems.find(sibling) != changedItems.end()){
                sibling->impl->notifySubTreeItemsOfTreePositionChangeIter(sibling, newParentItem, nullptr);
            }
        }
    } else {
        // Check if the item at the new position is included in the sub trees of the previous younger siblings
        bool isUnderPreviousYoungerSibling = false;
        for(Item* sibling = prevNextSibling; sibling; sibling = sibling->nextItem()){
            if(self->isOwnedBy(sibling)){
                isUnderPreviousYoungerSibling = true;
                break;
            }
        }
        if(!isUnderPreviousYoungerSibling){
            notifySubTreeItemsOfTreePositionChangeIter(self, prevParentItem, self);
        }
        // Younger siblings at the previous position
        for(Item* sibling = prevNextSibling; sibling; sibling = sibling->nextItem()){
            sibling->impl->notifySubTreeItemsOfTreePositionChangeIter(sibling, prevParentItem, self);
        }
        // Younger sibling at the new position
        if(!isUnderPreviousYoungerSibling && newParentItem){
            for(Item* sibling = self->nextItem(); sibling; sibling = sibling->nextItem()){
                sibling->impl->notifySubTreeItemsOfTreePositionChangeIter(sibling, newParentItem, nullptr);
            }
        }
    }
}


void Item::Impl::notifySubTreeItemsOfTreePositionChangeIter
(Item* topItem, Item* prevTopParentItem, Item* topPathChangedItem, bool isPathChanged)
{
    if(self == topPathChangedItem){
        isPathChanged = true;
    }
    self->onTreePositionChanged();
    self->onPositionChanged();
    if(isPathChanged){
        self->onTreePathChanged();
    }
    sigTreePositionChanged();
    sigTreePositionChanged2(topItem, prevTopParentItem);
    if(isPathChanged){
        sigTreePathChanged();
    }
    for(Item* child = self->childItem(); child; child = child->nextItem()){
        child->impl->notifySubTreeItemsOfTreePositionChangeIter(
            topItem, prevTopParentItem, topPathChangedItem, isPathChanged);
    }
}


void Item::onTreePathChanged()
{

}


void Item::onTreePositionChanged()
{

}


void Item::onPositionChanged()
{

}


void Item::onConnectedToRoot()
{

}


void Item::Impl::addToItemsToEmitSigSubTreeChanged()
{
    Item* item = self;
    do {
        itemsToEmitSigSubTreeChanged.insert(item);
        item = item->parentItem();
    } while(item);
}


void Item::Impl::emitSigSubTreeChanged()
{
    tmpItemArray.resize(itemsToEmitSigSubTreeChanged.size());
    std::copy(itemsToEmitSigSubTreeChanged.begin(), itemsToEmitSigSubTreeChanged.end(), tmpItemArray.begin());
    std::sort(tmpItemArray.begin(), tmpItemArray.end(), [](Item* lhs, Item* rhs){ return !rhs->isOwnedBy(lhs); });
    
    for(auto item : tmpItemArray){
        item->impl->sigSubTreeChanged();
    }
    
    itemsToEmitSigSubTreeChanged.clear();
    tmpItemArray.clear();
}


void Item::Impl::emitSigDisconnectedFromRootForSubTree()
{
    for(Item* child = self->childItem(); child; child = child->nextItem()){
        child->impl->emitSigDisconnectedFromRootForSubTree();
    }
    sigDisconnectedFromRoot();

    self->onDisconnectedFromRoot();
}


void Item::onDisconnectedFromRoot()
{
    if(TRACE_FUNCTIONS){
        cout << "Item::onDisconnectedFromRoot() of " << name_ << endl;
    }
}


SignalProxy<void()> Item::sigTreePathChanged()
{
    return impl->sigTreePathChanged;
}


SignalProxy<void()> Item::sigTreePositionChanged()
{
    return impl->sigTreePositionChanged;
}


SignalProxy<void()> Item::sigPositionChanged()
{
    return impl->sigTreePositionChanged;
}


SignalProxy<void(Item* topItem, Item* prevTopParentItem)> Item::sigTreePositionChanged2()
{
    return impl->sigTreePositionChanged2;
}


SignalProxy<void()> Item::sigSubTreeChanged()
{
    return impl->sigSubTreeChanged;
}


SignalProxy<void()> Item::sigDisconnectedFromRoot()
{
    return impl->sigDisconnectedFromRoot;
}


SignalProxy<void(bool isSelected)> Item::sigSelectionChanged()
{
    return impl->sigSelectionChanged;
}


SignalProxy<void(int checkId, bool on)> Item::sigAnyCheckToggled()
{
    return impl->sigAnyCheckToggled;
}


SignalProxy<void(bool on)> Item::sigCheckToggled(int checkId)
{
    if(checkId == LogicalSumOfAllChecks){
        return impl->sigLogicalSumOfAllChecksToggled;
    } else {
        return impl->checkIdToSignalMap[checkId];
    }
}


Item* Item::find(const std::string& path, const std::function<bool(Item* item)>& pred)
{
    return RootItem::instance()->findItem(path, pred, true);
}


Item* Item::findItem
(const std::string& path, std::function<bool(Item* item)> pred, bool isRecursive) const
{
    ItemPath ipath(path);
    if(ipath.begin() == ipath.end()){
        return impl->findItem(pred, isRecursive);
    }
    return impl->findItem(ipath.begin(), ipath.end(), pred, isRecursive);
}


// Use the breadth-first search
Item* Item::Impl::findItem(const std::function<bool(Item* item)>& pred, bool isRecursive) const
{
    for(auto child = self->childItem(); child; child = child->nextItem()){
        if(!pred || pred(child)){
            return child;
        }
    }
    if(isRecursive){
        for(auto child = self->childItem(); child; child = child->nextItem()){
            if(auto found = child->impl->findItem(pred, isRecursive)){
                return found;
            }
        }
    }
    return nullptr;
}


// Use the breadth-first search
Item* Item::Impl::findItem
(ItemPath::iterator iter, ItemPath::iterator end,  const std::function<bool(Item* item)>& pred,
 bool isRecursive) const
{
    if(iter == end){
        if(!pred || pred(self)){
            return self;
        }
        return nullptr;
    }
    for(auto child = self->childItem(); child; child = child->nextItem()){
        if(child->name() == *iter){
            if(auto item = child->impl->findItem(iter + 1, end, pred, false)){
                return item;
            }
        }
    }
    if(isRecursive){
        for(auto child = self->childItem(); child; child = child->nextItem()){
            if(auto item = child->impl->findItem(iter, end, pred, true)){
                return item;
            }
        }
    }
    return nullptr;
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


ItemList<> Item::childItems(std::function<bool(Item* item)> pred) const
{
    return getDescendantItems(pred, false);
}


ItemList<> Item::descendantItems(std::function<bool(Item* item)> pred) const
{
    return getDescendantItems(pred, true);
}


ItemList<Item> Item::getDescendantItems(std::function<bool(Item* item)> pred, bool isRecursive) const
{
    ItemList<> items;
    impl->getDescendantItemsIter(this, items, pred, isRecursive);
    return items;
}


void Item::Impl::getDescendantItemsIter
(const Item* parentItem, ItemList<>& io_items, const std::function<bool(Item* item)>& pred,
 bool isRecursive) const
{
    for(auto child = parentItem->childItem(); child; child = child->nextItem()){
        if(!pred || pred(child)){
            io_items.push_back(child);
        }
        if(isRecursive && child->childItem()){
            getDescendantItemsIter(child, io_items, pred, isRecursive);
        }
    }
}


ItemList<> Item::selectedDescendantItems(std::function<bool(Item* item)> pred) const
{
    ItemList<> items;
    impl->getSelectedDescendantItemsIter(this, items, pred);
    return items;
}


void Item::Impl::getSelectedDescendantItemsIter
(const Item* parentItem, ItemList<>& io_items, std::function<bool(Item* item)> pred) const
{
    for(auto child = parentItem->childItem(); child; child = child->nextItem()){
        if(child->isSelected() && (!pred || pred(child))){
            io_items.push_back(child);
        }
        if(child->childItem()){
            getSelectedDescendantItemsIter(child, io_items, pred);
        }
    }
}


bool Item::traverse(std::function<bool(Item*)> pred)
{
    return impl->traverse(this, pred);
}


bool Item::Impl::traverse(Item* item, const std::function<bool(Item*)>& pred)
{
    if(pred(item)){
        return true;
    }
    for(Item* child = item->childItem(); child; child = child->nextItem()){
        if(traverse(child, pred)){
            return true;
        }
    }
    return false;
}


void Item::notifyUpdate()
{
    impl->sigUpdated();
}


SignalProxy<void()> Item::sigUpdated()
{
    return impl->sigUpdated;
}


bool Item::setAddon(ItemAddon* addon)
{
    bool accepted = false;
    if(auto owner = addon->ownerItem()){
        if(owner == this){
            accepted = true;
        } else {
            owner->removeAddon(addon);
        }
    } else {
        accepted = addon->setOwnerItem(this);
    }
    if(accepted){
        impl->addonMap[typeid(*addon)] = addon;
    }
    return accepted;
}


void Item::removeAddon(ItemAddon* addon)
{
    if(auto owner = addon->ownerItem()){
        if(owner == this){
            impl->addonMap.erase(typeid(*addon));
            addon->setOwnerItem(nullptr);
        }
    }
}


ItemAddon* Item::findAddon_(const std::type_info& type)
{
    auto p = impl->addonMap.find(type);
    if(p != impl->addonMap.end()){
        return p->second;
    }
    return nullptr;
}


ItemAddon* Item::getAddon_(const std::type_info& type)
{
    ItemAddonPtr addon = findAddon_(type);
    if(!addon){
        addon = ItemManager::createAddon(type);
        if(addon){
            if(!setAddon(addon)){
                addon.reset();
            }
        }
    }
    return addon;
}


std::vector<ItemAddon*> Item::addons()
{
    std::vector<ItemAddon*> addons_;
    addons_.reserve(impl->addonMap.size());
    for(auto& kv : impl->addonMap){
        addons_.push_back(kv.second);
    }
    return addons_;
}


bool Item::load(const std::string& filename, const std::string& format, const Mapping* options)
{
    return ItemManager::load(this, filename, parentItem(), format, options);
}


bool Item::load(const std::string& filename, Item* parent, const std::string& format, const Mapping* options)
{
    return ItemManager::load(this, filename, parent, format, options);
}


bool Item::save(const std::string& filename, const std::string& format, const Mapping* options)
{
    return ItemManager::save(this, filename, format, options);
}


bool Item::overwrite(bool forceOverwrite, const std::string& format)
{
    return ItemManager::overwrite(this, forceOverwrite, format);
}


const std::string& Item::filePath() const
{
    return impl->filePath;
}


std::string Item::fileName() const
{
    return toUTF8(filesystem::path(fromUTF8(impl->filePath)).filename().string());
}


const std::string& Item::fileFormat() const
{
    return impl->fileFormat;
}


const Mapping* Item::fileOptions() const
{
    return impl->fileOptions;
}


#ifdef CNOID_BACKWARD_COMPATIBILITY
const std::string& Item::lastAccessedFilePath() const
{
    return impl->filePath;
}


const std::string& Item::lastAccessedFileFormatId() const
{
    return impl->fileFormat;
}
#endif


std::time_t Item::fileModificationTime() const
{
    return impl->fileModificationTime;
}


bool Item::isConsistentWithFile() const
{
    return impl->isConsistentWithFile;
}


void Item::setConsistentWithFile(bool isConsistent)
{
    impl->isConsistentWithFile = isConsistent;
}


void Item::suggestFileUpdate()
{
    impl->isConsistentWithFile = false;
}


void Item::updateFileInformation(const std::string& filename, const std::string& format, Mapping* options)
{
    filesystem::path fpath(fromUTF8(filename));
    if(filesystem::exists(fpath)){
        impl->fileModificationTime = filesystem::last_write_time_to_time_t(fpath);
        impl->isConsistentWithFile = true;
    } else {
        impl->fileModificationTime = 0;
        impl->isConsistentWithFile = false;
    }        
    impl->filePath = filename;
    impl->fileFormat = format;
    impl->fileOptions = options;
}


void Item::clearFileInformation()
{
    impl->filePath.clear();
    impl->fileFormat.clear();
    impl->isConsistentWithFile = true;
}


bool Item::reload()
{
    bool reloaded = false;
    
    if(parentItem() && !isSubItem() && !filePath().empty() && !fileFormat().empty()){
        ItemPtr reloadedItem = createNewInstance();
        if(reloadedItem){
            reloadedItem->setName(name());
            if(reloadedItem->load(filePath(), fileFormat(), fileOptions())){
                reloaded = reloadedItem->replace(this);
            }
        }
    }

    return reloaded;
}


bool Item::replace(Item* originalItem)
{
    bool replaced = false;
    
    if(originalItem->parentItem() && !originalItem->isSubItem()){

        replacementToOriginalItemMap[this] = originalItem;
        originalToReplacementItemMap[originalItem] = this;

        assign(originalItem);
        
        if(originalItem->parentItem()->insertChild(originalItem, this)){

            int nc = originalItem->numCheckStates();
            for(int i=0; i < nc; ++i){
                setChecked(i, originalItem->isChecked(i));
            }
            // move children to the reload item
            ItemPtr child = originalItem->childItem();
            while(child){
                ItemPtr nextChild = child->nextItem();
                if(!child->isSubItem()){
                    child->removeFromParentItem();
                    addChildItem(child);
                }
                child = nextChild;
            }
            bool isSelected = originalItem->isSelected();
            originalItem->removeFromParentItem();
            setSelected(isSelected);
            replaced = true;
        }

        clearItemReplacementMapsLater();
    }

    return replaced;
}


Item* Item::findOriginalItem() const
{
    auto p = replacementToOriginalItemMap.find(const_cast<Item*>(this));
    if(p != replacementToOriginalItemMap.end()){
        return p->second;
    }
    return nullptr;
}


Item* Item::findReplacementItem() const
{
    auto p = originalToReplacementItemMap.find(const_cast<Item*>(this));
    if(p != originalToReplacementItemMap.end()){
        return p->second;
    }
    return nullptr;
}


void Item::putProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Name"), name_,
                [&](const string& name){
                    if(!name.empty()){
                        return setName(name);
                    }
                    return false;
                });

    auto dname = displayName();
    if(dname != name_){
        putProperty(_("Display name"), dname);
    }
    
    std::string moduleName, className;
    ItemManager::getClassIdentifier(this, moduleName, className);
    putProperty(_("Class"), className);
    
    doPutProperties(putProperty);

    if(!impl->filePath.empty()){
        putProperty(_("File"), FilePathProperty(impl->filePath));
    }

    putProperty(_("Num children"), numChildren_);
    putProperty(_("Sub item?"), isSubItem());
    putProperty(_("Temporal"), isTemporal());
    putProperty(_("Refs"), refCount());
}


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
