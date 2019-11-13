/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_ITEM_H
#define CNOID_BASE_ITEM_H

#include "PutPropertyFunction.h"
#include <cnoid/Referenced>
#include <cnoid/Signal>
#include <cnoid/NullOut>
#include <ctime>
#include <bitset>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class Item;
typedef ref_ptr<Item> ItemPtr;
    
class RootItem;
class Archive;
class ExtensionManager;


class CNOID_EXPORT Item : public Referenced
{
protected:
    Item();
    Item(const Item& item);
    
public:

    enum Attribute {
        SUB_ITEM,
        TEMPORAL,
        LOAD_ONLY,
        NUM_ATTRIBUTES
    };

    virtual ~Item(); // The destructor should not be called in usual ways

    const std::string& name() const { return name_; }
    virtual void setName(const std::string& name);

    bool hasAttribute(Attribute attribute) const { return attributes[attribute]; }

    Item* childItem() const { return firstChild_; }
    Item* prevItem() const { return prevItem_; }
    Item* nextItem() const { return nextItem_; }
    Item* parentItem() const { return parent_; }

    bool addChildItem(Item* item, bool isManualOperation = false);
    bool addSubItem(Item* item);
    bool isSubItem() const;
    void detachFromParentItem();
    bool insertChildItem(Item* item, Item* nextItem, bool isManualOperation = false);
    bool insertSubItem(Item* item, Item* nextItem);

    bool isTemporal() const;
    void setTemporal(bool on = true);

    static Item* rootItem();
    RootItem* findRootItem() const;
    bool isConnectedToRoot() const;
    Item* getLocalRootItem() const;

    /**
       Find an item that has the corresponding path to it in the sub tree
    */
    Item* findItem(const std::string& path) const;
    template<class ItemType>
    ItemType* findItem(const std::string& path) const {
        return dynamic_cast<ItemType*>(findItem(path));
    }

    static Item* find(const std::string& path);
    template<class ItemType>
    ItemType* find(const std::string& path) {
        return dynamic_cast<ItemType*>(find(path));
    }
    
    /**
       Find an item that has the corresponding path from a child item to it
    */
    Item* findChildItem(const std::string& path) const;
    template<class ItemType>
    ItemType* findChildItem(const std::string& path) const {
        return dynamic_cast<ItemType*>(findChildItem(path));
    }

    /**
       Find a sub item that has the corresponding path from a direct sub item to it
    */
    Item* findSubItem(const std::string& path) const;
    template<class ItemType>
    ItemType* findSubItem(const std::string& path) const {
        return dynamic_cast<ItemType*>(findSubItem(path));
    }
    
    /*
      The function 'template <class ItemType> ItemList<ItemType> getSubItems() const'
      has been removed. Please use ItemList::extractChildItems(Item* item) instead of it.
    */

    Item* headItem() const;

    template <class ItemType> ItemType* findOwnerItem(bool includeSelf = false) const {
        Item* parentItem__ = includeSelf ? const_cast<Item*>(this) : parentItem();
        while(parentItem__){
            ItemType* ownerItem = dynamic_cast<ItemType*>(parentItem__);
            if(ownerItem){
                return ownerItem;
            }
            parentItem__ = parentItem__->parentItem();
        }
        return nullptr;
    }

    bool isOwnedBy(Item* item) const;

    bool traverse(std::function<bool(Item*)> function);

    template<class ItemType>
    bool traverse(std::function<bool(ItemType* item)> function){
        return Item::traverse(
            [&function](Item* item){
                if(auto* casted = dynamic_cast<ItemType*>(item)){
                    return function(casted);
                }
                return false;
            });
    }

    Item* duplicate() const;
    Item* duplicateAll() const;

    void assign(Item* srcItem);

    bool load(const std::string& filename, const std::string& format = std::string());
    bool load(const std::string& filename, Item* parent, const std::string& format = std::string());
    bool save(const std::string& filename, const std::string& format = std::string());
    bool overwrite(bool forceOverwrite = false, const std::string& format = std::string());

    const std::string& filePath() const;
    const std::string& fileFormat() const;
    std::time_t fileModificationTime() const;
    bool isConsistentWithFile() const;

    void updateFileInformation(const std::string& filename, const std::string& format);
    void setConsistentWithFile(bool isConsistent);
    void suggestFileUpdate();
    void clearFileInformation();

#ifdef CNOID_BACKWARD_COMPATIBILITY
    const std::string& lastAccessedFilePath() const;
    const std::string& lastAccessedFileFormatId() const;
#endif

    void putProperties(PutPropertyFunction& putProperty);

    virtual void notifyUpdate();

    SignalProxy<void(const std::string& oldName)> sigNameChanged() {
        return sigNameChanged_;
    }

    /**
       \todo Remove this signal and define 'sigPropertyChanged' instead of it
    */
    SignalProxy<void()> sigUpdated() {
        return sigUpdated_;
    }

    /**
       This signal is emitted when the position of this item in the item tree is changed.
       Being added to the tree and being removed from the tree are also the events
       to emit this signal.
       This signal is also emitted for descendent items when the position of an ancestor
       item is changed.
       This signal is emitted before RootItem::sigTreeChanged();
    */
    SignalProxy<void()> sigPositionChanged() {
        return sigPositionChanged_;
    }

    /**
       @note deprecated
    */
    SignalProxy<void()> sigDetachedFromRoot() {
        return sigDisconnectedFromRoot_;
    }

    /**
       @note Please use this instead of sigDetachedFromRoot()
    */
    SignalProxy<void()> sigDisconnectedFromRoot() {
        return sigDisconnectedFromRoot_;
    }

    SignalProxy<void()> sigSubTreeChanged() {
        return sigSubTreeChanged_;
    }

    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

protected:
    /**
       This function is called when the item has been connected to the tree including the root item.
       The onPositionChanged function and sigSubTreeChanged are processed before calling this function.
    */
    virtual void onConnectedToRoot();
    virtual void onDisconnectedFromRoot();
    virtual void onPositionChanged();
    virtual bool onChildItemAboutToBeAdded(Item* childItem, bool isManualOperation);
        
    virtual Item* doDuplicate() const;
    virtual void doAssign(Item* srcItem);
    virtual void doPutProperties(PutPropertyFunction& putProperty);

    void setAttribute(Attribute attribute) { attributes.set(attribute); }
    void unsetAttribute(Attribute attribute) { attributes.reset(attribute); }

private:

    Item* parent_;
    ItemPtr firstChild_;
    ItemPtr nextItem_;
    Item* prevItem_;
    Item* lastChild_;

    int numChildren_;

    std::string name_;

    std::bitset<NUM_ATTRIBUTES> attributes;

    Signal<void(const std::string& oldName)> sigNameChanged_;
    Signal<void()> sigDisconnectedFromRoot_;
    Signal<void()> sigUpdated_;
    Signal<void()> sigPositionChanged_;
    Signal<void()> sigSubTreeChanged_;

    // for file overwriting management, mainly accessed by ItemManagerImpl
    bool isConsistentWithFile_;
    std::string filePath_;
    std::string fileFormat_;
    std::time_t fileModificationTime_;

    // disable the assignment operator
    Item& operator=(const Item& rhs);

    void init();
    bool doInsertChildItem(ItemPtr item, Item* newNextItem, bool isManualOperation);
    void callSlotsOnPositionChanged();
    void callFuncOnConnectedToRoot();
    void addToItemsToEmitSigSubTreeChanged();
    void emitSigDisconnectedFromRootForSubTree();
    static void emitSigSubTreeChanged();

    void detachFromParentItemSub(bool isMoving);
    bool traverse(Item* item, const std::function<bool(Item*)>& function);
    Item* duplicateAllSub(Item* duplicated) const;
        
    //friend class ItemManagerImpl;
};

#ifndef CNOID_BASE_MVOUT_DECLARED
#define CNOID_BASE_MVOUT_DECLARED
CNOID_EXPORT std::ostream& mvout(bool doFlush = false);
#endif

}

#endif
