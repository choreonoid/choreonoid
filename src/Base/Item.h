/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_ITEM_H
#define CNOID_BASE_ITEM_H

#include <cnoid/Referenced>
#include <cnoid/Signal>
#include <string>
#include <vector>
#include <ctime>
#include "exportdecl.h"

namespace cnoid {

class Item;
typedef ref_ptr<Item> ItemPtr;

template<class ItemType = Item> class ItemList;
    
class ItemAddon;
class RootItem;
class Archive;
class Mapping;
class ExtensionManager;
class PutPropertyFunction;

class CNOID_EXPORT Item : public Referenced
{
protected:
    Item();
    Item(const Item& item);
    
public:
    enum Attribute {
        SubItem,
        Attached,
        Temporal,
        LoadOnly,
        NumAttributes,
        SUB_ITEM = SubItem,
        TEMPORAL = Temporal,
        LOAD_ONLY = LoadOnly
    };

    virtual ~Item();

    Item& operator=(const Item& rhs) = delete;

    int classId() const {
        if(classId_ < 0) validateClassId();
        return classId_;
    }

    //! Copy item properties as much as possible like the assignment operator
    void assign(Item* srcItem);

    Item* duplicate() const;

    //! This function creates a copy of the item including its descendant items
    Item* duplicateSubTree() const;

    [[deprecated("Use Item::duplicateSubTree.")]]
    Item* duplicateAll() const { return duplicateSubTree(); }

    const std::string& name() const { return name_; }
    // Return true if the name is actually changed
    virtual bool setName(const std::string& name);
    virtual std::string displayName() const;
    void setDisplayNameModifier(std::function<std::string(const Item* item)> modifier);
    SignalProxy<void(const std::string& oldName)> sigNameChanged();
    void notifyNameChange();

    void setAttribute(Attribute attribute);
    void unsetAttribute(Attribute attribute);
    bool hasAttribute(Attribute attribute) const;

    bool isSubItem() const;
    void setSubItemAttributes();

    /**
       If this is true, the item is not automatically saved or overwritten
       when a project is saved. For example, a motion item which is produced as a
       simulation result may be an temporal item because a user may not want to
       save the result. If a user manually save the item, the item becomes a
       non-temporal item. Or if a child item is manually attached to a temporal
       item, the item becomes non-temporal one, too.
    */
    bool isTemporal() const;
    
    void setTemporal(bool on = true);

    bool isSelected() const { return isSelected_; }
    void setSelected(bool on, bool isCurrent = false);
    void setSubTreeItemsSelected(bool on);

    /**
       \note LogicalSumOfAllChecks is only valid for "isChecked(LogicalSumOfAllChecks)" and
       the signal returned by "sigCheckToggled(LogicalSumOfAllChecks)".
    */
    enum CheckId { LogicalSumOfAllChecks = -1, PrimaryCheck = 0 };
    
    bool isChecked(int checkId = PrimaryCheck) const;
    void setChecked(bool on); // for PrimaryCheck
    void setChecked(int checkId, bool on);

    int numChildren() const { return numChildren_; }
    int countDescendantItems() const;

    Item* childItem() const { return firstChild_; }
    Item* prevItem() const { return prevItem_; }
    Item* nextItem() const { return nextItem_; }
    Item* lastChildItem() const { return lastChild_; }
    Item* parentItem() const { return parent_; }

    /**
       @return When the item is embeded one,
       this function returs the first parent item which is not an embeded one.
       Otherwise the item itself is returned.
    */
    Item* headItem() const;

    RootItem* findRootItem() const;
    bool isConnectedToRoot() const;
    Item* localRootItem() const;

    bool addChildItem(Item* item, bool isManualOperation = false);

    [[deprecated("Use Item::insertChild(Item* position, Item* item, bool isManualOperation).")]]
    bool insertChildItem(Item* item, Item* nextItem, bool isManualOperation = false);

    bool insertChild(Item* position, Item* item, bool isManualOperation = false);

    /**
       This function adds a sub item to the item.
       The sub item is an item that is a required element of the main item,
       and the sub item cannot be removed from the main item.
    */
    bool addSubItem(Item* item);

    [[deprecated("Use Item::setSubItemAttributes and Item::insertChild(Item* position, Item* item, bool isManualOperation).")]]
    bool insertSubItem(Item* item, Item* nextItem);

    void removeFromParentItem();

    [[deprecated("Use Item::removeFromParentItem.")]]
    void detachFromParentItem() { removeFromParentItem(); }
    
    void clearChildren();

    typedef std::function<bool(Item* item)> ItemPredicate;

    template<class ItemType>
    static ItemPredicate getItemPredicate() {
        return [](Item* item) -> bool { return dynamic_cast<ItemType*>(item); };
    }
    template<class ItemType>
    static ItemPredicate getItemPredicate(std::function<bool(ItemType* item)> pred) {
        return [pred](Item* item){
            if(auto casted = dynamic_cast<ItemType*>(item)){
                return pred ? pred(casted) : true;
            }
            return false;
        };
    }

    /**
       This is equivalent to RootItem::instance()->findItem(path);
    */
    static Item* find(const std::string& path) {
        return find(path, nullptr);
    }
    template<class ItemType>
    ItemType* find(const std::string& path = "") {
        return static_cast<ItemType*>(find(path, getItemPredicate<ItemType>()));
    }
    
    /**
       Find an item that has the corresponding path to it in the sub tree
    */
    Item* findItem(const std::string& path) const {
        return findItem(path, nullptr, true);
    }
    template<class ItemType>
    ItemType* findItem(const std::string& path) const {
        return static_cast<ItemType*>(
            findItem(path, getItemPredicate<ItemType>(), true));
    }
    template<class ItemType>
    ItemType* findItem(std::function<bool(ItemType* item)> pred = nullptr) const {
        return static_cast<ItemType*>(
            findItem("", getItemPredicate<ItemType>(pred), true));
    }
    
    /**
       Find an item that has the corresponding path from a child item to it
    */
    Item* findChildItem(const std::string& path, std::function<bool(Item* item)> pred = nullptr) const {
        return findItem(path, pred, false);
    }
    template<class ItemType>
    ItemType* findChildItem(const std::string& path, std::function<bool(ItemType* item)> pred = nullptr) const {
        return static_cast<ItemType*>(
            findItem(path, getItemPredicate<ItemType>(pred), false));
    }
    template<class ItemType>
    ItemType* findChildItem(std::function<bool(ItemType* item)> pred = nullptr) const {
        return static_cast<ItemType*>(
            findItem("", getItemPredicate<ItemType>(pred), false));
    }

    /**
       Find a sub item that has the corresponding path from a direct sub item to it
    */
    [[deprecated("Use Item::findChildItem with the pred function")]]
    Item* findSubItem(const std::string& path) const {
        return findItem(path, [](Item* item){ return item->isSubItem(); }, false);
    }

    template<class ItemType>
    [[deprecated("Use Item::findChildItem with the pred function")]]
    ItemType* findSubItem(const std::string& path = "") const {
        return static_cast<ItemType*>(
            findItem(
                path,
                getItemPredicate<ItemType>([](ItemType* item){ return item->isSubItem(); }),
                false));
    }
    
    template <class ItemType> ItemType* findOwnerItem(bool includeSelf = false) const {
        Item* parentItem__ = includeSelf ? const_cast<Item*>(this) : parentItem();
        while(parentItem__){
            if(ItemType* ownerItem = dynamic_cast<ItemType*>(parentItem__)){
                return ownerItem;
            }
            parentItem__ = parentItem__->parentItem();
        }
        return nullptr;
    }

    bool isOwnedBy(Item* item) const;

    ItemList<> childItems(std::function<bool(Item* item)> pred = nullptr) const;

    template <class ItemType>
    ItemList<ItemType> childItems(std::function<bool(ItemType* item)> pred = nullptr) const {
        return getDescendantItems(getItemPredicate<ItemType>(pred), false);
    }

    ItemList<> descendantItems(std::function<bool(Item* item)> pred = nullptr) const;

    template <class ItemType>
    ItemList<ItemType> descendantItems(std::function<bool(ItemType* item)> pred = nullptr) const {
        return getDescendantItems(getItemPredicate<ItemType>(pred), true);
    }

    ItemList<> selectedDescendantItems(std::function<bool(Item* item)> pred = nullptr) const;

    template <class ItemType>
    ItemList<ItemType> selectedDescendantItems(std::function<bool(Item* item)> pred = nullptr) const {
        return selectedDescendantItems(getItemPredicate<ItemType>(pred));
    }

    bool traverse(std::function<bool(Item*)> pred);

    template<class ItemType>
    bool traverse(std::function<bool(ItemType* item)> pred = nullptr){
        return Item::traverse(getItemPredicate<ItemType>(pred));
    }

    /**
       This signal is emitted when the position of this item in the item tree is changed.
       Being added to the tree and being removed from the tree are also the events
       to emit this signal.
       This signal is also emitted for descendent items when the position of an ancestor
       item is changed.
       This signal is emitted before RootItem::sigTreeChanged();
    */
    SignalProxy<void()> sigPositionChanged();

    /**
       The following signal name is temporary.
       This is a variant of sigPositionChanged, and should be a standard sigunature.
       When the template implementation of the Signal class is improved to connect to functions
       omitting some arguments defined in a signal, this sigunare should replace the old
       sigunature with no arguments.
       Do not use this signal if you don't have a special reason because this function name
       will be removed.
    */
    SignalProxy<void(Item* topItem, Item* prevTopParentItem)> sigPositionChanged2();
    
    SignalProxy<void()> sigSubTreeChanged();

    SignalProxy<void()> sigDisconnectedFromRoot();

    [[deprecated("Use Item::sigDisconnectedFromRoot.")]]
    SignalProxy<void()> sigDetachedFromRoot() { return sigDisconnectedFromRoot(); }

    SignalProxy<void(bool on)> sigSelectionChanged();
    SignalProxy<void(int checkId, bool on)> sigAnyCheckToggled();
    SignalProxy<void(bool on)> sigCheckToggled(int checkId = PrimaryCheck);

    virtual void notifyUpdate();
    SignalProxy<void()> sigUpdated();

    bool setAddon(ItemAddon* addon);
    void removeAddon(ItemAddon* addon);
    template<class AddonType> AddonType* findAddon(){
        return static_cast<AddonType*>(findAddon_(typeid(AddonType)));
    }
    template<class AddonType> AddonType* getAddon(){
        return static_cast<AddonType*>(getAddon_(typeid(AddonType)));
    }
    std::vector<ItemAddon*> addons();

    /**
       This function loads the data of the item from a file by using a pre-registered loading function.
   
       To make this function available, a loading function has to be registered to an ItemManager
       in advance by calling the addLoader() or addLoaderAndSaver() function.  Otherwise,
       this function cannot be used.
       Note that this function should not be overloaded or overridden in the derived classes.
    */
    bool load(
        const std::string& filename, const std::string& format = std::string(),
        const Mapping* options = nullptr);

    /**
       @param parentItem specify this when the item is newly created one and will be attached to a parent item
       if loading succeeds.
    */
    bool load(
        const std::string& filename, Item* parent, const std::string& format = std::string(),
        const Mapping* options = nullptr);
    
    /**
       This function saves the data of the item to a file by using a pre-registered saving function.
   
       To make this function available, a saving function has to be registered to an ItemManager
       in advance by calling the addSaver() or addLoaderAndSaver() function.  Otherwise,
       this function cannot be used.
       Note that this function should not be overloaded or overridden in the derived classes.
    */
    bool save(const std::string& filename, const std::string& format = std::string(),
              const Mapping* options = nullptr);

    /**
       This function save the data of the item to the file from which the data of the item has been loaded.
       If the data has not been loaded from a file, a file save dialog opens and user specifies a file.
    */
    bool overwrite(bool forceOverwrite = false, const std::string& format = std::string());

    // full path file name
    const std::string& filePath() const;
    // file name without the directory
    std::string fileName() const;
    const std::string& fileFormat() const;
    const Mapping* fileOptions() const;
    std::time_t fileModificationTime() const;
    bool isConsistentWithFile() const;

    void updateFileInformation(const std::string& filename, const std::string& format, Mapping* options = nullptr);
    void setConsistentWithFile(bool isConsistent);
    void suggestFileUpdate();

    //! Use this function to disable the implicit overwrite next time
    void clearFileInformation();

#ifdef CNOID_BACKWARD_COMPATIBILITY
    const std::string& lastAccessedFilePath() const;
    const std::string& lastAccessedFileFormatId() const;
#endif

    void putProperties(PutPropertyFunction& putProperty);

    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

protected:

    //! Implement the code to copy properties like the assingment operator
    virtual void doAssign(Item* srcItem);

    //! Override this function to allow duplication of an instance.
    virtual Item* doDuplicate() const;

    virtual bool onCheckNewPosition(bool isManualOperation);
    virtual void onAddedToParent();

    /**
       This function is called when the item has been connected to the tree including the root item.
       The onPositionChanged function and sigSubTreeChanged are processed before calling this function.
    */
    virtual void onConnectedToRoot();

    /**
       This function is called when the item position in the whole item tree is changed.
       Note that this function is not called when the item is not connected to the root
       even if the item is newly attached to another item that is not connected to the root.
       For such items, the function is called for each item when the sub tree is connected
       to the root.
    */
    virtual void onPositionChanged();

    virtual void onRemovedFromParent(Item* parentItem);
    virtual void onDisconnectedFromRoot();

    /**
       This function is called when a child item is about to added to this item.
       \return false if the item cannot be accepted as a child item
       \note The childItem is not actually connected to the item when this function is called.
    */
    virtual bool onChildItemAboutToBeAdded(Item* childItem, bool isManualOperation);

    /**
       Override this function to put properties of the item.
       @note Please call doPutProperties() of the parent class in this function.
       For example, when your class directly inherits the Item class,
       call Item::doPutProperties(putProperty).
    */
    virtual void doPutProperties(PutPropertyFunction& putProperty);

private:
    class Impl;
    Impl* impl;

    mutable int classId_;
    Item* parent_;
    ItemPtr firstChild_;
    ItemPtr nextItem_;
    Item* prevItem_;
    Item* lastChild_;
    int numChildren_;
    std::string name_;
    bool isSelected_;

    static Item* find(const std::string& path, const std::function<bool(Item* item)>& pred);
    Item* findItem(
        const std::string& path, std::function<bool(Item* item)> pred, bool isRecursive) const;
    ItemList<Item> getDescendantItems(std::function<bool(Item* item)> pred, bool isRecursive) const;
    void validateClassId() const;
    ItemAddon* findAddon_(const std::type_info& type);
    ItemAddon* getAddon_(const std::type_info& type);
};

#ifndef CNOID_BASE_MVOUT_DECLARED
#define CNOID_BASE_MVOUT_DECLARED
CNOID_EXPORT std::ostream& mvout(bool doFlush = false);
#endif

}

#endif
