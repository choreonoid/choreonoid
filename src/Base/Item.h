#ifndef CNOID_BASE_ITEM_H
#define CNOID_BASE_ITEM_H

#include <cnoid/ClonableReferenced>
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
class EditRecord;
class ItemTreeWidget;
class MessageOut;

class CNOID_EXPORT Item : public ClonableReferenced
{
protected:
    Item();
    Item(const std::string& name);
    Item(const Item& item);
    
public:
    enum Attribute {
        /**
           This attribute is set if the role of the item is specified by the system and it works
           with other items at a specific position in the item tree.
           If the item has this attribute, changing the name or position of the item, or removing the item,
           cannot be done interactively by a user on the item tree view.
        */
        Attached = 1 << 0,

        SubItemAdditionalAttribute = 1 << 1,
        
        /**
           This attribute is set if the item is a part of a composite item and is not the main (top) item of it.
           The attribute first has the same attributes as Attached.
           In addition to that, the state of the item is stored or restored as a part of the top item of the
           corresponding composite item when the project save or load is performed.
           Note that items with this attribute are excluded from the built-in undo / redo processing implemented
           in ItemEditRecordManager as well as the ExcludedFromUnifiedEditHistory attribute.
           Add the IncludedInUnifiedEditHistory attribute to enable the built-in undo / redo processing.
        */
        SubItem = Attached | SubItemAdditionalAttribute,

        /**
           This attribute is set if the item is unique in a specific situation.
           This attribute is first set to a singleton item instance, and may be set to non-singleton
           item instance as well if the item is used in the above situation.
           As a specific effect of this attribute, the operation of copying an instance is forbidden
           to a user on the item tree view.
        */
        Unique = 1 << 2,

        /**
           This attribute is set if the item is temporarily generated item.
           An item with this attribute is not stored or restored when a project is saved or loaded.
        */
        Temporary = 1 << 3,

        /**
           This attribute is set if the item is a built-in type item, which is automatically created and
           managed by a Choreonoid application, If there are no items other than built-in items, the
           project is recognized as empty even though there are such built-in items.
         */
        Builtin = 1 << 4,

        /**
           This attribute is set if the item is basically loaded from a file and the file does not
           need to be modified by user operations on the item.
           In that case, an item inherits the information on the original file when it is duplicated
           from an existing item, and the information is stored as item data without confirmation
           when the project is saved.
        */
        FileImmutable = 1 << 5,
        
        /**
           This attribute is set to enable the reloading function for this item.
        */
        Reloadable = 1 << 6,

        /**
           Items with this attribute are excluded from the the built-in undo / redo processing implemented
           in ItemEditRecordManager. Add this attribute to avoid the built-in processing or to implement
           custom processing for undo / redo operations.
        */
        ExcludedFromUnifiedEditHistory = 1 << 7,

        /**
           Add this attribute to enable the built-in undo / redo processing for items with the SubItem attribute.
           Note that the built-in processing are enabled by default for items without the SubItem attribute.
           Adding this attribute is only valid for the item with the SubItem attribute.
        */
        IncludedInUnifiedEditHistory = 1 << 8,
        
        // deprecated
        SUB_ITEM = SubItem,
        Temporal = Temporary,
        TEMPORAL = Temporal,
        LOAD_ONLY = FileImmutable,
        LocaOnly = FileImmutable
    };

    virtual ~Item();

    Item& operator=(const Item& rhs) = delete;

    int classId() const {
        if(classId_ < 0) validateClassId();
        return classId_;
    }
    int superClassId() const;

    Item* createNewInstance() const;

    //! Copy item properties as much as possible like the assignment operator
    bool assign(const Item* srcItem);

    Item* clone() const { return static_cast<Item*>(doClone(nullptr)); }
    Item* clone(CloneMap& cloneMap) const { return static_cast<Item*>(doClone(&cloneMap)); }

    //! This function creates a copy of the item including its descendant items
    Item* cloneSubTree(CloneMap& cloneMap) const;

    [[deprecated("Use the clone function.")]]
    Item* duplicate() const { return clone(); }

    [[deprecated("Use the cloneSubTree function.")]]
    Item* duplicateSubTree() const;

    [[deprecated("Use the cloneSubTree function.")]]
    Item* duplicateAll() const;

    const std::string& name() const { return name_; }
    //! \return true if the name is successfully updated or the item originally has the same name
    virtual bool setName(const std::string& name);
    virtual std::string displayName() const;
    void setDisplayNameModifier(std::function<std::string(const Item* item)> modifier);
    bool hasDisplayNameModifier() const;
    SignalProxy<void(const std::string& oldName)> sigNameChanged();
    //! This function notifies the system of a displayName change
    void notifyNameChange();

    void setAttribute(Attribute attribute) {
        attributes_ |= attribute;
    }
    void setAttributes(int attributes) {
        attributes_ |= attributes;
    }
    void unsetAttribute(Attribute attribute) {
        attributes_ &= ~attribute;
    }
    bool hasAttribute(Attribute attribute) const {
        return (attributes_ & attribute) == attribute;
    }
    bool isSubItem() const {
        return hasAttribute(SubItem);
    }

    [[deprecated("Use setAttribute(Item::SubItem)")]]
    void setSubItemAttributes(){
        setAttribute(Item::SubItem);
    }

    /**
       If this is true, the item is not automatically saved or overwritten
       when a project is saved. For example, a motion item which is produced as a
       simulation result may be an temporary item because a user may not want to
       save the result. If a user manually save the item, the item becomes a
       non-temporary item. Or if a child item is manually attached to a temporary
       item, the item becomes non-temporary one as well.
    */
    bool isTemporary() const {
        return hasAttribute(Temporary);
    }
    void setTemporary(bool on = true);

    [[deprecated("Use isTemporary.")]]
    bool isTemporal() const { return isTemporary(); }
    [[deprecated("Use setTemporary.")]]
    void setTemporal(bool on = true) { setTemporary(on); }

    SignalProxy<void()> sigAttributeChanged();
    void notifyAttributeChange();

    bool isSelected() const { return isSelected_; }
    void setSelected(bool on = true, bool isCurrent = false);
    void setSubTreeItemsSelected(bool on);

    /**
       \note LogicalSumOfAllChecks is only valid for "isChecked(LogicalSumOfAllChecks)" and
       the signal returned by "sigCheckToggled(LogicalSumOfAllChecks)".
    */
    enum CheckId { LogicalSumOfAllChecks = -1, PrimaryCheck = 0 };
    
    bool isChecked(int checkId = PrimaryCheck) const;
    void setChecked(bool on = true); // for PrimaryCheck
    void setChecked(int checkId, bool on = true);
    int numCheckStates() const;

    int numChildren() const { return numChildren_; }

    int countDescendantItems() const;
    template<class ItemType>
    int countDescendantItems(){
        return countDescendantItems_(getItemPredicate<ItemType>());
    }

    Item* childItem() const { return firstChild_; }
    Item* prevItem() const { return prevItem_; }
    Item* nextItem() const { return nextItem_; }
    Item* lastChildItem() const { return lastChild_; }
    Item* parentItem() const { return parent_; }

    template<class ItemType>
    ItemType* parentItem() const {
        return dynamic_cast<ItemType*>(parent_);
    }

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
    void clearNonSubItemChildren();

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
       This is equivalent to RootItem::instance<ItemType>()->findItem();
    */
    template<class ItemType>
    static ItemType* find() {
        return static_cast<ItemType*>(find(getItemPredicate<ItemType>()));
    }
    /**
       This is equivalent to RootItem::instance()->findItem(path);
    */
    static Item* find(const std::string& path) {
        return find(path, nullptr);
    }
    /**
       This is equivalent to RootItem::instance<ItemType>()->findItem(path);
    */
    template<class ItemType>
    static ItemType* find(const std::string& path) {
        return static_cast<ItemType*>(find(path, getItemPredicate<ItemType>()));
    }
    
    template<class ItemType>
    ItemType* findItem(std::function<bool(ItemType* item)> pred = nullptr) const {
        return static_cast<ItemType*>(
            findItem(getItemPredicate<ItemType>(pred), true));
    }

    /**
       Find an item that has the corresponding path to it in the sub tree
    */
    Item* findItem(const std::string& path) const {
        return findItem(path, nullptr, true);
    }
    
    /**
       Find an item that has the corresponding path to it in the sub tree
    */
    template<class ItemType>
    ItemType* findItem(const std::string& path) const {
        return static_cast<ItemType*>(findItem(path, getItemPredicate<ItemType>(), true));
    }
    
    template<class ItemType>
    ItemType* findChildItem(std::function<bool(ItemType* item)> pred = nullptr) const {
        return static_cast<ItemType*>(findItem(getItemPredicate<ItemType>(pred), false));
    }

    /**
       Find an item that has the corresponding path from a child item to it
    */
    Item* findChildItem(const std::string& path, std::function<bool(Item* item)> pred = nullptr) const {
        return findItem(path, pred, false);
    }
    
    /**
       Find an item that has the corresponding path from a child item to it
    */
    template<class ItemType>
    ItemType* findChildItem(const std::string& path, std::function<bool(ItemType* item)> pred = nullptr) const {
        return static_cast<ItemType*>(
            findItem(path, getItemPredicate<ItemType>(pred), false));
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

    /**
       \param callback Return true by this functor to continue the traverse,
       or return false to stop the traverse more deeply.
    */
    void traverse(std::function<bool(Item*)> callback, bool includeSelf = true);

    /**
       This signal is emitted when the path from the root to this item in an item tree is changed.
       The path changes include the addition or removal of this item to or from the item tree.
       The item tree can contain or not contain the RootItem instance as the tree root.
       Note that the signal is emitted before RootItem::sigTreeChanged().
    */
    SignalProxy<void()> sigTreePathChanged();

    /**
       This signal is emitted when the position of this item in the item tree is changed.
       In contrast to sigTreePathChanged, this signal is emitted when the order of the item
       in the parent item is changed even if the path to the item is not changed.
       Except for that, this signal is same as sigTreePathChanged.
    */
    SignalProxy<void()> sigTreePositionChanged();

    /**
       The arguments of this signal should be included in sigTreePositionChanged.
       However, the current implementation of the signal template class does now allow the omission of
       arguments in connecting a slot function and these arguments are not frequently used.
       Therefore the signal with the arguments are defined separately from the signal without arguments.
       If the Signal template class is improved so that it can omit arguments in the slot connection,
       it is better to integrate the signals.
       In any case, do not use this signal if you don't have a special reason.
    */
    SignalProxy<void(Item* topItem, Item* prevTopParentItem)> sigTreePositionChanged2();

    [[deprecated("Use sigTreePositionChanged")]]
    SignalProxy<void()> sigPositionChanged();

    /**
       This signal is emitted when the structure or elements of the sub tree changes.
    */
    SignalProxy<void()> sigSubTreeChanged();

    SignalProxy<void()> sigDisconnectedFromRoot();

    [[deprecated("Use Item::sigDisconnectedFromRoot.")]]
    SignalProxy<void()> sigDetachedFromRoot() { return sigDisconnectedFromRoot(); }

    SignalProxy<void(bool on)> sigSelectionChanged();
    SignalProxy<void(int checkId, bool on)> sigAnyCheckToggled();
    SignalProxy<void(bool on)> sigCheckToggled(int checkId = PrimaryCheck);

    virtual void notifyUpdate();
    void notifyUpdateWithProjectFileConsistency();
    SignalProxy<void()> sigUpdated();

    bool setAddon(ItemAddon* addon);
    ItemAddon* findAddon(const std::type_info& type);
    const ItemAddon* findAddon(const std::type_info& type) const;
    template<class AddonType> AddonType* findAddon(){
        return static_cast<AddonType*>(findAddon(typeid(AddonType)));
    }
    template<class AddonType> const AddonType* findAddon() const {
        return static_cast<AddonType*>(findAddon(typeid(AddonType)));
    }
    ItemAddon* getAddon(const std::type_info& type);
    template<class AddonType> AddonType* getAddon(){
        return static_cast<AddonType*>(getAddon(typeid(AddonType)));
    }
    const ItemAddon* getAddon(const std::type_info& type) const;
    template<class AddonType> const AddonType* getAddon() const {
        return static_cast<const AddonType*>(getAddon(typeid(AddonType)));
    }
    void removeAddon(ItemAddon* addon);
    std::vector<ItemAddon*> addons();
    std::vector<const ItemAddon*> addons() const;

    class ContinuousUpdateRef : public Referenced
    {
    private:
        ContinuousUpdateRef(Item* item);
        ~ContinuousUpdateRef();
        weak_ref_ptr<Item> itemRef;
        friend class Item;
    };
    typedef ref_ptr<ContinuousUpdateRef> ContinuousUpdateEntry;

    ContinuousUpdateEntry startContinuousUpdate();
    bool isContinuousUpdateState() const { return continuousUpdateCounter > 0; }
    bool isContinuousUpdateStateSubTree() const;
    
    /**
       \note The sigUpdated signal is not emitted when the continuous update state is changed
       becasue the state is not a permenent one.
    */
    SignalProxy<void(bool on)> sigContinuousUpdateStateChanged();

    /**
       This function loads the data of the item from a file by using a registered FileIO object.
       To make this function available, a FileIO object must be registered to an ItemManager
       in advance with its registerFileIO function.
    */
    bool load(
        const std::string& filename, const std::string& format = std::string(),
        const Mapping* options = nullptr, MessageOut* mout = nullptr);

    /**
       An overload version of the load function.
       @param parent specify this when the item is newly created one and will be attached to a parent item
       if loading succeeds.
    */
    bool load(
        const std::string& filename, Item* parent, const std::string& format = std::string(),
        const Mapping* options = nullptr, MessageOut* mout = nullptr);

    bool isFileSavable() const;    
    
    /**
       This function saves the data of the item to a file by using a registered FileIO object.
       To make this function available, a FileIO object must be registered to an ItemManager
       in advance with its registerFileIO function.
    */
    bool save(
        const std::string& filename, const std::string& format = std::string(), const Mapping* options = nullptr,
        MessageOut* mout = nullptr);

    bool saveWithFileDialog();

    /**
       This function tries to overwrite the external data file of the item.
       If the data has not been loaded from a file, the functions do nothing and returns false.
    */
    bool overwrite(
        bool forceOverwrite = false, const std::string& format = std::string(), time_t cutoffTime = 0,
        MessageOut* mout = nullptr);

    /**
       This function tries to save the data of the item to the file from which the data of the item has
       been loaded. If the data has not been loaded from a file, a file save dialog for the item is shown.
    */
    bool overwriteOrSaveWithDialog(bool forceOverwrite = false, const std::string& format = std::string());
    
    //! Full path file name
    const std::string& filePath() const;
    //! File name without the directory
    std::string fileName() const;
    const std::string& fileFormat() const;
    Mapping* fileOptions();
    const Mapping* fileOptions() const;
    std::time_t fileModificationTime() const;
    bool isConsistentWithFile() const;
    int fileConsistencyId() const;

    void updateFileInformation(
        const std::string& filename, const std::string& format, Mapping* options = nullptr, bool doNotify = true);
    void setConsistentWithFile(bool isConsistent);
    void suggestFileUpdate();

    //! Use this function to disable the implicit overwrite next time
    void clearFileInformation();

    bool reload();
    bool replace(Item* originalItem);
    Item* findOriginalItem() const;
    Item* findReplacementItem() const;

#ifdef CNOID_BACKWARD_COMPATIBILITY
    const std::string& lastAccessedFilePath() const;
    const std::string& lastAccessedFileFormatId() const;
#endif

    void putProperties(PutPropertyFunction& putProperty);

    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

    void setConsistentWithProjectArchive(bool isConsistent);
    bool isConsistentWithProjectArchive() const;
    
protected:

    //! Implement the code to copy properties like the assingment operator
    virtual bool doAssign(const Item* srcItem);

    virtual Referenced* doClone(CloneMap* cloneMap) const override final;

    virtual Item* doCloneItem(CloneMap* cloneMap) const;

    //! Deprecated. Override doCloneItem instead of this function.
    virtual Item* doDuplicate() const;

    virtual bool onNewTreePositionCheck(bool isManualOperation, std::function<void()>& out_callbackWhenAdded);
    virtual void onAddedToParent();

    /**
       This function is called at the same time as sigTreePathChanged.
       The order in which it is called precedes the signal.
       By overriding this function in a derived item class, you can implement the necessary processing
       when the path in the item tree is changed.
    */
    virtual void onTreePathChanged();

    /**
       This function is similar to the onTreePathChanged function. The condition to call this function
       is same as sigTreePositionChanged, and the order in which it is called precedes the signal.
    */
    virtual void onTreePositionChanged();

    /**
       \deprecated This function was replaced with onTreePositionChanged and onTreePathChanged is now available.
       Please use either of those functions to implement the necessary processing.
    */
    virtual void onPositionChanged();

    /**
       This function is called when the item has been connected to the item tree with RootItem as the root.
       Note that this function is called before onTreePathChanged and onTreePositionChanged are called.
    */
    virtual void onConnectedToRoot();

    virtual void onRemovedFromParent(Item* parentItem, bool isParentBeingDeleted);
    virtual void onDisconnectedFromRoot();

    /**
       This function is called when a child item is about to be added to this item.
       By overriding this function, you can check a child item and prevent the item from being added
       if necessary.
       \return false if the item cannot be accepted as a child item
       \note The childItem is not actually connected to the item when this function is called.
    */
    virtual bool onChildItemAboutToBeAdded(Item* childItem, bool isManualOperation);

    virtual void onDoubleClicked();

    /**
       This function is used to put a standard properties of the item.
       You can implement the standard properties by overriding this function.
       \note The overridden function should call the same function of the parent class.
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
    unsigned int attributes_;
    int continuousUpdateCounter;
    std::string name_;
    bool isSelected_;

    static Item* find(const std::function<bool(Item* item)>& pred);
    static Item* find(const std::string& path, const std::function<bool(Item* item)>& pred);
    Item* findItem(std::function<bool(Item* item)> pred, bool isRecursive) const;
    Item* findItem(const std::string& path, std::function<bool(Item* item)> pred, bool isRecursive) const;
    int countDescendantItems_(std::function<bool(Item* item)> pred);
    ItemList<Item> getDescendantItems(std::function<bool(Item* item)> pred, bool isRecursive) const;
    void validateClassId() const;

    friend class ItemTreeWidget;
};

#ifndef CNOID_BASE_MVOUT_DECLARED
#define CNOID_BASE_MVOUT_DECLARED
CNOID_EXPORT std::ostream& mvout(bool doFlush = true);
#endif

}

#endif
