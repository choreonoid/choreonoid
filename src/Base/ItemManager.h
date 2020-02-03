/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_ITEM_MANAGER_H
#define CNOID_BASE_ITEM_MANAGER_H

#include "ExtensionManager.h"
#include "ItemList.h"
#include <QWidget>
#include <string>
#include <typeinfo>
#include <iosfwd>
#include <memory>
#include "exportdecl.h"

namespace cnoid {

class MenuManager;

class Item;
typedef ref_ptr<Item> ItemPtr;

class ItemManagerImpl;
class ItemFileIO;
class Mapping;

class ItemCreationPanel : public QWidget
{
public:
    ItemCreationPanel(QWidget* parent = nullptr) : QWidget(parent) { }

    virtual bool initializePanel(Item* protoItem) = 0;
    virtual bool initializeItem(Item* protoItem) = 0;

protected:
    ItemCreationPanel* findPanelOnTheSameDialog(const std::string& name);
};


class CNOID_EXPORT ItemManager
{
public:
    ItemManager(const std::string& moduleName, MenuManager& menuManager);
    ~ItemManager();

    void detachAllManagedTypeItemsFromRoot();

private:
    template <class ItemType> class Factory {
    public:
        virtual Item* operator()() { return new ItemType(); }
    };

    class CreationPanelFilterBase
    {
    public:
        virtual ~CreationPanelFilterBase() { }
        virtual bool operator()(Item* protoItem, Item* parentItem) = 0;
    };

    class OverwritingCheckFunctionBase
    {
    public:
        ~OverwritingCheckFunctionBase();
        virtual bool operator()(Item* item) = 0;
    };

public:
    class FileFunctionBase
    {
    public:
        virtual ~FileFunctionBase() { }
        virtual bool operator()(Item* item, const std::string& filename, std::ostream& os, Item* parentItem) = 0;
    };

    void bindTextDomain(const std::string& domain);

    enum { PRIORITY_CONVERSION = -10, PRIORITY_COMPATIBILITY = 0, PRIORITY_OPTIONAL = 0, PRIORITY_DEFAULT = 10 };

    template <class ItemType> class CreationPanelFilter : public CreationPanelFilterBase
    {
    public:
        typedef std::function<bool(ItemType* protoItem, Item* parentItem)> Function;
        CreationPanelFilter(Function function) : function(function) { }
        virtual bool operator()(Item* protoItem, Item* parentItem){
            return function(static_cast<ItemType*>(protoItem), parentItem);
        }
    private:
        Function function;
    };
        
    template <class ItemType> class FileFunction : public FileFunctionBase
    {
    public:
        typedef std::function<bool(ItemType* item, const std::string& filename, std::ostream& os, Item* parentItem)> Function;
        FileFunction(Function function) : function(function) { }
        virtual bool operator()(Item* item, const std::string& filename, std::ostream& os, Item* parentItem){
            return function(static_cast<ItemType*>(item), filename, os, parentItem);
        }
    private:
        Function function;
    };

    template <class ItemType, class SuperItemType = Item>
    ItemManager& registerClass(const std::string& className) {
        registerClassSub(className, typeid(ItemType), typeid(SuperItemType), Factory<ItemType>(), nullptr);
        return *this;
    }

    //! This function registers a singleton item class
    template <class ItemType, class SuperItemType = Item>
    ItemManager& registerClass(const std::string& className, ItemType* singletonInstance){
        registerClassSub(className, typeid(ItemType), typeid(SuperItemType), nullptr, singletonInstance);
        return *this;
    }

    template <class ItemType, class SuperItemType = Item>
    ItemManager& registerAbstractClass() {
        registerClassSub("", typeid(ItemType), typeid(SuperItemType), nullptr, nullptr);
        return *this;
    }

    static bool getClassIdentifier(ItemPtr item, std::string& out_moduleName, std::string& out_className);

    template <class ItemType> static ItemType* singletonInstance() {
        return static_cast<ItemType*>(getSingletonInstance(typeid(ItemType).name()));
    }

    static Item* singletonInstance(ItemFileIO* fileIO);

    template <class ItemType> ItemManager& addCreationPanel(ItemCreationPanel* panel = 0) {
        addCreationPanelSub(typeid(ItemType).name(), panel);
        return *this;
    }

    template <class ItemType>
    void addCreationPanelPreFilter(const typename CreationPanelFilter<ItemType>::Function& filter) {
        addCreationPanelFilterSub(
            typeid(ItemType).name(),
            std::make_shared<CreationPanelFilter<ItemType>>(filter),
            false);
    }
        
    template <class ItemType>
    void addCreationPanelPostFilter(const typename CreationPanelFilter<ItemType>::Function& filter){
        addCreationPanelFilterSub(
            typeid(ItemType).name(),
            std::make_shared<CreationPanelFilter<ItemType>>(filter),
            true);
    }

    template <class ItemType>
    ItemManager& registerFileIO(ItemFileIO* fileIO) {
        registerFileIO_(typeid(ItemType), fileIO);
        return *this;
    }

    static ItemFileIO* findFileIO(const std::type_info& type, const std::string& formatId);

    template <class ItemType>
    ItemManager& addLoader(
        const std::string& caption, const std::string& formatId, const std::string& extensions, 
        const typename FileFunction<ItemType>::Function& function, int priority = PRIORITY_DEFAULT) {
        addLoaderSub(typeid(ItemType), caption, formatId, [extensions](){ return extensions; },
                     std::make_shared<FileFunction<ItemType>>(function), priority);
        return *this;
    }

    template <class ItemType>
    ItemManager& addLoader(
        const std::string& caption, const std::string& formatId, std::function<std::string()> getExtensions,
        const typename FileFunction<ItemType>::Function& function, int priority = PRIORITY_DEFAULT) {
        addLoaderSub(typeid(ItemType), caption, formatId, getExtensions,
                     std::make_shared<FileFunction<ItemType>>(function), priority);
        return *this;
    }
    
    template<class ItemType>
    ItemManager& addSaver(
        const std::string& caption, const std::string& formatId, const std::string& extensions,
        const typename FileFunction<ItemType>::Function& function, int priority = PRIORITY_DEFAULT){
        addSaverSub(typeid(ItemType), caption, formatId, [extensions](){ return extensions; },
                    std::make_shared<FileFunction<ItemType>>(function), priority);
        return *this;
    }

    template<class ItemType>
    ItemManager& addSaver(
        const std::string& caption, const std::string& formatId, std::function<std::string()> getExtensions,
        const typename FileFunction<ItemType>::Function& function, int priority = PRIORITY_DEFAULT){
        addSaverSub(typeid(ItemType), caption, formatId, getExtensions, 
                    std::make_shared<FileFunction<ItemType>>(function), priority);
        return *this;
    }
    
    template<class ItemType>
    ItemManager& addLoaderAndSaver(const std::string& caption, const std::string& formatId,
                                   const std::string& extensions,
                                   const typename FileFunction<ItemType>::Function& loadingFunction,
                                   const typename FileFunction<ItemType>::Function& savingFunction,
                                   int priority = PRIORITY_DEFAULT){
        addLoader<ItemType>(caption, formatId, extensions, loadingFunction, priority);
        addSaver<ItemType>(caption, formatId, extensions, savingFunction, priority);
        return *this;
    }

    template<class ItemType>
    ItemManager& addLoaderAndSaver(const std::string& caption, const std::string& formatId,
                                   std::function<std::string()> getExtensions,
                                   const typename FileFunction<ItemType>::Function& loadingFunction,
                                   const typename FileFunction<ItemType>::Function& savingFunction,
                                   int priority = PRIORITY_DEFAULT){
        addLoader<ItemType>(caption, formatId, getExtensions, loadingFunction, priority);
        addSaver<ItemType>(caption, formatId, getExtensions, savingFunction, priority);
        return *this;
    }
    
    void addMenuItemToImport(const std::string& caption, std::function<void()> slot);

    static Item* createItem(const std::string& moduleName, const std::string& itemClassName);

    /**
       Create a new item interactively using the corresponding dialog
       @param parentItem The item that will be a parent of the new item.
       @doAddition The created item is actually added to the parent item if this flag is true.
       @nextItem The position to insert the created item can be specifid by this parameter.
    */
    template <class ItemType>
    static ItemType* createItemWithDialog(Item* parentItem, bool doAddition = true, Item* nextItem = nullptr){
        return static_cast<ItemType*>(createItemWithDialog_(typeid(ItemType), parentItem, doAddition, nextItem));
    }

    template <class ItemType>
    static ItemList<ItemType> loadItemsWithDialog(
        Item* parentItem, bool doAddtion = true, Item* nextItem = nullptr){
        return loadItemsWithDialog_(typeid(ItemType), parentItem, doAddtion, nextItem);
    }

    static void reloadItems(const ItemList<>& items);
    static Item* findOriginalItemForReloadedItem(Item* item);

private:
    void registerClassSub(
        const std::string& className, const std::type_info& type, const std::type_info& superType,
        std::function<Item*()> factory, Item* singletonInstance);
    void addCreationPanelSub(const std::string& typeId, ItemCreationPanel* panel);
    void addCreationPanelFilterSub(
        const std::string& typeId, std::shared_ptr<CreationPanelFilterBase> filter, bool afterInitializionByPanels);
    void registerFileIO_(const std::type_info& type, ItemFileIO* fileIO);
    void addLoaderSub(
        const std::type_info& type, const std::string& caption, const std::string& formatId,
        std::function<std::string()> getExtensions, std::shared_ptr<FileFunctionBase> function, int priority);
    void addSaverSub(
        const std::type_info& type, const std::string& caption, const std::string& formatId,
        std::function<std::string()> getExtensions, std::shared_ptr<FileFunctionBase> function, int priority);

    static Item* getSingletonInstance(const std::string& typeId);

    static Item* createItemWithDialog_(const std::type_info& type, Item* parentItem, bool doAddition, Item* nextItem);
    static ItemList<Item> loadItemsWithDialog_(
        const std::type_info& type, Item* parentItem, bool doAddtion, Item* nextItem);

    // The following static functions are called from functions in the Item class
    static bool load(
        Item* item, const std::string& filename, Item* parentItem, const std::string& formatId,
        const Mapping* options = nullptr);
    static bool save(Item* item, const std::string& filename, const std::string& formatId);
    static bool overwrite(Item* item, bool forceOverwrite, const std::string& formatId); // overwrite

    friend class Item;
    friend class ItemManagerImpl;
    ItemManagerImpl* impl;
};

CNOID_EXPORT std::string getOpenFileName(const std::string& caption, const std::string& extensions);
CNOID_EXPORT std::vector<std::string> getOpenFileNames(const std::string& caption, const std::string& extensions);

}

#endif
