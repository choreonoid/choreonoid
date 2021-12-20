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

class Item;
class ItemFileIO;
class ItemAddon;
class Mapping;

class CNOID_EXPORT ItemCreationPanel : public QWidget
{
public:
    ItemCreationPanel();
    virtual bool initializeCreation(Item* protoItem, Item* parentItem) = 0;
    virtual bool updateItem(Item* protoItem, Item* parentItem) = 0;
};

template<class ItemType>
class ItemCreationPanelBase : public ItemCreationPanel
{
protected:
    ItemCreationPanelBase() { }
    virtual bool initializeCreation(ItemType* protoItem, Item* parentItem) = 0;
    virtual bool updateItem(ItemType* protoItem, Item* parentItem) = 0;
    
private:
    virtual bool initializeCreation(Item* protoItem, Item* parentItem) override final {
        return initializeCreation(dynamic_cast<ItemType*>(protoItem), parentItem);
    }
    virtual bool updateItem(Item* protoItem, Item* parentItem) override final {
        return updateItem(dynamic_cast<ItemType*>(protoItem), parentItem);
    }
};

class CNOID_EXPORT DefaultItemCreationPanel : public ItemCreationPanel
{
    QWidget* nameEntry;
public:
    DefaultItemCreationPanel();
    virtual bool initializeCreation(Item* protoItem, Item* parentItem) override;
    virtual bool updateItem(Item* protoItem, Item* parentItem) override;
};

class CNOID_EXPORT ItemManager
{
public:
    static void initializeClass(ExtensionManager* ext);

    ItemManager(const std::string& moduleName);
    ~ItemManager();

    void detachAllManagedTypeItemsFromRoot();

private:
    template <class ItemType> class Factory {
    public:
        virtual Item* operator()() { return new ItemType(); }
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

    enum IoUsageType {
        Standard = 10,
        Conversion = 0,
        Internal = -10,

        // deprecated
        PRIORITY_DEFAULT = Standard,
        PRIORITY_COMPATIBILITY = Internal,
        PRIORITY_OPTIONAL = Internal,
        PRIORITY_CONVERSION = Conversion
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
        registerClass_(className, typeid(ItemType), typeid(SuperItemType), Factory<ItemType>(), nullptr);
        return *this;
    }

    //! This function registers a singleton item class
    template <class ItemType, class SuperItemType = Item>
    ItemManager& registerClass(const std::string& className, ItemType* singletonInstance){
        registerClass_(className, typeid(ItemType), typeid(SuperItemType), nullptr, singletonInstance);
        return *this;
    }

    template <class ItemType, class SuperItemType = Item>
    ItemManager& registerAbstractClass() {
        registerClass_("", typeid(ItemType), typeid(SuperItemType), nullptr, nullptr);
        return *this;
    }

    template <class ItemType>
    ItemManager& addAlias(const std::string& className, const std::string& moduleName){
        addAlias_(typeid(ItemType), className, moduleName);
        return *this;
    }

    static bool getClassIdentifier(Item* item, std::string& out_moduleName, std::string& out_className);

    template <class ItemType> static ItemType* getPrototypeInstance() {
        return static_cast<ItemType*>(getPrototypeInstance_(typeid(ItemType)));
    }

    template <class ItemType> ItemManager& addCreationPanel(ItemCreationPanel* panel = nullptr) {
        addCreationPanel_(typeid(ItemType), panel);
        return *this;
    }

    template <class ItemType>
    ItemManager& addFileIO(ItemFileIO* fileIO) {
        registerFileIO_(typeid(ItemType), fileIO);
        return *this;
    }

    //! \deprecated
    template <class ItemType>
    [[deprecated("Use addFileIO")]]
    ItemManager& registerFileIO(ItemFileIO* fileIO) {
        return addFileIO<ItemType>(fileIO);
    }

    template <class ItemType>
    static std::vector<ItemFileIO*> getFileIOs(){
        return getFileIOs(typeid(ItemType));
    }
    static std::vector<ItemFileIO*> getFileIOs(const std::type_info& type);
    static std::vector<ItemFileIO*> getFileIOs(
        Item* item, std::function<bool(ItemFileIO* fileIO)> pred, bool includeSuperClassIos);
    static ItemFileIO* findFileIO(const std::type_info& type, const std::string& format);

    template <class ItemType>
    ItemManager& addLoader(
        const std::string& caption, const std::string& format, const std::string& extensions, 
        typename FileFunction<ItemType>::Function function, int usage = Standard)
    {
        addLoader_(typeid(ItemType), caption, format, extensions,
                   std::make_shared<FileFunction<ItemType>>(function), usage);
        return *this;
    }

    template<class ItemType>
    ItemManager& addSaver(
        const std::string& caption, const std::string& format, const std::string& extensions,
        typename FileFunction<ItemType>::Function function, int usage = Standard)
    {
        addSaver_(typeid(ItemType), caption, format, extensions,
                  std::make_shared<FileFunction<ItemType>>(function), usage);
        return *this;
    }

    template<class ItemType>
    ItemManager& addLoaderAndSaver(
        const std::string& caption, const std::string& format, const std::string& extensions,
        typename FileFunction<ItemType>::Function loaderFunction,
        typename FileFunction<ItemType>::Function saverFunction,
        int usage = Standard)
    {
        addLoader<ItemType>(caption, format, extensions, loaderFunction, usage);
        addSaver<ItemType>(caption, format, extensions, saverFunction, usage);
        return *this;
    }

    static Item* createItem(const std::string& moduleName, const std::string& itemClassName);
    static Item* createItem(int itemClassId);

    /**
       Create a new item interactively using the corresponding dialog
       \param parentItem The item that will be a parent of the new item.
       \param doAddition The created item is actually added to the parent item if this flag is true.
       \param nextItem The position to insert the created item can be specifid by this parameter.
       \param protoItem The prototype item instance for creating a new item. If this parameter is
       omitted, the default prototype item is used.
       \param title The dialog title string
    */
    template <class ItemType>
    static ItemType* createItemWithDialog(
        Item* parentItem, bool doAddition = true, Item* nextItem = nullptr,
        Item* protoItem = nullptr, const std::string& title = std::string())
    {
        return static_cast<ItemType*>(
            createItemWithDialog_(
                typeid(ItemType), parentItem, doAddition, nextItem, protoItem, title));
    }

    template <class ItemType>
    static ItemList<ItemType> loadItemsWithDialog(
        Item* parentItem, bool doAddtion = true, Item* nextItem = nullptr)
    {
        return loadItemsWithDialog_(typeid(ItemType), parentItem, doAddtion, nextItem);
    }

    template <class ItemType>
    static bool saveItemWithDialog(ItemType* item){
        return saveItemWithDialog_(typeid(ItemType), item);
    }

    [[deprecated("Use Item::reload().")]]
    static void reloadItems(const ItemList<>& items);

    [[deprecated("Use Item::findOriginalItem().")]]
    static Item* findOriginalItemForReloadedItem(Item* item);

    template<class AddonType>
    void registerAddon(const std::string& name){
        registerAddon_(typeid(AddonType), name, [](){ return new AddonType; });
    }
    static ItemAddon* createAddon(const std::type_info& type);
    static ItemAddon* createAddon(const std::string& moduleName, const std::string& addonName);
    static bool getAddonIdentifier(const ItemAddon* addon, std::string& out_moduleName, std::string& out_addonName);

    class Impl;

private:
    void registerClass_(
        const std::string& className, const std::type_info& type, const std::type_info& superType,
        std::function<Item*()> factory, Item* singletonInstance);
    void addAlias_(const std::type_info& type, const std::string& className, const std::string& moduleName);
    void addCreationPanel_(const std::type_info& type, ItemCreationPanel* panel);
    static Item* getPrototypeInstance_(const std::type_info& type);

    void registerFileIO_(const std::type_info& type, ItemFileIO* fileIO);

    void addLoader_(
        const std::type_info& type, const std::string& caption, const std::string& format,
        const std::string& extensions, std::shared_ptr<FileFunctionBase> function, int usage);
    void addSaver_(
        const std::type_info& type, const std::string& caption, const std::string& format,
        const std::string& extensions, std::shared_ptr<FileFunctionBase> function, int usage);

    static Item* createItemWithDialog_(
        const std::type_info& type, Item* parentItem, bool doAddition, Item* nextItem,
        Item* protoItem, const std::string& title);
    static ItemList<Item> loadItemsWithDialog_(
        const std::type_info& type, Item* parentItem, bool doAddtion, Item* nextItem);
    static bool saveItemWithDialog_(const std::type_info& type, Item* item);

    // The following static functions are called from functions in the Item class
    static bool load(
        Item* item, const std::string& filename, Item* parentItem, const std::string& format,
        const Mapping* options = nullptr);
    static bool save(
        Item* item, const std::string& filename, const std::string& format, const Mapping* options = nullptr);
    static bool overwrite(Item* item, bool forceOverwrite, const std::string& format);

    void registerAddon_(
        const std::type_info& type, const std::string& name, const std::function<ItemAddon*(void)>& factory);

    friend class Item;

    Impl* impl;
};

CNOID_EXPORT std::string getOpenFileName(const std::string& caption, const std::string& extensions);
CNOID_EXPORT std::vector<std::string> getOpenFileNames(const std::string& caption, const std::string& extensions);

}

#endif
