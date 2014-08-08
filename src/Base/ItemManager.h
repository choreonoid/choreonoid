/**
   @author Shin'ichiro NAKAOKA
*/

#ifndef CNOID_BASE_ITEM_MANAGER_H
#define CNOID_BASE_ITEM_MANAGER_H

#include "ExtensionManager.h"
#include "ItemList.h"
#include <string>
#include <typeinfo>
#include <iosfwd>
#include <boost/function.hpp>
#include <QWidget>
#include "exportdecl.h"

namespace cnoid {

class MenuManager;

class Item;
typedef ref_ptr<Item> ItemPtr;

class ItemManagerImpl;

class ItemCreationPanel : public QWidget
{
public:
    ItemCreationPanel(QWidget* parent = 0) : QWidget(parent) { }

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

    class FactoryBase {
    public:
        virtual ItemPtr create() = 0;
        virtual ~FactoryBase() { }
    };

    template <class ItemType> class Factory : public FactoryBase {
    public:
        virtual ItemPtr create() { return new ItemType(); }
    };

    class CreationPanelFilterBase
    {
    public:
        virtual ~CreationPanelFilterBase() { }
        virtual bool operator()(Item* protoItem, Item* parentItem) = 0;
    };
    typedef boost::shared_ptr<CreationPanelFilterBase> CreationPanelFilterBasePtr;
        

    class FileFunctionBase
    {
    public:
        virtual ~FileFunctionBase() { }
        virtual bool operator()(Item* item, const std::string& filename, std::ostream& os, Item* parentItem) = 0;
    };
    typedef boost::shared_ptr<FileFunctionBase> FileFunctionBasePtr;


    class OverwritingCheckFunctionBase
    {
    public:
        ~OverwritingCheckFunctionBase();
        virtual bool operator()(Item* item) = 0;
    };
    typedef boost::shared_ptr<OverwritingCheckFunctionBase> OverwritingCheckFunctionBasePtr;
        

    void registerClassSub(FactoryBase* factory, const std::string& typeId, const std::string& className);
        
    void addCreationPanelSub(const std::string& typeId, ItemCreationPanel* panel);

    void addCreationPanelFilterSub(
        const std::string& typeId, CreationPanelFilterBasePtr filter, bool afterInitializionByPanels);

    void addLoaderSub(const std::string& typeId, const std::string& caption, const std::string& formatId,
                      const std::string& extensions, FileFunctionBasePtr function, int priority);

    void addSaverSub(const std::string& typeId, const std::string& caption, const std::string& formatId,
                     const std::string& extensions, FileFunctionBasePtr function, int priority);

public:

    void bindTextDomain(const std::string& domain);

    enum { PRIORITY_CONVERSION = -10, PRIORITY_OPTIONAL = 0, PRIORITY_DEFAULT = 10, PRIORITY_FORCE = 20 };

    template <class ItemType> class CreationPanelFilter : public CreationPanelFilterBase
    {
    public:
        typedef boost::function<bool(ItemType* protoItem, Item* parentItem)> Function;
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
        typedef boost::function<bool(ItemType* item, const std::string& filename, std::ostream& os, Item* parentItem)> Function;
        FileFunction(Function function) : function(function) { }
        virtual bool operator()(Item* item, const std::string& filename, std::ostream& os, Item* parentItem){
            return function(static_cast<ItemType*>(item), filename, os, parentItem);
        }
    private:
        Function function;
    };

    template <class ItemType> ItemManager& registerClass(const std::string& className) {
        registerClassSub(new Factory<ItemType>(), typeid(ItemType).name(), className);
        return *this;
    }

    template <class ItemType, class BaseType>
        void registerDerivedClass(const std::string& className) {
        // registerClassSub(new Factory<ItemType>(), typeid(ItemType).name(), className);
    }

    static bool getClassIdentifier(ItemPtr item, std::string& out_moduleName, std::string& out_className);

    static ItemPtr create(const std::string& moduleName, const std::string& itemClassName);

    template <class ItemType> ItemManager& addCreationPanel(ItemCreationPanel* panel = 0) {
        addCreationPanelSub(typeid(ItemType).name(), panel);
        return *this;
    }

    template <class ItemType>
        void addCreationPanelPreFilter(const typename CreationPanelFilter<ItemType>::Function& filter) {
        addCreationPanelFilterSub(typeid(ItemType).name(),
                                  CreationPanelFilterBasePtr(new CreationPanelFilter<ItemType>(filter)),
                                  false);
    }
        
    template <class ItemType>
        void addCreationPanelPostFilter(const typename CreationPanelFilter<ItemType>::Function& filter){
        addCreationPanelFilterSub(typeid(ItemType).name(),
                                  CreationPanelFilterBasePtr(new CreationPanelFilter<ItemType>(filter)),
                                  true);
    }
    
    template <class ItemType>
        ItemManager& addLoader(const std::string& caption, const std::string& formatId, const std::string& extensions, 
                               const typename FileFunction<ItemType>::Function& function, int priority = PRIORITY_DEFAULT) {
        addLoaderSub(typeid(ItemType).name(), caption, formatId, extensions,
                     FileFunctionBasePtr(new FileFunction<ItemType>(function)), priority);
        return *this;
    }

    template<class ItemType>
        ItemManager& addSaver(const std::string& caption, const std::string& formatId, const std::string& extensions,
                              const typename FileFunction<ItemType>::Function& function, int priority = PRIORITY_DEFAULT){
        addSaverSub(typeid(ItemType).name(), caption, formatId, extensions,
                    FileFunctionBasePtr(new FileFunction<ItemType>(function)), priority);
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

    void addMenuItemToImport(const std::string& caption, boost::function<void()> slot);

    static void reloadItems(const ItemList<>& items);

private:
        
    // The following static functions are called from functions in the Item class
    static bool load(Item* item, const std::string& filename, Item* parentItem, const std::string& formatId);
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
