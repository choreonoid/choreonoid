/**
   @author Shin'ichiro Nakaoka
*/

#include "ItemManager.h"
#include "Item.h"
#include "ItemAddon.h"
#include "RootItem.h"
#include "ItemClassRegistry.h"
#include "ItemFileIO.h"
#include "ItemFileIOImpl.h"
#include "ItemFileDialog.h"
#include "FileDialog.h"
#include "MenuManager.h"
#include "AppConfig.h"
#include "MainWindow.h"
#include "MessageView.h"
#include "CheckBox.h"
#include <cnoid/FileUtil>
#include <cnoid/ExecutablePath>
#include <cnoid/ParametricPathProcessor>
#include <QLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QDialog>
#include <QDialogButtonBox>
#include <QSignalMapper>
#include <QRegExp>
#include <fmt/format.h>
#include <chrono>
#include <set>
#include <sstream>
#include <typeindex>
#include "gettext.h"

using namespace std;
using namespace cnoid;
namespace filesystem = cnoid::stdx::filesystem;
using fmt::format;

namespace {

class CreationPanelBase;

struct ClassInfo : public Referenced
{
    ItemManagerImpl* manager;
    string className;
    string name; // without the 'Item' suffix
    function<Item*()> factory;
    CreationPanelBase* creationPanelBase;
    vector<ItemFileIOPtr> fileIOs;
    ItemPtr singletonInstance;
    bool isSingleton;

    ClassInfo();
    ~ClassInfo();
};
typedef ref_ptr<ClassInfo> ClassInfoPtr;
    
typedef map<std::type_index, ClassInfoPtr> ItemTypeToInfoMap;
typedef map<string, ClassInfoPtr> ItemClassNameToInfoMap;

class DefaultCreationPanel : public ItemCreationPanel
{
    QLineEdit* nameEntry;
        
public:

    DefaultCreationPanel(QWidget* parent)
        : ItemCreationPanel(parent) {
        QHBoxLayout* layout = new QHBoxLayout();
        layout->addWidget(new QLabel(_("Name:")));
        nameEntry = new QLineEdit();
        layout->addWidget(nameEntry);
        setLayout(layout);
    }
        
    virtual bool initializePanel(Item* protoItem) {
        nameEntry->setText(protoItem->name().c_str());
        return true;
    }
            
    virtual bool initializeItem(Item* protoItem) {
        protoItem->setName(nameEntry->text().toStdString());
        return true;
    }
};

ItemClassRegistry* itemClassRegistry = nullptr;
MessageView* messageView = nullptr;
bool isStaticMembersInitialized = false;

ItemTypeToInfoMap itemTypeToInfoMap;

typedef map<string, ItemManagerImpl*> ModuleNameToItemManagerImplMap;
ModuleNameToItemManagerImplMap moduleNameToItemManagerImplMap;

typedef map<string, vector<ItemFileIO*>> CaptionToFileIoMap;
CaptionToFileIoMap captionToStandardLoaderMap;
CaptionToFileIoMap captionToConversionLoaderMap;
    
QWidget* importMenu;

std::map<ItemPtr, ItemPtr> reloadedItemToOriginalItemMap;

class AddonClassInfo : public Referenced
{
public:
    ItemManagerImpl* manager;
    std::string name;
    std::function<ItemAddon*(void)> factory;
};

typedef ref_ptr<AddonClassInfo> AddonClassInfoPtr;

typedef map<std::type_index, AddonClassInfoPtr> AddonTypeToInfoMap;
typedef map<string, AddonClassInfoPtr> AddonNameToInfoMap;
    
AddonTypeToInfoMap addonTypeToInfoMap;

}

namespace cnoid {

class ItemManagerImpl
{
public:
    ItemManagerImpl(const string& moduleName, MenuManager& menuManager);
    ~ItemManagerImpl();

    string moduleName;
    string textDomain;
    MenuManager& menuManager;

    ItemClassNameToInfoMap itemClassNameToInfoMap;
    set<std::type_index> registeredTypes;

    typedef list<shared_ptr<ItemManager::CreationPanelFilterBase>> CreationPanelFilterList;
    typedef set<pair<std::type_index, shared_ptr<ItemManager::CreationPanelFilterBase>>> CreationPanelFilterSet;
    
    set<ItemCreationPanel*> registeredCreationPanels;
    CreationPanelFilterSet registeredCreationPanelFilters;
    
    set<ItemFileIOPtr> registeredFileIOs;
    
    QSignalMapper* mapperForNewItemActivated;
    QSignalMapper* mapperForLoadSpecificTypeItemActivated;

    AddonNameToInfoMap addonNameToInfoMap;


    void detachManagedTypeItems(Item* parentItem);
        
    void registerClass(
        function<Item*()>& factory, Item* singletonInstance, const std::type_info& type, const string& className);
    
    void addCreationPanel(const std::type_info& type, ItemCreationPanel* panel);
    void addCreationPanelFilter(
        const std::type_info& type, shared_ptr<ItemManager::CreationPanelFilterBase> filter, bool afterInitializionByPanels);
    CreationPanelBase* getOrCreateCreationPanelBase(const std::type_info& type);

    ClassInfoPtr registerFileIO(const type_info& typeId, ItemFileIOPtr fileIO);
    void addLoader(ItemFileIO* fileIO, CaptionToFileIoMap& loaderMap);

    static bool load(
        Item* item, const string& filename, Item* parentItem, const string& formatId, const Mapping* options);
    static ItemFileIO* findFileIOForLoading(const type_info& type, const string& filename, const string& formatId);

    static void loadItemsWithDialog(const vector<ItemFileIO*>& fileIOs);

    static bool save(Item* item, bool useDialogToGetFilename, bool doExport, string filename, const string& formatId);
    static ItemFileIOPtr getFileIOAndFilenameFromSaveDialog(
        vector<ItemFileIOPtr>& fileIOs, bool doExport,
        const string& itemLabel, const string& formatId, string& io_filename);
    static ItemFileIOPtr determineFileIOForSaving(
        vector<ItemFileIOPtr>& fileIOs, const string& filename, const string& formatId);
    static bool overwrite(Item* item, bool forceOverwrite, const string& formatId);

    void onNewItemActivated(CreationPanelBase* base);
    void onReloadSelectedItemsActivated();
    void onSaveSelectedItemsActivated();
    void onSaveSelectedItemsAsActivated();
    void onSaveAllItemsActivated();
    void onExportSelectedItemsActivated();
};

}

namespace {

class CreationPanelBase : public QDialog
{
public:
    CreationPanelBase(const QString& title, ClassInfo* classInfo, ItemPtr protoItem, bool isSingleton);
    void addPanel(ItemCreationPanel* panel);
    Item* createItem(Item* parentItem);
    ItemManagerImpl::CreationPanelFilterList preFilters;
    ItemManagerImpl::CreationPanelFilterList postFilters;
    ClassInfo* classInfo;
    QVBoxLayout* panelLayout;
    ItemPtr protoItem;
    bool isSingleton;
};

}


ClassInfo::ClassInfo()
{
    creationPanelBase = nullptr;
}


ClassInfo::~ClassInfo()
{
    delete creationPanelBase;
}


ItemManager::ItemManager(const std::string& moduleName, MenuManager& menuManager)
{
    impl = new ItemManagerImpl(moduleName, menuManager);
}


ItemManagerImpl::ItemManagerImpl(const string& moduleName, MenuManager& menuManager)
    : moduleName(moduleName),
      menuManager(menuManager)
{
    if(!isStaticMembersInitialized){

        itemClassRegistry = &ItemClassRegistry::instance();

        menuManager.setPath("/File").setPath(N_("New ..."));
        
        menuManager.setPath("/File");
        menuManager.setPath(N_("Open ..."));
        menuManager.setPath("/File");
        menuManager.addItem(_("Reload Selected Items"))
            ->sigTriggered().connect([&](){ onReloadSelectedItemsActivated(); });
        
        menuManager.addSeparator();

        menuManager.addItem(_("Save Selected Items"))
            ->sigTriggered().connect([&](){ onSaveSelectedItemsActivated(); });
        menuManager.addItem(_("Save Selected Items As"))
            ->sigTriggered().connect([&](){ onSaveSelectedItemsAsActivated(); });
        menuManager.addItem(_("Save All Items"))
            ->sigTriggered().connect([&](){ onSaveAllItemsActivated(); });

        menuManager.addSeparator();
        
        menuManager.setPath(N_("Import ..."));
        importMenu = menuManager.current();
        
        menuManager.setPath("/File");
        menuManager.addItem(_("Export Selected Items"))
            ->sigTriggered().connect([&](){ onExportSelectedItemsActivated(); });
        
        menuManager.addSeparator();

        messageView = MessageView::mainInstance();

        isStaticMembersInitialized = true;
    }

    moduleNameToItemManagerImplMap[moduleName] = this;
}


void ItemManager::addMenuItemToImport(const std::string& caption, std::function<void()> slot)
{
    impl->menuManager.setCurrent(importMenu).addItem(caption.c_str())->sigTriggered().connect(slot);
}


ItemManager::~ItemManager()
{
    delete impl;
}


ItemManagerImpl::~ItemManagerImpl()
{
    for(auto it = registeredCreationPanels.begin(); it != registeredCreationPanels.end(); ++it){
        ItemCreationPanel* panel = *it;
        delete panel;
    }

    for(auto& fileIO : registeredFileIOs){
        auto& ioImpl = fileIO->impl;
        if((ioImpl->api & ItemFileIO::Load) && (ioImpl->interfaceLevel == ItemFileIO::Standard)){
            auto p = captionToStandardLoaderMap.find(ioImpl->caption);
            if(p != captionToStandardLoaderMap.end()){
                auto& loaders = p->second;
                loaders.erase(std::remove(loaders.begin(), loaders.end(), fileIO), loaders.end());
            }
            p = captionToConversionLoaderMap.find(ioImpl->caption);
            if(p != captionToConversionLoaderMap.end()){
                auto& loaders = p->second;
                loaders.erase(std::remove(loaders.begin(), loaders.end(), fileIO), loaders.end());
            }
        }
    }
    
    for(auto& type : registeredTypes){
        itemTypeToInfoMap.erase(type);
        addonTypeToInfoMap.erase(type);
    }

    for(auto p = registeredCreationPanelFilters.begin(); p != registeredCreationPanelFilters.end(); ++p){
        auto q = itemTypeToInfoMap.find(p->first);
        if(q != itemTypeToInfoMap.end()){
            ClassInfoPtr& classInfo = q->second;
            classInfo->creationPanelBase->preFilters.remove(p->second);
            classInfo->creationPanelBase->postFilters.remove(p->second);
        }
    }

    moduleNameToItemManagerImplMap.erase(moduleName);
}


void ItemManager::detachAllManagedTypeItemsFromRoot()
{
    impl->detachManagedTypeItems(RootItem::instance());
}


void ItemManagerImpl::detachManagedTypeItems(Item* parentItem)
{
    Item* item = parentItem->childItem();
    while(item){
        Item* nextItem = item->nextItem();
        if(registeredTypes.find(typeid(*item)) != registeredTypes.end()){
            item->detachFromParentItem();
        } else {
            detachManagedTypeItems(item);
        }
        item = nextItem;
    }
}


void ItemManager::bindTextDomain(const std::string& domain)
{
    impl->textDomain = domain;
}


void ItemManager::registerClass_
(const std::string& className, const std::type_info& type, const std::type_info& superType,
 std::function<Item*()> factory, Item* singletonInstance)
{
    if(factory || singletonInstance){
        impl->registerClass(factory, singletonInstance, type, className);
    }
    itemClassRegistry->registerClassAsTypeInfo(type, superType);
}


void ItemManagerImpl::registerClass
(std::function<Item*()>& factory, Item* singletonInstance, const std::type_info& type, const string& className)
{
    auto inserted = itemClassNameToInfoMap.insert(make_pair(className, ClassInfoPtr()));
    ClassInfoPtr& info = inserted.first->second;
    if(inserted.second){
        info = new ClassInfo;
        info->manager  = this;
        info->className = className;

        // set the class name without the "Item" suffix
        QString name(className.c_str());
        name.replace(QRegExp("Item$"), "");
        info->name = name.toStdString();
    }

    if(singletonInstance){
        if(singletonInstance->name().empty()){
            singletonInstance->setName(info->name);
        }
        info->singletonInstance = singletonInstance;
        info->isSingleton = true;
    } else {
        info->factory = factory;
        info->isSingleton = false;
    }

    registeredTypes.insert(type);
    itemTypeToInfoMap[type] = info;
}


bool ItemManager::getClassIdentifier(Item* item, std::string& out_moduleName, std::string& out_className)
{
    bool result;

    auto p = itemTypeToInfoMap.find(typeid(*item));
    if(p != itemTypeToInfoMap.end()){
        auto& info = p->second;
        out_moduleName = info->manager->moduleName;
        out_className = info->className;
        result = true;
    } else {
        out_moduleName.clear();
        out_className = typeid(*item).name();
        result = false;
    }

    return result;
}


Item* ItemManager::getSingletonInstance(const std::type_info& type)
{
    auto p = itemTypeToInfoMap.find(type);
    if(p != itemTypeToInfoMap.end()){
        auto& info = p->second;
        if(info->isSingleton){
            return info->singletonInstance;
        }
    }
    return nullptr;
}


bool ItemFileIO::Impl::isRegisteredForSingletonItem() const
{
    if(auto info = static_pointer_cast<ClassInfo>(classInfo.lock())){
        return info->isSingleton;
    }
    return false;
}


Item* ItemFileIO::Impl::findSingletonItemInstance() const
{
    if(auto info = static_pointer_cast<ClassInfo>(classInfo.lock())){
        return info->singletonInstance;
    }
    return nullptr;
}


Item* ItemManager::createItem(const std::string& moduleName, const std::string& className)
{
    Item* item = nullptr;

    auto p = moduleNameToItemManagerImplMap.find(moduleName);
    if(p != moduleNameToItemManagerImplMap.end()){
        auto& itemClassNameToInfoMap = p->second->itemClassNameToInfoMap;
        auto q = itemClassNameToInfoMap.find(className);
        if(q != itemClassNameToInfoMap.end()){
            auto& info = q->second;
            if(info->isSingleton){
                if(info->singletonInstance->parentItem()){
                    //! \todo put a warning message to notify that the instance of this singleton class has been in the item tree
                } else {
                    item = info->singletonInstance;
                }
            } else {
                if(info->factory){
                    item = info->factory();
                }
            }
        }
    }

    return item;
}


Item* ItemManager::createItemWithDialog_
(const std::type_info& type, Item* parentItem, bool doAddition, Item* nextItem)
{
    Item* newItem = nullptr;
    
    auto iter = itemTypeToInfoMap.find(type);
    if(iter == itemTypeToInfoMap.end()){
        showWarningDialog(format(_("Class {} is not registered as an item class."), type.name()));

    } else {
        auto& info = iter->second;
        auto panel = info->creationPanelBase;
        if(!panel){
            showWarningDialog(format(_("The panel to create {} is not registered."), info->className));
        } else {
            if(!parentItem){
                parentItem = RootItem::instance();
            }
            newItem = panel->createItem(parentItem);
        }
    }

    if(newItem && doAddition){
        parentItem->insertChildItem(newItem, nextItem);
    }
    
    return newItem;
}


void ItemManager::addCreationPanel_(const std::type_info& type, ItemCreationPanel* panel)
{
    impl->addCreationPanel(type, panel);
}


void ItemManagerImpl::addCreationPanel(const std::type_info& type, ItemCreationPanel* panel)
{
    CreationPanelBase* base = getOrCreateCreationPanelBase(type);
    if(panel){
        base->addPanel(panel);
    } else {
        base->addPanel(new DefaultCreationPanel(base));
    }
    registeredCreationPanels.insert(panel);
}


void ItemManager::addCreationPanelFilter_
(const std::type_info& type, std::shared_ptr<CreationPanelFilterBase> filter, bool afterInitializionByPanels)
{
    impl->addCreationPanelFilter(type, filter, afterInitializionByPanels);
}


void ItemManagerImpl::addCreationPanelFilter
(const std::type_info& type, shared_ptr<ItemManager::CreationPanelFilterBase> filter, bool afterInitializionByPanels)
{
    CreationPanelBase* base = getOrCreateCreationPanelBase(type);
    if(!afterInitializionByPanels){
        base->preFilters.push_back(filter);
    } else {
        base->postFilters.push_back(filter);
    }
    registeredCreationPanelFilters.insert(make_pair(std::type_index(type), filter));
}


CreationPanelBase* ItemManagerImpl::getOrCreateCreationPanelBase(const std::type_info& type)
{
    CreationPanelBase* base = nullptr;
    
    auto p = itemTypeToInfoMap.find(type);
    if(p != itemTypeToInfoMap.end()){
        auto& info = p->second;
        base = info->creationPanelBase;
        if(!base){
            const char* className_c_str = info->className.c_str();
            QString className(className_c_str);
            const char* translatedClassName_c_str = dgettext(textDomain.c_str(), className_c_str);
            QString translatedClassName(translatedClassName_c_str);
            QString translatedName(translatedClassName);
            if(translatedClassName_c_str == className_c_str){
                translatedName.replace(QRegExp("Item$"), "");
            } else {
                translatedName.replace(QRegExp(_("Item$")), "");
            }
            QString title(QString(_("Create New %1")).arg(translatedClassName));
            ItemPtr protoItem;
            if(info->isSingleton){
                protoItem = info->singletonInstance;
            }
            base = new CreationPanelBase(title, info, protoItem, info->isSingleton);
            base->hide();
            menuManager.setPath("/File/New ...").addItem(translatedName)
                ->sigTriggered().connect([=](){ onNewItemActivated(base); });
            info->creationPanelBase = base;
        }
    }

    return base;
}


void ItemManagerImpl::onNewItemActivated(CreationPanelBase* base)
{
    ItemList<Item> parentItems = RootItem::instance()->selectedItems();

    if(parentItems.empty()){
        parentItems.push_back(RootItem::instance());
    }
    for(size_t i=0; i < parentItems.size(); ++i){
        auto parentItem = parentItems[i];
        auto newItem = base->createItem(parentItem);
        if(newItem){
            parentItem->addChildItem(newItem, true);
        }
    }
}


CreationPanelBase::CreationPanelBase
(const QString& title, ClassInfo* classInfo, ItemPtr protoItem, bool isSingleton)
    : QDialog(MainWindow::instance()),
      classInfo(classInfo),
      protoItem(protoItem),
      isSingleton(isSingleton)
{
    setWindowTitle(title);
    
    QPushButton* createButton = new QPushButton(_("&Create"));
    createButton->setDefault(true);

    QPushButton* cancelButton = new QPushButton(_("&Cancel"));

    QDialogButtonBox* buttonBox = new QDialogButtonBox(this);
    buttonBox->addButton(createButton, QDialogButtonBox::AcceptRole);
    buttonBox->addButton(cancelButton, QDialogButtonBox::RejectRole);
    connect(buttonBox,SIGNAL(accepted()), this, SLOT(accept()));
    connect(buttonBox,SIGNAL(rejected()), this, SLOT(reject()));

    QVBoxLayout* topLayout = new QVBoxLayout();
    panelLayout = new QVBoxLayout();
    topLayout->addLayout(panelLayout);
    topLayout->addWidget(buttonBox);
    setLayout(topLayout);
}


void CreationPanelBase::addPanel(ItemCreationPanel* panel)
{
    panelLayout->addWidget(panel);
}


Item* CreationPanelBase::createItem(Item* parentItem)
{
    if(isSingleton){
        if(protoItem->parentItem()){
            return nullptr;
        }
    }
            
    vector<ItemCreationPanel*> panels;

    int n = panelLayout->count();
    for(int i=0; i < n; ++i){
        ItemCreationPanel* panel =
            dynamic_cast<ItemCreationPanel*>(panelLayout->itemAt(i)->widget());
        if(panel){
            panels.push_back(panel);
        }
    }

    bool result = true;

    if(!protoItem && (!preFilters.empty() || !postFilters.empty())){
        protoItem = classInfo->factory();
        protoItem->setName(classInfo->name);
    }
    ItemPtr item = protoItem;
    if(!item){
        item = classInfo->factory();
        item->setName(classInfo->name);
    }

    for(auto p = preFilters.begin(); p != preFilters.end(); ++p){
        auto filter = *p;
        if(!(*filter)(item, parentItem)){
            result = false;
            break;
        }
    }

    if(result){
        for(size_t i=0; i < panels.size(); ++i){
            if(!panels[i]->initializePanel(item)){
                result = false;
                break;
            }
        }
    }

    if(result){
        if(exec() == QDialog::Accepted){
            for(size_t i=0; i < panels.size(); ++i){
                if(!panels[i]->initializeItem(item)){
                    result = false;
                    break;
                }
            }
            if(result){
                for(auto p = postFilters.begin(); p != postFilters.end(); ++p){
                    auto filter = *p;
                    if(!(*filter)(item, parentItem)){
                        result = false;
                        break;
                    }
                }
            }
        } else {
            result = false;
        }
    }
    
    if(!result){
        item = nullptr;
    } else if(item == protoItem && !isSingleton){
        item = item->duplicate();
    }

    return item.retn();
}


ItemCreationPanel* ItemCreationPanel::findPanelOnTheSameDialog(const std::string& name)
{
    QBoxLayout* layout = dynamic_cast<QBoxLayout*>(parentWidget());
 
    if(layout){
        int n = layout->count();
        for(int i=0; i < n; ++i){
            ItemCreationPanel* panel = dynamic_cast<ItemCreationPanel*>(layout->itemAt(i)->widget());
            if(panel){
                if(panel->objectName().toStdString() == name){
                    return panel;
                }
            }
        }
    }
    return nullptr;
}

namespace {

// Defined to use existing loaders and savers based on FileFunctionBase for the backward compatiblity
class FileFunctionAdapter : public ItemFileIO
{
public:
    std::shared_ptr<ItemManager::FileFunctionBase> fileFunction;
    function<Item*()> factory;
    
    FileFunctionAdapter(
        int api, const std::string& caption, const std::string& formatId,
        const std::function<std::string()>& getExtensions, std::shared_ptr<ItemManager::FileFunctionBase> function,
        int priority)
        : ItemFileIO(formatId, api),
          fileFunction(function)
    {
        setCaption(caption);
        setExtensionFunction(getExtensions);
        if(priority >= ItemManager::PRIORITY_DEFAULT){
            setInterfaceLevel(Standard);
        } else if(priority >= ItemManager::PRIORITY_COMPATIBILITY){
            setInterfaceLevel(Internal);
        } else {
            setInterfaceLevel(Conversion);
        }
    }

    virtual Item* createItem() override
    {
        if(factory){
            return factory();
        }
        return nullptr;
    }

    virtual bool load(Item* item, const std::string& filename) override
    {
        return (*fileFunction)(item, filename, os(), parentItem());
    };

    virtual bool save(Item* item, const std::string& filename) override
    {
        return (*fileFunction)(item, filename, os(), parentItem());
    };
};

}


void ItemManager::addLoaderSub
(const std::type_info& type, const std::string& caption, const std::string& formatId,
 std::function<std::string()> getExtensions, std::shared_ptr<FileFunctionBase> function, int priority)
{
    auto adapter = new FileFunctionAdapter(
        ItemFileIO::Load, caption, formatId, getExtensions, function, priority);
    auto classInfo = impl->registerFileIO(type, adapter);
    adapter->factory = classInfo->factory;
}


void ItemManager::addSaverSub
(const std::type_info& type, const std::string& caption, const std::string& formatId,
 std::function<std::string()> getExtensions, std::shared_ptr<FileFunctionBase> function, int priority)
{
    auto adapter = new FileFunctionAdapter(
        ItemFileIO::Save, caption, formatId, getExtensions, function, priority);
    impl->registerFileIO(type, adapter);
}


void ItemManager::registerFileIO_(const std::type_info& type, ItemFileIO* fileIO)
{
    impl->registerFileIO(type, fileIO);
}


ClassInfoPtr ItemManagerImpl::registerFileIO(const type_info& type, ItemFileIOPtr fileIO)
{
    ClassInfoPtr classInfo;
    
    auto p = itemTypeToInfoMap.find(type);
    if(p != itemTypeToInfoMap.end()){
        classInfo = p->second;
        auto& ioImpl = fileIO->impl;
        ioImpl->classInfo = classInfo;

        registeredFileIOs.insert(fileIO);

        auto p = classInfo->fileIOs.begin();
        while(p != classInfo->fileIOs.end()){
            if((*p)->impl->interfaceLevel > ioImpl->interfaceLevel){
                break;
            }
            ++p;
        }
        classInfo->fileIOs.insert(p, fileIO);

        if(ioImpl->api & ItemFileIO::Load){
            if(ioImpl->interfaceLevel == ItemFileIO::Standard){
                menuManager.setPath("/File/Open ...");
                addLoader(fileIO, captionToStandardLoaderMap);
            } else if(ioImpl->interfaceLevel == ItemFileIO::Conversion){
                menuManager.setPath("/File/Import ...");
                addLoader(fileIO, captionToConversionLoaderMap);
            }
        }
    }

    return classInfo;
}


void ItemManagerImpl::addLoader(ItemFileIO* fileIO, CaptionToFileIoMap& loaderMap)
{
    auto& caption = fileIO->caption();
    auto& loaders = loaderMap[caption];
    if(loaders.empty()){
        menuManager.addItem(caption)
            ->sigTriggered().connect(
                [this, caption, &loaderMap]() mutable {
                    loadItemsWithDialog(loaderMap[caption]); });
    }
    loaders.push_back(fileIO);
}


ItemFileIO* ItemManager::findFileIO(const std::type_info& type, const std::string& formatId)
{
    ItemFileIO* found = nullptr;
    auto p = itemTypeToInfoMap.find(type);
    if(p != itemTypeToInfoMap.end()){
        auto& classInfo = p->second;
        for(auto& fileIO : classInfo->fileIOs){
            if(formatId.empty() || fileIO->impl->isFormat(formatId)){
                found = fileIO;
                break;
            }
        }
    }
    return found;
}


bool ItemManager::load
(Item* item, const std::string& filename, Item* parentItem, const std::string& formatId, const Mapping* options)
{
    return ItemManagerImpl::load(item, filename, parentItem, formatId, options);
}


bool ItemManagerImpl::load
(Item* item, const string& filename, Item* parentItem, const string& formatId, const Mapping* options)
{
    if(auto fileIO = findFileIOForLoading(typeid(*item), filename, formatId)){
        return fileIO->loadItem(item, filename, parentItem, false, nullptr, options);
    }
    return false;
}


ItemList<Item> ItemManager::loadItemsWithDialog_
(const std::type_info& type, Item* parentItem, bool doAddtion, Item* nextItem)
{
    if(auto fileIO = ItemManagerImpl::findFileIOForLoading(type, "", "")){
        ItemFileDialog dialog;
        return dialog.loadItems({fileIO}, parentItem, doAddtion, nextItem);
    }
    return ItemList<Item>();
}


ItemFileIO* ItemManagerImpl::findFileIOForLoading
(const type_info& type, const string& filename, const string& formatId)
{
    ItemFileIO* targetFileIO = nullptr;
    
    auto p = itemTypeToInfoMap.find(type);
    if(p == itemTypeToInfoMap.end()){
        messageView->putln(
            format(_("\"{0}\" cannot be loaded because item type \"{1}\" is not registered."),
                   filename, type.name()),
            MessageView::ERROR);
        return targetFileIO;;
    }
    
    ClassInfoPtr& classInfo = p->second;
    auto& fileIOs = classInfo->fileIOs;

    if(!formatId.empty() || filename.empty()){
        for(auto& fileIO : fileIOs){
            if(fileIO->impl->api & ItemFileIO::Load){
                if(formatId.empty() || fileIO->impl->isFormat(formatId)){
                    targetFileIO = fileIO;
                    break;
                }
            }
        }
    } else if(!filename.empty()){
        filesystem::path filepath(filename);
        string dotextension = filepath.extension().string();
        if(dotextension.size() >= 2){
            string extension = dotextension.substr(1); // remove dot
            for(auto& fileIO : fileIOs){
                if(fileIO->impl->api & ItemFileIO::Load){
                    for(auto& ext : fileIO->extensions()){
                        if(ext == extension){
                            targetFileIO = fileIO;
                            break;
                        }
                    }
                }
            }
        }
    }

    if(!targetFileIO){
        if(formatId.empty()){
            messageView->putln(
                format(_("\"{}\" cannot be loaded because the file format is unknown."), filename),
                MessageView::ERROR);
        } else {
            messageView->putln(
                format(_("\"{0}\" cannot be loaded because file format \"{1}\" is unknown."),
                       filename, formatId),
                MessageView::ERROR);
        }
    }

    return targetFileIO;
}
        

void ItemManagerImpl::loadItemsWithDialog(const vector<ItemFileIO*>& fileIOs)
{
    Item* parentItem = RootItem::instance()->selectedItems().toSingle();
    if(!parentItem){
        parentItem = RootItem::instance();
    }
    
    ItemFileDialog dialog;
    dialog.loadItems(fileIOs, parentItem, true);
}


bool ItemManager::save(Item* item, const std::string& filename, const std::string& formatId)
{
    return ItemManagerImpl::save(item, false, false, filename, formatId);
}


/**
   \todo Move the implementation except for finding the target ItemFileIO into
   the ItemFileIO class like the functions for loading.
*/
bool ItemManagerImpl::save
(Item* item, bool useDialogToGetFilename, bool doExport, string filename, const string& formatId)
{
    auto p = itemTypeToInfoMap.find(typeid(*item));
    if(p == itemTypeToInfoMap.end()){
        return false;
    }
    ClassInfoPtr& classInfo = p->second;
   
    string itemLabel = classInfo->className + " \"" + item->name() + "\"";

    auto& fileIOs = classInfo->fileIOs;
    ItemFileIOPtr targetFileIO;
    if(useDialogToGetFilename){
        targetFileIO = getFileIOAndFilenameFromSaveDialog(fileIOs, doExport, itemLabel, formatId, filename);
        if(!targetFileIO){ // Canceled by a user
            messageView->put(format(_("Saving {0} was canceled."), itemLabel), MessageView::WARNING);
            return false;
        }
    } else {
        targetFileIO = determineFileIOForSaving(fileIOs, filename, formatId);
    }

    item->setTemporal(false);
    bool saved = false;
    bool tryToSave = false;

    if(targetFileIO && targetFileIO->impl->api & ItemFileIO::Save){
        
        tryToSave = true;
        
        if(!doExport){
            messageView->notify(format(_("Saving {0} to \"{1}\""), itemLabel, filename));
        } else {
            messageView->notify(format(_("Exporting {0} into \"{1}\""), itemLabel, filename));
        }
        
        Item* parentItem = item->parentItem();
        if(!parentItem){
            parentItem = RootItem::instance();
        }
        targetFileIO->impl->parentItem = parentItem;

        saved = targetFileIO->save(item, filename);
        targetFileIO->impl->os->flush();
        
        if(!saved){
            messageView->put(MessageView::HIGHLIGHT, _(" -> failed.\n"));
        } else {
            if(targetFileIO->impl->interfaceLevel != ItemFileIO::Conversion){
                item->updateFileInformation(filename, targetFileIO->impl->formatId, nullptr);
            }
            messageView->put(_(" -> ok!\n"));
        }
    }
    
    if(!tryToSave){
        string actualFormatId = targetFileIO ? targetFileIO->impl->formatId : formatId;
        if(actualFormatId.empty()){
            if(!doExport){
                messageView->put(format(_("{} cannot be saved.\n"), itemLabel));
            } else {
                messageView->put(format(_("{} cannot be exported.\n"), itemLabel));
            }
        } else {
            if(!doExport){
                messageView->put(format(_("{0} cannot be saved as the {1} format.\n"), itemLabel, actualFormatId));
            } else {
                messageView->put(format(_("{0} cannot be exported into the {1} format.\n"), itemLabel, actualFormatId));
            }
        }
    }

    messageView->flush();

    return saved;
}


ItemFileIOPtr ItemManagerImpl::getFileIOAndFilenameFromSaveDialog
(vector<ItemFileIOPtr>& fileIOs, bool doExport, const string& itemLabel, const string& formatId,
 string& io_filename)
{
    ItemFileIOPtr targetFileIO;

    QStringList filters;
    vector<ItemFileIOPtr> activeFileIOs;
    for(auto& fileIO : fileIOs){
        if(!(fileIO->impl->api & ItemFileIO::Save)){
            continue;
        }
        if(fileIO->impl->interfaceLevel == ItemFileIO::Internal){
            continue;
        }
        bool isExporter = (fileIO->impl->interfaceLevel == ItemFileIO::Conversion);
        if((doExport && !isExporter) || (!doExport && isExporter)){
            continue;
        }
        if(!formatId.empty() && !fileIO->impl->isFormat(formatId)){
            continue;
        }
        filters << ItemFileIO::Impl::makeNameFilter(
            fileIO->caption(),
            fileIO->extensions(),
            true);
        activeFileIOs.push_back(fileIO);
    }

    if(filters.size() > 0){

        FileDialog dialog(MainWindow::instance());
        dialog.setWindowTitle(QString(_("Save %1 as")).arg(itemLabel.c_str()));
        dialog.setFileMode(QFileDialog::AnyFile);
        dialog.setAcceptMode(QFileDialog::AcceptSave);
        dialog.setViewMode(QFileDialog::List);
        dialog.setLabelText(QFileDialog::Accept, _("Save"));
        dialog.setLabelText(QFileDialog::Reject, _("Cancel"));
        dialog.setNameFilters(filters);
        dialog.updatePresetDirectories();

        if(!io_filename.empty()){
            dialog.selectFile(io_filename.c_str());
            io_filename.clear();
        }

        if(dialog.exec() == QFileDialog::Accepted){

            io_filename = dialog.selectedFiles()[0].toStdString();
            if(!io_filename.empty()){
                int fileIOIndex = -1;
                QString selectedFilter = dialog.selectedNameFilter();
                for(int i=0; i < filters.size(); ++i){
                    if(filters[i] == selectedFilter){
                        fileIOIndex = i;
                        break;
                    }
                }
                if(fileIOIndex >= 0){
                    targetFileIO = activeFileIOs[fileIOIndex];
                    auto exts = targetFileIO->extensions();
                    // add a lacking extension automatically
                    if(!exts.empty()){
                        bool hasExtension = false;
                        string dotextension = filesystem::path(io_filename).extension().string();
                        if(!dotextension.empty()){
                            string extension = dotextension.substr(1); // remove the first dot
                            if(std::find(exts.begin(), exts.end(), extension) != exts.end()){
                                hasExtension = true;
                            }
                        }
                        if(!hasExtension && !exts.empty()){
                            io_filename += ".";
                            io_filename += exts[0];
                        }
                    }
                }
            }
        }
    }

    return targetFileIO;
}


ItemFileIOPtr ItemManagerImpl::determineFileIOForSaving
(vector<ItemFileIOPtr>& fileIOs, const string& filename, const string& formatId)
{
    ItemFileIOPtr targetFileIO;

    if(!formatId.empty()){
        for(auto& fileIO : fileIOs){
            if(fileIO->impl->api & ItemFileIO::Save){
                if(fileIO->impl->isFormat(formatId)){
                    targetFileIO = fileIO;
                    break;
                }
            }
        }
    } else {
        string dotextension = filesystem::path(filename).extension().string();
        if(!dotextension.empty()){
            string extension = dotextension.substr(1);
            for(auto& fileIO : fileIOs){
                if(fileIO->impl->api & ItemFileIO::Save){
                    for(auto& ext : fileIO->extensions()){
                        if(ext == extension){
                            targetFileIO = fileIO;
                            break;
                        }
                    }
                }
            }
        }
        if(!targetFileIO){
            for(auto& fileIO : fileIOs){
                if(fileIO->impl->api & ItemFileIO::Save){
                    targetFileIO = fileIO;
                    break;
                }
            }
        }
    }

    return targetFileIO;
}


bool ItemManager::overwrite(Item* item, bool forceOverwrite, const std::string& formatId)
{
    return ItemManagerImpl::overwrite(item, forceOverwrite, formatId);
}


bool ItemManagerImpl::overwrite(Item* item, bool forceOverwrite, const string& formatId)
{
    item->setTemporal(false);
    
    bool needToOverwrite = forceOverwrite;

    string filename(item->filePath());
    string lastFormatId(item->fileFormat());

    string defaultFilenameOnDialog;
    if(filename.empty()){
        defaultFilenameOnDialog = item->name();
    }

    if(!formatId.empty() && formatId != lastFormatId){
        needToOverwrite = true;
    } else {
        if(!filename.empty()){
            filesystem::path fpath(filename);
            if(!filesystem::exists(fpath) ||
               filesystem::last_write_time_to_time_t(fpath) > item->fileModificationTime()){
                needToOverwrite = true;
                filename.clear();
            }
        }
    }
    if(!needToOverwrite && !item->isConsistentWithFile()){
        needToOverwrite = true;
    }

    bool synchronized = !needToOverwrite;
    
    if(!synchronized){
        if(!filename.empty() && formatId.empty()){
            synchronized = save(item, false, false, filename, lastFormatId);
        } 
        if(!synchronized){
            synchronized = save(item, true, false, defaultFilenameOnDialog, formatId);
        }
    }

    return synchronized;
}


void ItemManagerImpl::onReloadSelectedItemsActivated()
{
    ItemManager::reloadItems(RootItem::instance()->selectedItems());
}


void ItemManager::reloadItems(const ItemList<>& items)
{
    reloadedItemToOriginalItemMap.clear();
    
    for(size_t i=0; i < items.size(); ++i){

        Item* item = items.get(i);

        if(item->parentItem() && !item->isSubItem() &&
           !item->filePath().empty() && !item->fileFormat().empty()){

            ItemPtr reloaded = item->duplicate();
            if(reloaded){
                if(reloaded->load(item->filePath(), item->parentItem(), item->fileFormat())){

                    reloadedItemToOriginalItemMap[reloaded] = item;

                    item->parentItem()->insertChildItem(reloaded, item);
                    
                    // move children to the reload item
                    ItemPtr child = item->childItem();
                    while(child){
                        ItemPtr nextChild = child->nextItem();
                        if(!child->isSubItem()){
                            child->detachFromParentItem();
                            reloaded->addChildItem(child);
                        }
                        child = nextChild;
                    }
                    reloaded->assign(item);

                    item->detachFromParentItem();
                }
            }
        }
    }

    reloadedItemToOriginalItemMap.clear();
}


Item* ItemManager::findOriginalItemForReloadedItem(Item* item)
{
    auto iter = reloadedItemToOriginalItemMap.find(item);
    if(iter != reloadedItemToOriginalItemMap.end()){
        return iter->second;
    }
    return nullptr;
}


void ItemManagerImpl::onSaveSelectedItemsActivated()
{
    const ItemList<>& selectedItems = RootItem::instance()->selectedItems();
    for(size_t i=0; i < selectedItems.size(); ++i){
        overwrite(selectedItems.get(i), true, "");
    }
}


void ItemManagerImpl::onSaveSelectedItemsAsActivated()
{
    const ItemList<>& selectedItems = RootItem::instance()->selectedItems();
    for(size_t i=0; i < selectedItems.size(); ++i){
        string formatId;
        save(selectedItems.get(i), true, false, selectedItems[i]->headItem()->name(), formatId);
    }
}


void ItemManagerImpl::onSaveAllItemsActivated()
{

}


void ItemManagerImpl::onExportSelectedItemsActivated()
{
    const ItemList<>& selectedItems = RootItem::instance()->selectedItems();
    for(size_t i=0; i < selectedItems.size(); ++i){
        string formatId;
        save(selectedItems.get(i), true, true, selectedItems[i]->headItem()->name(), formatId);
    }
}


void ItemManager::registerAddon_
(const std::type_info& type, const std::string& name, const std::function<ItemAddon*(void)>& factory)
{
    auto info = new AddonClassInfo;
    info->manager = impl;
    info->name = name;
    info->factory = factory;
    impl->addonNameToInfoMap[name] = info;
    addonTypeToInfoMap[type] = info;
}


ItemAddon* ItemManager::createAddon(const std::type_info& type)
{
    ItemAddon* addon = nullptr;
    auto p = addonTypeToInfoMap.find(type);
    if(p != addonTypeToInfoMap.end()){
        auto& info = p->second;
        addon = info->factory();
    }
    return addon;
}


ItemAddon* ItemManager::createAddon(const std::string& moduleName, const std::string& addonName)
{
    ItemAddon* addon = nullptr;
    auto p = moduleNameToItemManagerImplMap.find(moduleName);
    if(p != moduleNameToItemManagerImplMap.end()){
        auto& addonNameToInfoMap = p->second->addonNameToInfoMap;
        auto q = addonNameToInfoMap.find(addonName);
        if(q != addonNameToInfoMap.end()){
            auto& info = q->second;
            addon = info->factory();
        }
    }
    return addon;
}


bool ItemManager::getAddonIdentifier(ItemAddon* addon, std::string& out_moduleName, std::string& out_addonName)
{
    bool result;
    auto p = addonTypeToInfoMap.find(typeid(*addon));
    if(p != addonTypeToInfoMap.end()){
        auto& info = p->second;
        out_moduleName = info->manager->moduleName;
        out_addonName = info->name;
        result = true;
    } else {
        out_moduleName.clear();
        out_addonName = typeid(*addon).name();
        result = false;
    }
    return result;
}
    

static QString makeNameFilterString(const std::string& caption, const string& extensions)
{
    QString filters =
        ItemFileIO::Impl::makeNameFilter(
            caption,
            ItemFileIO::Impl::separateExtensions(extensions));
    filters += _(";;Any files (*)");
    return filters;
}


namespace cnoid {

string getOpenFileName(const string& caption, const string& extensions)
{
    string filename;
    FileDialog dialog(MainWindow::instance());
    dialog.setWindowTitle(caption.c_str());
    dialog.setNameFilter(makeNameFilterString(caption, extensions));
    dialog.setFileMode(QFileDialog::ExistingFile);
    dialog.updatePresetDirectories();
    if(dialog.exec() == QDialog::Accepted){
        filename = dialog.selectedFiles().value(0).toStdString();
    }
    return filename;
}
    

vector<string> getOpenFileNames(const string& caption, const string& extensions)
{
    vector<string> filenames;
    FileDialog dialog(MainWindow::instance());
    dialog.setWindowTitle(caption.c_str());
    dialog.setNameFilter(makeNameFilterString(caption, extensions));
    dialog.setFileMode(QFileDialog::ExistingFiles);
    dialog.updatePresetDirectories();
    if(dialog.exec() == QDialog::Accepted){
        for(auto& file : dialog.selectedFiles()){
            filenames.push_back(file.toStdString());
        }
    }
    return filenames;
}

}
