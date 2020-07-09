/**
   @author Shin'ichiro Nakaoka
*/

#include "ItemManager.h"
#include "Item.h"
#include "ItemAddon.h"
#include "RootItem.h"
#include "ItemClassRegistry.h"
#include "PluginManager.h"
#include "ItemFileIO.h"
#include "ItemFileDialog.h"
#include "FileDialog.h"
#include "MenuManager.h"
#include "AppConfig.h"
#include "MainWindow.h"
#include "MessageView.h"
#include "CheckBox.h"
#include <cnoid/ExecutablePath>
#include <cnoid/UTF8>
#include <cnoid/stdx/filesystem>
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
    ItemManager::Impl* manager;
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

ItemClassRegistry* itemClassRegistry = nullptr;
MessageView* messageView = nullptr;
bool isStaticMembersInitialized = false;

ItemTypeToInfoMap itemTypeToInfoMap;

typedef map<string, ItemManager::Impl*> ModuleNameToItemManagerImplMap;
ModuleNameToItemManagerImplMap moduleNameToItemManagerImplMap;

map<string, map<string, pair<string, string>>> aliasClassNameToAliasModuleNameToTrueNamePairMap;

typedef map<string, vector<ItemFileIO*>> CaptionToFileIoListMap;
CaptionToFileIoListMap captionToStandardLoadersMap;
CaptionToFileIoListMap captionToConversionLoadersMap;
    
QWidget* importMenu;

std::map<ItemPtr, ItemPtr> reloadedItemToOriginalItemMap;

class AddonClassInfo : public Referenced
{
public:
    ItemManager::Impl* manager;
    std::string name;
    std::function<ItemAddon*(void)> factory;
};

typedef ref_ptr<AddonClassInfo> AddonClassInfoPtr;

typedef map<std::type_index, AddonClassInfoPtr> AddonTypeToInfoMap;
typedef map<string, AddonClassInfoPtr> AddonNameToInfoMap;
    
AddonTypeToInfoMap addonTypeToInfoMap;

}

namespace cnoid {

class ItemManager::Impl
{
public:
    Impl(const string& moduleName, MenuManager& menuManager);
    ~Impl();

    string moduleName;
    string textDomain;
    MenuManager& menuManager;

    ItemClassNameToInfoMap itemClassNameToInfoMap;
    set<std::type_index> registeredTypes;

    typedef list<shared_ptr<CreationPanelFilterBase>> CreationPanelFilterList;
    typedef set<pair<std::type_index, shared_ptr<CreationPanelFilterBase>>> CreationPanelFilterSet;
    
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
        const std::type_info& type, shared_ptr<CreationPanelFilterBase> filter,
        bool afterInitializionByPanels);
    CreationPanelBase* getOrCreateCreationPanelBase(const std::type_info& type);
    static void onNewItemActivated(CreationPanelBase* base);

    ClassInfoPtr registerFileIO(const type_info& typeId, ItemFileIO* fileIO);
    void addLoader(ItemFileIO* fileIO, CaptionToFileIoListMap& loaderMap);
    static vector<ItemFileIO*> getFileIOs(Item* item, function<bool(ItemFileIO* fileIO)> pred);
    static ItemFileIO* findMatchedFileIO(
        const type_info& type, const string& filename, const string& formatId, int ioTypeFlag);
    static void onLoadOrImportItemsActivated(const vector<ItemFileIO*>& fileIOs);
    static void onReloadSelectedItemsActivated();
    static void onSaveSelectedItemsAsActivated();
    static void onExportSelectedItemsActivated();
    static void onSaveSelectedItemsActivated();
};

}

namespace {

class CreationPanelBase : public QDialog
{
public:
    CreationPanelBase(const QString& title, ClassInfo* classInfo, ItemPtr protoItem, bool isSingleton);
    void addPanel(ItemCreationPanel* panel);
    Item* createItem(Item* parentItem);
    ItemManager::Impl::CreationPanelFilterList preFilters;
    ItemManager::Impl::CreationPanelFilterList postFilters;
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
    impl = new Impl(moduleName, menuManager);
}


ItemManager::Impl::Impl(const string& moduleName, MenuManager& menuManager)
    : moduleName(moduleName),
      menuManager(menuManager)
{
    if(!isStaticMembersInitialized){

        itemClassRegistry = &ItemClassRegistry::instance();

        menuManager.setPath("/File").setPath(N_("New ..."));
        
        menuManager.setPath("/File");
        menuManager.setPath(N_("Load ..."));
        menuManager.setPath("/File");
        menuManager.addItem(_("Reload Selected Items"))
            ->sigTriggered().connect([&](){ onReloadSelectedItemsActivated(); });
        
        menuManager.addSeparator();

        menuManager.addItem(_("Save Selected Items"))
            ->sigTriggered().connect([&](){ onSaveSelectedItemsActivated(); });
        menuManager.addItem(_("Save Selected Items As"))
            ->sigTriggered().connect([&](){ onSaveSelectedItemsAsActivated(); });

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


ItemManager::Impl::~Impl()
{
    for(auto it = registeredCreationPanels.begin(); it != registeredCreationPanels.end(); ++it){
        ItemCreationPanel* panel = *it;
        delete panel;
    }

    for(auto& fileIO : registeredFileIOs){
        if(fileIO->hasApi(ItemFileIO::Load) && (fileIO->interfaceLevel() == ItemFileIO::Standard)){
            auto& caption = fileIO->caption();
            auto p = captionToStandardLoadersMap.find(caption);
            if(p != captionToStandardLoadersMap.end()){
                auto& loaders = p->second;
                loaders.erase(std::remove(loaders.begin(), loaders.end(), fileIO), loaders.end());
            }
            p = captionToConversionLoadersMap.find(caption);
            if(p != captionToConversionLoadersMap.end()){
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


void ItemManager::Impl::detachManagedTypeItems(Item* parentItem)
{
    Item* item = parentItem->childItem();
    while(item){
        Item* nextItem = item->nextItem();
        if(registeredTypes.find(typeid(*item)) != registeredTypes.end()){
            item->removeFromParentItem();
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


void ItemManager::Impl::registerClass
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


void ItemManager::addAlias_
(const std::type_info& type, const std::string& aliasClassName, const std::string& aliasModuleName)
{
    auto p = itemTypeToInfoMap.find(type);
    if(p != itemTypeToInfoMap.end()){
        auto classInfo =  p->second;
        aliasClassNameToAliasModuleNameToTrueNamePairMap[aliasClassName][aliasModuleName] =
            make_pair(classInfo->manager->moduleName, classInfo->className);
    }
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


bool ItemFileIO::isRegisteredForSingletonItem() const
{
    if(auto info = dynamic_cast<const ClassInfo*>(itemClassInfo())){
        return info->isSingleton;
    }
    return false;
}


Item* ItemFileIO::findSingletonItemInstance() const
{
    if(auto info = dynamic_cast<const ClassInfo*>(itemClassInfo())){
        return info->singletonInstance;
    }
    return nullptr;
}


static Item* createItem
(const std::string& moduleName, const std::string& className, bool searchOtherModules)
{
    auto p = moduleNameToItemManagerImplMap.find(moduleName);

    if(p == moduleNameToItemManagerImplMap.end()){
        auto alias = PluginManager::instance()->guessActualPluginName(moduleName);
        if(!alias){
            return nullptr;
        }
        p = moduleNameToItemManagerImplMap.find(alias);
    }
    
    Item* item = nullptr;

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
        } else if(searchOtherModules){
            auto r = aliasClassNameToAliasModuleNameToTrueNamePairMap.find(className);
            if(r != aliasClassNameToAliasModuleNameToTrueNamePairMap.end()){
                auto& aliasModuleNameToTrueNamePairMap = r->second;
                auto s = aliasModuleNameToTrueNamePairMap.find(moduleName);
                if(s != aliasModuleNameToTrueNamePairMap.end()){
                    auto& trueNamePair = s->second;
                    item = createItem(trueNamePair.first, trueNamePair.second, false);
                }
            }
        }
    }

    return item;
}


Item* ItemManager::createItem(const std::string& moduleName, const std::string& className)
{
    return ::createItem(moduleName, className, true);
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
        parentItem->insertChild(nextItem, newItem);
    }
    
    return newItem;
}


void ItemManager::addCreationPanel_(const std::type_info& type, ItemCreationPanel* panel)
{
    impl->addCreationPanel(type, panel);
}


void ItemManager::Impl::addCreationPanel(const std::type_info& type, ItemCreationPanel* panel)
{
    CreationPanelBase* base = getOrCreateCreationPanelBase(type);
    if(panel){
        base->addPanel(panel);
    } else {
        base->addPanel(new DefaultItemCreationPanel);
    }
    registeredCreationPanels.insert(panel);
}


void ItemManager::addCreationPanelFilter_
(const std::type_info& type, std::shared_ptr<CreationPanelFilterBase> filter, bool afterInitializionByPanels)
{
    impl->addCreationPanelFilter(type, filter, afterInitializionByPanels);
}


void ItemManager::Impl::addCreationPanelFilter
(const std::type_info& type, shared_ptr<CreationPanelFilterBase> filter, bool afterInitializionByPanels)
{
    CreationPanelBase* base = getOrCreateCreationPanelBase(type);
    if(!afterInitializionByPanels){
        base->preFilters.push_back(filter);
    } else {
        base->postFilters.push_back(filter);
    }
    registeredCreationPanelFilters.insert(make_pair(std::type_index(type), filter));
}


CreationPanelBase* ItemManager::Impl::getOrCreateCreationPanelBase(const std::type_info& type)
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


void ItemManager::Impl::onNewItemActivated(CreationPanelBase* base)
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
        for(auto& panel : panels){
            if(!panel->initializePanel(item, parentItem)){
                result = false;
                break;
            }
            if(!panel->initializePanel(item)){ // old
                result = false;
                break;
            }
        }
    }

    if(result){
        if(exec() == QDialog::Accepted){
            for(auto& panel : panels){
                if(!panel->initializeItem(item, parentItem)){
                    result = false;
                    break;
                }
                if(!panel->initializeItem(item)){ // old
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


ItemCreationPanel::ItemCreationPanel()
{

}


bool ItemCreationPanel::initializePanel(Item* /* protoItem */, Item* /* parentItem */)
{
    return true;
}


bool ItemCreationPanel::initializeItem(Item* /* protoItem */, Item* /* parentItem */)
{
    return true;
}


bool ItemCreationPanel::initializePanel(Item* /* protoItem */)
{
    return true;
}


bool ItemCreationPanel::initializeItem(Item* /* protoItem */)
{
    return true;
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


DefaultItemCreationPanel::DefaultItemCreationPanel()
{
    QHBoxLayout* layout = new QHBoxLayout();
    layout->addWidget(new QLabel(_("Name:")));
    nameEntry = new QLineEdit();
    layout->addWidget(nameEntry);
    setLayout(layout);
}
        

bool DefaultItemCreationPanel::initializePanel(Item* protoItem, Item* /* parentItem */)
{
    static_cast<QLineEdit*>(nameEntry)->setText(protoItem->name().c_str());
    return true;
}
            

bool DefaultItemCreationPanel::initializeItem(Item* protoItem, Item* /* parentItem */)
{
    protoItem->setName(static_cast<QLineEdit*>(nameEntry)->text().toStdString());
    return true;
}


void ItemManager::registerFileIO_(const std::type_info& type, ItemFileIO* fileIO)
{
    impl->registerFileIO(type, fileIO);
}


ClassInfoPtr ItemManager::Impl::registerFileIO(const type_info& type, ItemFileIO* fileIO)
{
    ClassInfoPtr classInfo;
    
    auto p = itemTypeToInfoMap.find(type);
    if(p != itemTypeToInfoMap.end()){
        classInfo = p->second;
        fileIO->setItemClassInfo(classInfo);
        
        registeredFileIOs.insert(fileIO);
        auto interfaceLevel = fileIO->interfaceLevel();
        
        auto p = classInfo->fileIOs.begin();
        while(p != classInfo->fileIOs.end()){
            if((*p)->interfaceLevel() > interfaceLevel){
                break;
            }
            ++p;
        }
        classInfo->fileIOs.insert(p, fileIO);
        
        if(fileIO->hasApi(ItemFileIO::Load)){
            if(interfaceLevel == ItemFileIO::Standard){
                menuManager.setPath("/File/Load ...");
                addLoader(fileIO, captionToStandardLoadersMap);
            } else if(interfaceLevel == ItemFileIO::Conversion){
                menuManager.setPath("/File/Import ...");
                addLoader(fileIO, captionToConversionLoadersMap);
            }
        }
    }

    return classInfo;
}


void ItemManager::Impl::addLoader(ItemFileIO* fileIO, CaptionToFileIoListMap& loaderMap)
{
    auto& caption = fileIO->caption();
    auto& loaders = loaderMap[caption];
    if(loaders.empty()){
        menuManager.addItem(caption)
            ->sigTriggered().connect(
                [this, caption, &loaderMap]() mutable {
                    onLoadOrImportItemsActivated(loaderMap[caption]); });
    }
    loaders.push_back(fileIO);
}


vector<ItemFileIO*> ItemManager::Impl::getFileIOs(Item* item, function<bool(ItemFileIO* fileIO)> pred)
{
    vector<ItemFileIO*> fileIOs;
    auto p = itemTypeToInfoMap.find(typeid(*item));
    if(p != itemTypeToInfoMap.end()){
        auto& classInfo = p->second;
        for(auto& fileIO : classInfo->fileIOs){
            if(pred(fileIO)){
                fileIOs.push_back(fileIO);
            }
        }
    }
    return fileIOs;
}


vector<ItemFileIO*> ItemManager::getFileIOs(const std::type_info& type)
{
    vector<ItemFileIO*> fileIOs;
    auto p = itemTypeToInfoMap.find(type);
    if(p != itemTypeToInfoMap.end()){
        auto& classInfo = p->second;
        for(auto& fileIO : classInfo->fileIOs){
            fileIOs.push_back(fileIO);
        }
    }
    return fileIOs;
}
    

ItemFileIO* ItemManager::findFileIO(const std::type_info& type, const std::string& formatId)
{
    ItemFileIO* found = nullptr;
    auto p = itemTypeToInfoMap.find(type);
    if(p != itemTypeToInfoMap.end()){
        auto& classInfo = p->second;
        for(auto& fileIO : classInfo->fileIOs){
            if(formatId.empty() || fileIO->isFormat(formatId)){
                found = fileIO;
                break;
            }
        }
    }
    return found;
}


ItemFileIO* ItemManager::Impl::findMatchedFileIO
(const type_info& type, const string& filename, const string& formatId, int ioTypeFlag)
{
    ItemFileIO* targetFileIO = nullptr;
    
    auto p = itemTypeToInfoMap.find(type);
    if(p == itemTypeToInfoMap.end()){
        messageView->putln(
            format(_("\"{0}\" cannot be accessed because the specified item type \"{1}\" is not registered."),
                   filename, type.name()),
            MessageView::Error);
        return targetFileIO;;
    }
    
    ClassInfoPtr& classInfo = p->second;
    auto& fileIOs = classInfo->fileIOs;

    if(!formatId.empty() || filename.empty()){
        for(auto& fileIO : fileIOs){
            if(fileIO->hasApi(ioTypeFlag)){
                if(formatId.empty() || fileIO->isFormat(formatId)){
                    targetFileIO = fileIO;
                    break;
                }
            }
        }
    } else if(!filename.empty()){
        filesystem::path filepath(fromUTF8(filename));
        string dotextension = filepath.extension().string();
        if(dotextension.size() >= 2){
            string extension = dotextension.substr(1); // remove dot
            for(auto& fileIO : fileIOs){
                if(fileIO->hasApi(ioTypeFlag)){
                    for(auto& ext : fileIO->extensions()){
                        if(ext == extension){
                            targetFileIO = fileIO;
                            break;
                        }
                    }
                    if(targetFileIO){
                        break;
                    }
                }
            }
        }
    }

    if(!targetFileIO){
        if(formatId.empty()){
            messageView->putln(
                format(_("The file format for accessing \"{0}\" cannot be determined."), filename),
                MessageView::Error);
        } else {
            messageView->putln(
                format(_("Unknown file format \"{0}\" is specified in accessing \"{1}\"."),
                       formatId, filename),
                MessageView::Error);
        }
    }

    return targetFileIO;
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


void ItemManager::addLoader_
(const std::type_info& type, const std::string& caption, const std::string& formatId,
 std::function<std::string()> getExtensions, std::shared_ptr<FileFunctionBase> function, int priority)
{
    auto adapter = new FileFunctionAdapter(
        ItemFileIO::Load, caption, formatId, getExtensions, function, priority);
    auto classInfo = impl->registerFileIO(type, adapter);
    adapter->factory = classInfo->factory;
}


void ItemManager::addSaver_
(const std::type_info& type, const std::string& caption, const std::string& formatId,
 std::function<std::string()> getExtensions, std::shared_ptr<FileFunctionBase> function, int priority)
{
    auto adapter = new FileFunctionAdapter(
        ItemFileIO::Save, caption, formatId, getExtensions, function, priority);
    impl->registerFileIO(type, adapter);
}


bool ItemManager::load
(Item* item, const std::string& filename, Item* parentItem, const std::string& formatId, const Mapping* options)
{
    if(auto fileIO = Impl::findMatchedFileIO(typeid(*item), filename, formatId, ItemFileIO::Load)){
        return fileIO->loadItem(item, filename, parentItem, false, nullptr, options);
    }
    return false;
}


ItemList<Item> ItemManager::loadItemsWithDialog_
(const std::type_info& type, Item* parentItem, bool doAddtion, Item* nextItem)
{
    if(auto fileIO = Impl::findMatchedFileIO(type, "", "", ItemFileIO::Load)){
        ItemFileDialog dialog;
        dialog.setFileIO(fileIO);
        return dialog.loadItems(parentItem, doAddtion, nextItem);
    }
    return ItemList<Item>();
}


void ItemManager::Impl::onLoadOrImportItemsActivated(const vector<ItemFileIO*>& fileIOs)
{
    Item* parentItem = RootItem::instance()->selectedItems().toSingle();
    if(!parentItem){
        parentItem = RootItem::instance();
    }
    ItemFileDialog dialog;
    dialog.setFileIOs(fileIOs);
    dialog.loadItems(parentItem, true);
}


void ItemManager::reloadItems(const ItemList<>& items)
{
    reloadedItemToOriginalItemMap.clear();
    
    for(size_t i=0; i < items.size(); ++i){

        Item* item = items.get(i);

        if(item->parentItem() && !item->isSubItem() &&
           !item->filePath().empty() && !item->fileFormat().empty()){

            ItemPtr reloadedItem = item->duplicate();
            if(reloadedItem){
                bool reloaded = reloadedItem->load(
                    item->filePath(), item->parentItem(), item->fileFormat(), item->fileOptions());
                if(reloaded){
                    reloadedItemToOriginalItemMap[reloadedItem] = item;

                    item->parentItem()->insertChild(item, reloadedItem);
                    
                    // move children to the reload item
                    ItemPtr child = item->childItem();
                    while(child){
                        ItemPtr nextChild = child->nextItem();
                        if(!child->isSubItem()){
                            child->removeFromParentItem();
                            reloadedItem->addChildItem(child);
                        }
                        child = nextChild;
                    }
                    reloadedItem->assign(item);

                    item->removeFromParentItem();
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


void ItemManager::Impl::onReloadSelectedItemsActivated()
{
    reloadItems(RootItem::instance()->selectedItems());
}


bool ItemManager::save
(Item* item, const std::string& filename, const std::string& formatId, const Mapping* options)
{
    if(auto fileIO = Impl::findMatchedFileIO(typeid(*item), filename, formatId, ItemFileIO::Save)){
        return fileIO->saveItem(item, filename, options);
    }
    return false;
}


bool ItemManager::saveItemWithDialog_(const std::type_info& type, Item* item)
{
    if(auto fileIO = Impl::findMatchedFileIO(type, "", "", ItemFileIO::Save)){
        ItemFileDialog dialog;
        dialog.setFileIO(fileIO);
        return dialog.saveItem(item);
    }
    return false;
}


void ItemManager::Impl::onSaveSelectedItemsAsActivated()
{
    ItemFileDialog dialog;
    for(auto& item : RootItem::instance()->selectedItems()){
        dialog.setFileIOs(
            getFileIOs(
                item,
                [](ItemFileIO* fileIO){
                    return (fileIO->hasApi(ItemFileIO::Save) &&
                            fileIO->interfaceLevel() == ItemFileIO::Standard);
                }));
        dialog.saveItem(item);
    }
}


void ItemManager::Impl::onExportSelectedItemsActivated()
{
    ItemFileDialog dialog;
    dialog.setExportMode();
    for(auto& item : RootItem::instance()->selectedItems()){
        dialog.setFileIOs(
            getFileIOs(
                item,
                [](ItemFileIO* fileIO){
                    return (fileIO->hasApi(ItemFileIO::Save) &&
                            fileIO->interfaceLevel() == ItemFileIO::Conversion);
                }));
        dialog.saveItem(item);
    }
}


bool ItemManager::overwrite(Item* item, bool forceOverwrite, const std::string& formatId)
{
    bool needToOverwrite = forceOverwrite;

    string filename(item->filePath());
    string lastFormatId(item->fileFormat());

    if(!formatId.empty() && formatId != lastFormatId){
        needToOverwrite = true;
    } else {
        if(!filename.empty()){
            filesystem::path fpath(fromUTF8(filename));
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
            synchronized = save(item, filename, lastFormatId, item->fileOptions());
        } 
        if(!synchronized){
            auto fileIOs =
                Impl::getFileIOs(
                    item,
                    [&](ItemFileIO* fileIO){
                        return (fileIO->hasApi(ItemFileIO::Save) &&
                                fileIO->interfaceLevel() == ItemFileIO::Standard &&
                                (formatId.empty() || fileIO->isFormat(formatId)));
                    });
            
            ItemFileDialog dialog;
            dialog.setFileIOs(fileIOs);
            synchronized = dialog.saveItem(item);
        }
    }

    return synchronized;
}


void ItemManager::Impl::onSaveSelectedItemsActivated()
{
    for(auto& item : RootItem::instance()->selectedItems()){
        overwrite(item, true, "");
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
        ItemFileDialog::makeNameFilter(
            caption, ItemFileIO::separateExtensions(extensions));
    
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
