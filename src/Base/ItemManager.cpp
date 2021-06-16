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

namespace {

class CreationDialog;

struct ClassInfo : public Referenced
{
    ItemManager::Impl* manager;
    string className;
    string name; // without the 'Item' suffix
    function<Item*()> factory;
    vector<CreationDialog*> creationDialogs;
    vector<ItemFileIOPtr> fileIOs;
    ItemPtr singletonInstance;
    bool isSingleton;

    ClassInfo();
    ~ClassInfo();
};
typedef ref_ptr<ClassInfo> ClassInfoPtr;
    
typedef map<int, ClassInfoPtr> ItemClassIdToInfoMap;
typedef map<string, ClassInfoPtr> ItemClassNameToInfoMap;

ItemClassRegistry* itemClassRegistry = nullptr;
MessageView* messageView = nullptr;
bool isStaticMembersInitialized = false;

ItemClassIdToInfoMap itemClassIdToInfoMap;

typedef map<string, ItemManager::Impl*> ModuleNameToItemManagerImplMap;
ModuleNameToItemManagerImplMap moduleNameToItemManagerImplMap;

map<string, map<string, pair<string, string>>> aliasClassNameToAliasModuleNameToTrueNamePairMap;

typedef map<string, vector<ItemFileIO*>> CaptionToFileIoListMap;
CaptionToFileIoListMap captionToStandardLoadersMap;
CaptionToFileIoListMap captionToConversionLoadersMap;
    
QWidget* importMenu;

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
    set<int> registeredItemClassIds;
    set<std::type_index> registeredAddonTypes;
    set<ItemCreationPanel*> registeredCreationPanels;
    set<ItemFileIOPtr> registeredFileIOs;
    
    QSignalMapper* mapperForNewItemActivated;
    QSignalMapper* mapperForLoadSpecificTypeItemActivated;

    AddonNameToInfoMap addonNameToInfoMap;

    void detachManagedTypeItems(Item* parentItem);
        
    void registerClass(
        function<Item*()>& factory, Item* singletonInstance, int classId, const string& className);

    void addCreationPanel(const std::type_info& type, ItemCreationPanel* panel);
    CreationDialog* createCreationDialog(const std::type_info& type);
    static void onNewItemActivated(CreationDialog* dialog);

    ClassInfoPtr registerFileIO(const type_info& typeId, ItemFileIO* fileIO);
    void addLoader(ItemFileIO* fileIO, CaptionToFileIoListMap& loaderMap);
    static vector<ItemFileIO*> getFileIOs(Item* item, function<bool(ItemFileIO* fileIO)> pred, bool includeSuperClassIos);
    static ItemFileIO* findMatchedFileIO(
        const type_info& type, const string& filename, const string& format, int ioTypeFlag);
    static void onLoadOrImportItemsActivated(const vector<ItemFileIO*>& fileIOs);
    static void onReloadSelectedItemsActivated();
    static void onSaveSelectedItemsAsActivated();
    static void onExportSelectedItemsActivated();
    static void onSaveSelectedItemsActivated();
};

}

namespace {

class CreationDialog : public QDialog
{
public:
    CreationDialog(const QString& title, ClassInfo* classInfo, Item* singletonInstance);
    void addPanel(ItemCreationPanel* panel);
    Item* createItem(Item* parentItem, Item* protoItem = nullptr);
    ClassInfo* classInfo;
    ItemCreationPanel* creationPanel;
    QVBoxLayout* panelLayout;
    ItemPtr defaultProtoItem;
    bool isSingleton;
};

ClassInfo::ClassInfo()
{

}

ClassInfo::~ClassInfo()
{
    for(auto& dialog : creationDialogs){
        delete dialog;
    }
}

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
    for(auto& panel : registeredCreationPanels){
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
    for(auto& id : registeredItemClassIds){
        itemClassIdToInfoMap.erase(id);
    }
    for(auto& type : registeredAddonTypes){
        addonTypeToInfoMap.erase(type);
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
        if(registeredItemClassIds.find(item->classId()) != registeredItemClassIds.end()){
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
    int classId = itemClassRegistry->registerClassAsTypeInfo(type, superType);

    if(factory || singletonInstance){
        impl->registerClass(factory, singletonInstance, classId, className);
    }
}


void ItemManager::Impl::registerClass
(std::function<Item*()>& factory, Item* singletonInstance, int classId, const string& className)
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

    registeredItemClassIds.insert(classId);
    itemClassIdToInfoMap[classId] = info;
}


void ItemManager::addAlias_
(const std::type_info& type, const std::string& aliasClassName, const std::string& aliasModuleName)
{
    auto p = itemClassIdToInfoMap.find(itemClassRegistry->classId(type));
    if(p != itemClassIdToInfoMap.end()){
        auto classInfo =  p->second;
        aliasClassNameToAliasModuleNameToTrueNamePairMap[aliasClassName][aliasModuleName] =
            make_pair(classInfo->manager->moduleName, classInfo->className);
    }
}


bool ItemManager::getClassIdentifier(Item* item, std::string& out_moduleName, std::string& out_className)
{
    bool result;

    auto p = itemClassIdToInfoMap.find(item->classId());
    if(p != itemClassIdToInfoMap.end()){
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
    auto p = itemClassIdToInfoMap.find(itemClassRegistry->classId(type));
    if(p != itemClassIdToInfoMap.end()){
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
        if(auto alias = PluginManager::instance()->guessActualPluginName(moduleName)){
            p = moduleNameToItemManagerImplMap.find(alias);
        }
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
        }
    }

    if(searchOtherModules){
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

    return item;
}


Item* ItemManager::createItem(const std::string& moduleName, const std::string& className)
{
    return ::createItem(moduleName, className, true);
}


Item* ItemManager::createItem(int itemClassId)
{
    auto p = itemClassIdToInfoMap.find(itemClassId);
    if(p != itemClassIdToInfoMap.end()){
        auto classInfo = p->second;
        if(classInfo->factory){
            return classInfo->factory();
        }
    }
    return nullptr;
}


Item* ItemManager::createItemWithDialog_
(const std::type_info& type, Item* parentItem, bool doAddition, Item* nextItem, Item* protoItem, const std::string& title)
{
    Item* newItem = nullptr;
    
    auto iter = itemClassIdToInfoMap.find(itemClassRegistry->classId(type));
    if(iter == itemClassIdToInfoMap.end()){
        showWarningDialog(fmt::format(_("Class {} is not registered as an item class."), type.name()));

    } else {
        auto& info = iter->second;
        CreationDialog* dialog = nullptr;
        if(!info->creationDialogs.empty()){
            dialog = info->creationDialogs.front();
        }
        if(!dialog){
            showWarningDialog(fmt::format(_("The panel to create {} is not registered."), info->className));
        } else {
            if(!parentItem){
                parentItem = RootItem::instance();
            }
            QString orgTitle;
            if(!title.empty()){
                orgTitle = dialog->windowTitle();
                dialog->setWindowTitle(title.c_str());
            }
            newItem = dialog->createItem(parentItem, protoItem);
            if(!orgTitle.isEmpty()){
                dialog->setWindowTitle(orgTitle);
            }
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
    CreationDialog* dialog = createCreationDialog(type);
    if(panel){
        dialog->addPanel(panel);
    } else {
        dialog->addPanel(new DefaultItemCreationPanel);
    }
    registeredCreationPanels.insert(panel);
}


CreationDialog* ItemManager::Impl::createCreationDialog(const std::type_info& type)
{
    CreationDialog* dialog = nullptr;
    
    auto p = itemClassIdToInfoMap.find(itemClassRegistry->classId(type));
    if(p != itemClassIdToInfoMap.end()){
        auto& info = p->second;
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
        QString title(QString(_("Create New %1")).arg(translatedName));
        dialog = new CreationDialog(title, info, info->singletonInstance);
        dialog->hide();
        menuManager.setPath("/File/New ...").addItem(translatedName)
            ->sigTriggered().connect([=](){ onNewItemActivated(dialog); });
        info->creationDialogs.push_back(dialog);
    }

    return dialog;
}


void ItemManager::Impl::onNewItemActivated(CreationDialog* dialog)
{
    ItemList<Item> parentItems = RootItem::instance()->selectedItems();

    if(parentItems.empty()){
        parentItems.push_back(RootItem::instance());
    }
    for(size_t i=0; i < parentItems.size(); ++i){
        auto parentItem = parentItems[i];
        auto newItem = dialog->createItem(parentItem);
        if(newItem){
            parentItem->addChildItem(newItem, true);
        }
    }
}


namespace {

CreationDialog::CreationDialog
(const QString& title, ClassInfo* classInfo, Item* singletonInstance)
    : QDialog(MainWindow::instance()),
      classInfo(classInfo),
      defaultProtoItem(singletonInstance),
      isSingleton((bool)singletonInstance)
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

    QVBoxLayout* topLayout = new QVBoxLayout;
    panelLayout = new QVBoxLayout;
    topLayout->addLayout(panelLayout);
    topLayout->addWidget(buttonBox);
    setLayout(topLayout);
}


void CreationDialog::addPanel(ItemCreationPanel* panel)
{
    creationPanel = panel;
    panelLayout->addWidget(panel);
}


Item* CreationDialog::createItem(Item* parentItem, Item* protoItem)
{
    if(!protoItem){
        protoItem = defaultProtoItem;
    }
    if(isSingleton){
        if(protoItem->parentItem()){
            return nullptr;
        }
    }
    if(!protoItem){
        defaultProtoItem = classInfo->factory();
        defaultProtoItem->setName(classInfo->name);
        protoItem = defaultProtoItem;
    }
    ItemPtr newInstance;
    if(creationPanel->initializeCreation(protoItem, parentItem)){
        if(exec() == QDialog::Accepted){
            if(creationPanel->updateItem(protoItem, parentItem)){
                if((protoItem == defaultProtoItem) && !isSingleton){
                    newInstance = protoItem->duplicate();
                } else {
                    newInstance = protoItem;
                }
            }
        }
    }
    return newInstance.retn();
}

}


ItemCreationPanel::ItemCreationPanel()
{

}


DefaultItemCreationPanel::DefaultItemCreationPanel()
{
    QHBoxLayout* layout = new QHBoxLayout();
    layout->addWidget(new QLabel(_("Name:")));
    nameEntry = new QLineEdit();
    layout->addWidget(nameEntry);
    setLayout(layout);
}
        

bool DefaultItemCreationPanel::initializeCreation(Item* protoItem, Item* /* parentItem */)
{
    static_cast<QLineEdit*>(nameEntry)->setText(protoItem->name().c_str());
    return true;
}
            

bool DefaultItemCreationPanel::updateItem(Item* protoItem, Item* /* parentItem */)
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
    
    auto p = itemClassIdToInfoMap.find(itemClassRegistry->classId(type));
    if(p != itemClassIdToInfoMap.end()){
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


vector<ItemFileIO*> ItemManager::Impl::getFileIOs
(Item* item, function<bool(ItemFileIO* fileIO)> pred, bool includeSuperClassIos)
{
    vector<ItemFileIO*> fileIOs;

    int classId = item->classId();
    while(classId > 0){
        auto p = itemClassIdToInfoMap.find(classId);
        if(p != itemClassIdToInfoMap.end()){
            auto& classInfo = p->second;
            for(auto& fileIO : classInfo->fileIOs){
                if(pred(fileIO)){
                    fileIOs.push_back(fileIO);
                }
            }
        }
        if(includeSuperClassIos){
            classId = itemClassRegistry->superClassId(classId);
        } else {
            classId = 0;
        }
    }
    return fileIOs;
}


vector<ItemFileIO*> ItemManager::getFileIOs(const std::type_info& type)
{
    vector<ItemFileIO*> fileIOs;
    auto p = itemClassIdToInfoMap.find(itemClassRegistry->classId(type));
    if(p != itemClassIdToInfoMap.end()){
        auto& classInfo = p->second;
        for(auto& fileIO : classInfo->fileIOs){
            fileIOs.push_back(fileIO);
        }
    }
    return fileIOs;
}
    

ItemFileIO* ItemManager::findFileIO(const std::type_info& type, const std::string& format)
{
    ItemFileIO* found = nullptr;
    auto p = itemClassIdToInfoMap.find(itemClassRegistry->classId(type));
    if(p != itemClassIdToInfoMap.end()){
        auto& classInfo = p->second;
        for(auto& fileIO : classInfo->fileIOs){
            if(format.empty() || fileIO->isFormat(format)){
                found = fileIO;
                break;
            }
        }
    }
    return found;
}


ItemFileIO* ItemManager::Impl::findMatchedFileIO
(const type_info& type, const string& filename, const string& format, int ioTypeFlag)
{
    ItemFileIO* targetFileIO = nullptr;
    
    auto p = itemClassIdToInfoMap.find(itemClassRegistry->classId(type));
    if(p == itemClassIdToInfoMap.end()){
        messageView->putln(
            fmt::format(_("\"{0}\" cannot be accessed because the specified item type \"{1}\" is not registered."),
                        filename, type.name()),
            MessageView::Error);
        return targetFileIO;;
    }
    
    ClassInfoPtr& classInfo = p->second;
    auto& fileIOs = classInfo->fileIOs;

    if(!format.empty() || filename.empty()){
        for(auto& fileIO : fileIOs){
            if(fileIO->hasApi(ioTypeFlag)){
                if(format.empty() || fileIO->isFormat(format)){
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
        if(format.empty()){
            messageView->putln(
                fmt::format(_("The file format for accessing \"{0}\" cannot be determined."), filename),
                MessageView::Error);
        } else {
            messageView->putln(
                fmt::format(_("Unknown file format \"{0}\" is specified in accessing \"{1}\"."),
                            format, filename),
                MessageView::Error);
        }
    }

    return targetFileIO;
}


namespace {

// The following adapter class is defined to use existing loaders and savers
// based on FileFunctionBase for the backward compatiblity
class FileFunctionAdapter : public ItemFileIO
{
public:
    std::shared_ptr<ItemManager::FileFunctionBase> fileFunction;
    function<Item*()> factory;
    
    FileFunctionAdapter(
        int api, const std::string& caption, const std::string& format,
        const std::function<std::string()>& getExtensions, std::shared_ptr<ItemManager::FileFunctionBase> function,
        int usage)
        : ItemFileIO(format, api),
          fileFunction(function)
    {
        setCaption(caption);
        setExtensionFunction(getExtensions);
        if(usage >= ItemManager::Standard){
            setInterfaceLevel(Standard);
        } else if(usage >= ItemManager::Conversion){
            setInterfaceLevel(Conversion);
        } else {
            setInterfaceLevel(Internal);
        }
    }

    virtual Item* createItem() override
    {
        if(factory){
            return factory();
        } else if(isRegisteredForSingletonItem()){
            return findSingletonItemInstance();
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
(const std::type_info& type, const std::string& caption, const std::string& format,
 std::function<std::string()> getExtensions, std::shared_ptr<FileFunctionBase> function, int usage)
{
    auto adapter = new FileFunctionAdapter(
        ItemFileIO::Load, caption, format, getExtensions, function, usage);
    auto classInfo = impl->registerFileIO(type, adapter);
    adapter->factory = classInfo->factory;
}


void ItemManager::addSaver_
(const std::type_info& type, const std::string& caption, const std::string& format,
 std::function<std::string()> getExtensions, std::shared_ptr<FileFunctionBase> function, int usage)
{
    auto adapter = new FileFunctionAdapter(
        ItemFileIO::Save, caption, format, getExtensions, function, usage);
    impl->registerFileIO(type, adapter);
}


bool ItemManager::load
(Item* item, const std::string& filename, Item* parentItem, const std::string& format, const Mapping* options)
{
    if(auto fileIO = Impl::findMatchedFileIO(typeid(*item), filename, format, ItemFileIO::Load)){
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
    for(auto& item : items){
        item->reload();
    }
}


Item* ItemManager::findOriginalItemForReloadedItem(Item* item)
{
    return item->findOriginalItem();
}


void ItemManager::Impl::onReloadSelectedItemsActivated()
{
    for(auto& item : RootItem::instance()->selectedItems()){
        item->reload();
    }
}


bool ItemManager::save
(Item* item, const std::string& filename, const std::string& format, const Mapping* options)
{
    if(auto fileIO = Impl::findMatchedFileIO(typeid(*item), filename, format, ItemFileIO::Save)){
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
                },
                false));
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
                },
                true));
        dialog.saveItem(item);
    }
}


bool ItemManager::overwrite(Item* item, bool forceOverwrite, const std::string& format)
{
    bool needToOverwrite = forceOverwrite;

    string filename(item->filePath());
    string lastFormat(item->fileFormat());

    if(!format.empty() && format != lastFormat){
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
        if(!filename.empty() && format.empty()){
            synchronized = save(item, filename, lastFormat, item->fileOptions());
        } 
        if(!synchronized){
            auto fileIOs =
                Impl::getFileIOs(
                    item,
                    [&](ItemFileIO* fileIO){
                        return (fileIO->hasApi(ItemFileIO::Save) &&
                                fileIO->interfaceLevel() == ItemFileIO::Standard &&
                                (format.empty() || fileIO->isFormat(format)));
                    },
                    false);
            
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
    impl->registeredAddonTypes.insert(type);
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
