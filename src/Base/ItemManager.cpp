#include "ItemManager.h"
#include "Item.h"
#include "ItemAddon.h"
#include "RootItem.h"
#include "ItemClassRegistry.h"
#include "PluginManager.h"
#include "ItemFileIO.h"
#include "ItemFileDialog.h"
#include "FileDialog.h"
#include "MainWindow.h"
#include "MessageView.h"
#include "MainMenu.h"
#include "Action.h"
#include "CheckBox.h"
#include <cnoid/ExecutablePath>
#include <cnoid/UTF8>
#include <cnoid/Format>
#include <cnoid/stdx/filesystem>
#include <QLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QDialog>
#include <QDialogButtonBox>
#include <QSignalMapper>
#include <chrono>
#include <set>
#include <sstream>
#include <typeindex>
#include <regex>
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

ItemClassIdToInfoMap itemClassIdToInfoMap;

typedef map<string, ItemManager::Impl*> ModuleNameToItemManagerImplMap;
ModuleNameToItemManagerImplMap moduleNameToItemManagerImplMap;

map<string, map<string, pair<string, string>>> aliasClassNameToAliasModuleNameToTrueNamePairMap;

typedef map<string, vector<ItemFileIO*>> CaptionToFileIoListMap;
CaptionToFileIoListMap captionToStandardLoadersMap;
CaptionToFileIoListMap captionToConversionLoadersMap;
    
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
    Impl(const string& moduleName);
    ~Impl();

    string moduleName;
    string textDomain;
    MainMenu* mainMenu;

    ItemClassNameToInfoMap itemClassNameToInfoMap;
    set<int> registeredItemClassIds;
    set<std::type_index> registeredAddonTypes;
    set<ItemCreationPanel*> registeredCreationPanels;
    set<ItemFileIOPtr> registeredFileIOs;

    bool hasLoaders;
    bool hasImporters;

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
    void addLoader(ItemFileIO* fileIO, CaptionToFileIoListMap& loaderMap, bool isImporter);
    static ItemFileIO* findMatchedFileIO(
        const type_info& type, const string& filename, const string& format, int ioTypeFlag);
    static void onLoadOrImportItemsActivated(const vector<ItemFileIO*>& fileIOs);
};

}

namespace {

class CreationDialog : public QDialog
{
public:
    CreationDialog(const QString& title, ClassInfo* classInfo, Item* singletonInstance);
    void addPanel(ItemCreationPanel* panel);
    Item* createItem(Item* parentItem, Item* protoItem = nullptr);
    Item* getOrCreateDefaultProtoItem();
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


void ItemManager::initializeClass(ExtensionManager* ext)
{
    itemClassRegistry = &ItemClassRegistry::instance();
    messageView = MessageView::instance();
}


ItemManager::ItemManager(const std::string& moduleName)
{
    impl = new Impl(moduleName);
}


ItemManager::Impl::Impl(const string& moduleName)
    : moduleName(moduleName),
      mainMenu(MainMenu::instance())
{
    moduleNameToItemManagerImplMap[moduleName] = this;
    hasLoaders = false;
    hasImporters = false;
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
        static std::regex pattern("Item$");
        info->name = regex_replace(className, pattern, "");
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
    auto p = itemClassIdToInfoMap.find(itemClassRegistry->getClassId(type));
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
    Item* item = nullptr;

    auto p = moduleNameToItemManagerImplMap.find(moduleName);
    if(p == moduleNameToItemManagerImplMap.end()){
        if(auto alias = PluginManager::instance()->guessActualPluginName(moduleName)){
            p = moduleNameToItemManagerImplMap.find(alias);
        }
    }
    if(p != moduleNameToItemManagerImplMap.end()){
        auto& itemClassNameToInfoMap = p->second->itemClassNameToInfoMap;
        auto q = itemClassNameToInfoMap.find(className);
        if(q != itemClassNameToInfoMap.end()){
            auto& info = q->second;
            if(!info->isSingleton){
                if(info->factory){
                    item = info->factory();
                }
            } else {
                auto instance = info->singletonInstance;
                if(!instance->parentItem()){
                    item = instance;
                } else {
                    if(!instance->isConnectedToRoot()){
                        instance->removeFromParentItem();
                        instance->clearNonSubItemChildren();
                        item = instance;
                    } else {
                        messageView->putln(
                            formatR(_("{0} is a singleton item type and its instance exists in the project item tree."),
                                    info->className),
                            MessageView::Warning);
                        searchOtherModules = false;
                    }
                }
            }
        }
    }
    if(!item && searchOtherModules){
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
    
    auto iter = itemClassIdToInfoMap.find(itemClassRegistry->getClassId(type));
    if(iter == itemClassIdToInfoMap.end()){
        showWarningDialog(formatR(_("Class {} is not registered as an item class."), type.name()));

    } else {
        auto& info = iter->second;
        CreationDialog* dialog = nullptr;
        if(!info->creationDialogs.empty()){
            dialog = info->creationDialogs.front();
        }
        if(!dialog){
            showWarningDialog(formatR(_("The panel to create {} is not registered."), info->className));
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
    if(dialog){
        if(panel){
            dialog->addPanel(panel);
        } else {
            dialog->addPanel(new DefaultItemCreationPanel);
        }
    }
    
    registeredCreationPanels.insert(panel);
}


CreationDialog* ItemManager::Impl::createCreationDialog(const std::type_info& type)
{
    CreationDialog* dialog = nullptr;
    
    auto p = itemClassIdToInfoMap.find(itemClassRegistry->getClassId(type));
    if(p != itemClassIdToInfoMap.end()){
        auto& info = p->second;
        const char* className_c_str = info->className.c_str();
        const char* translatedClassName_c_str = dgettext(textDomain.c_str(), className_c_str);
        string translatedName;
        if(translatedClassName_c_str == className_c_str){
            static regex itemSuffix("Item$");
            translatedName = regex_replace(translatedClassName_c_str, itemSuffix, "");
        } else {
            static regex itemSuffix(_("Item$"));
            translatedName = regex_replace(translatedClassName_c_str, itemSuffix, "");
        }
        QString title(QString(_("Create New %1")).arg(translatedName.c_str()));
        dialog = new CreationDialog(title, info, info->singletonInstance);
        dialog->hide();
        info->creationDialogs.push_back(dialog);

        mainMenu->add_File_New_Item(
            translatedName,
            [=](){ onNewItemActivated(dialog); },
            registeredCreationPanels.empty());
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
            if(!protoItem->isConnectedToRoot()){
                protoItem->removeFromParentItem();
                protoItem->clearNonSubItemChildren();
            } else {
                showWarningDialog(
                    formatR(_("{0} is a singleton item type and its instance exists in the project item tree."),
                            classInfo->className));
                return nullptr;
            }
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
                    newInstance = protoItem->clone();
                } else {
                    newInstance = protoItem;
                }
            }
        }
    }
    return newInstance.retn();
}


Item* CreationDialog::getOrCreateDefaultProtoItem()
{
    if(isSingleton){
        if(!defaultProtoItem || !defaultProtoItem->parentItem()){
            return defaultProtoItem;
        }
    }
    if(!defaultProtoItem){
        defaultProtoItem = classInfo->factory();
        defaultProtoItem->setName(classInfo->name);
    }
    return defaultProtoItem;
}

}


ItemCreationPanel::ItemCreationPanel()
{

}


DefaultItemCreationPanel::DefaultItemCreationPanel()
{
    QHBoxLayout* layout = new QHBoxLayout;
    layout->addWidget(new QLabel(_("Name:")));
    nameEntry = new QLineEdit;
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


Item* ItemManager::getPrototypeInstance_(const std::type_info& type)
{
    auto p = itemClassIdToInfoMap.find(itemClassRegistry->getClassId(type));
    if(p != itemClassIdToInfoMap.end()){
        auto& info = p->second;
        if(!info->creationDialogs.empty()){
            return info->creationDialogs.front()->getOrCreateDefaultProtoItem();
        }
    }
    return nullptr;
}


void ItemManager::registerFileIO_(const std::type_info& type, ItemFileIO* fileIO)
{
    impl->registerFileIO(type, fileIO);
}


ClassInfoPtr ItemManager::Impl::registerFileIO(const type_info& type, ItemFileIO* fileIO)
{
    ClassInfoPtr classInfo;
    
    auto p = itemClassIdToInfoMap.find(itemClassRegistry->getClassId(type));
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
                addLoader(fileIO, captionToStandardLoadersMap, false);
            } else if(interfaceLevel == ItemFileIO::Conversion){
                addLoader(fileIO, captionToConversionLoadersMap, true);
            }
        }
    }

    return classInfo;
}


void ItemManager::Impl::addLoader(ItemFileIO* fileIO, CaptionToFileIoListMap& loaderMap, bool isImporter)
{
    auto& caption = fileIO->caption();
    auto& loaders = loaderMap[caption];
    if(loaders.empty()){
        auto handler =
            [this, caption, &loaderMap]() mutable {
                onLoadOrImportItemsActivated(loaderMap[caption]); };
        if(!isImporter){
            mainMenu->add_File_Load_Item(caption, handler, !hasLoaders);
            hasLoaders = true;
        } else {
            mainMenu->add_File_Import_Item(caption, handler, !hasImporters);
            hasImporters = true;
        }
    }
    loaders.push_back(fileIO);
}


std::vector<ItemFileIO*> ItemManager::getFileIOs
(const Item* item, std:: function<bool(ItemFileIO* fileIO)> pred, bool includeSuperClassIos)
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
            classId = itemClassRegistry->getSuperClassId(classId);
        } else {
            classId = 0;
        }
    }
    return fileIOs;
}


vector<ItemFileIO*> ItemManager::getFileIOs(const std::type_info& type)
{
    vector<ItemFileIO*> fileIOs;
    auto p = itemClassIdToInfoMap.find(itemClassRegistry->getClassId(type));
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
    auto p = itemClassIdToInfoMap.find(itemClassRegistry->getClassId(type));
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
    
    auto p = itemClassIdToInfoMap.find(itemClassRegistry->getClassId(type));
    if(p == itemClassIdToInfoMap.end()){
        if(filename.empty()){
            messageView->putln(
                formatR(_("There is no file I/O processor registered for the \"{0}\" type."), type.name()),
                MessageView::Error);
        } else {
            messageView->putln(
                formatR(_("\"{0}\" cannot be accessed because there is no file I/O processor registered for the \"{1}\" type."),
                            filename, type.name()),
                MessageView::Error);
        }
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
                    for(auto& ext : fileIO->extensions(ioTypeFlag)){
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
                formatR(_("The file format for accessing \"{0}\" cannot be determined."), filename),
                MessageView::Error);
        } else {
            messageView->putln(
                formatR(_("Unknown file format \"{0}\" is specified in accessing \"{1}\"."), format, filename),
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
        const std::string& extensions, std::shared_ptr<ItemManager::FileFunctionBase> function,
        int usage)
        : ItemFileIO(format, api),
          fileFunction(function)
    {
        setCaption(caption);

        if(api == ItemFileIO::Load){
            setExtensionsForLoading(separateExtensions(extensions));
        } else if(api == ItemFileIO::Save){
            setExtensionsForSaving(separateExtensions(extensions));
            setItemNameUpdateInSavingEnabled(true);
        }

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
 const std::string& extensions, std::shared_ptr<FileFunctionBase> function, int usage)
{
    auto adapter = new FileFunctionAdapter(
        ItemFileIO::Load, caption, format, extensions, function, usage);
    auto classInfo = impl->registerFileIO(type, adapter);
    adapter->factory = classInfo->factory;
}


void ItemManager::addSaver_
(const std::type_info& type, const std::string& caption, const std::string& format,
 const std::string& extensions, std::shared_ptr<FileFunctionBase> function, int usage)
{
    auto adapter = new FileFunctionAdapter(
        ItemFileIO::Save, caption, format, extensions, function, usage);
    impl->registerFileIO(type, adapter);
}


bool ItemManager::loadItem
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


bool ItemManager::saveItem
(Item* item, const std::string& filename, const std::string& format, const Mapping* options)
{
    if(auto fileIO = Impl::findMatchedFileIO(typeid(*item), filename, format, ItemFileIO::Save)){
        return fileIO->saveItem(item, filename, options);
    }
    return false;
}


static bool checkFileImmutable(Item* item)
{
    bool doContinue = true;
    if(item->hasAttribute(Item::FileImmutable)){
        doContinue = showWarningDialog(
            formatR(_("\"{0}\" is an item that usually does not need to be saved. "
                      "Do you really want to save this item?"),
                    item->displayName()),
            true);
    }
    return doContinue;
}


bool ItemManager::saveItemWithDialog(Item* item, const std::string& format, bool doCheckFileImmutable)
{
    bool saved = false;
    bool doSave = true;

    if(doCheckFileImmutable){
        if(!checkFileImmutable(item)){
            return false;
        }
    }
    
    if(doSave){
        auto fileIOs =
            getFileIOs(
                item,
                [&](ItemFileIO* fileIO){
                    return (fileIO->hasApi(ItemFileIO::Save) &&
                            fileIO->interfaceLevel() == ItemFileIO::Standard &&
                            (format.empty() || fileIO->isFormat(format)));
                },
                false);
        
        ItemFileDialog dialog;
        dialog.setFileIOs(fileIOs);
        saved = dialog.saveItem(item);
    }

    return saved;
}


bool ItemManager::overwriteItem
(Item* item, bool forceOverwrite, const std::string& format, bool doSaveItemWithDialog)
{
    if(doSaveItemWithDialog){
        if(!checkFileImmutable(item)){
            return false;
        }
    }
        
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
            synchronized = saveItem(item, filename, lastFormat, item->fileOptions());
        } 
        if(!synchronized && doSaveItemWithDialog){
            synchronized = saveItemWithDialog(item, format, false);
        }
    }

    return synchronized;
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


bool ItemManager::getAddonIdentifier(const ItemAddon* addon, std::string& out_moduleName, std::string& out_addonName)
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
    dialog.updatePresetDirectories(true);
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
    dialog.updatePresetDirectories(true);
    if(dialog.exec() == QDialog::Accepted){
        for(auto& file : dialog.selectedFiles()){
            filenames.push_back(file.toStdString());
        }
    }
    return filenames;
}

}
