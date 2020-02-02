/**
   @author Shin'ichiro Nakaoka
*/

#include "ItemManager.h"
#include "Item.h"
#include "RootItem.h"
#include "ItemClassRegistry.h"
#include "ItemFileIO.h"
#include "ItemFileIOBaseImpl.h"
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
#include <QFileDialog>
#include <QStyle>
#include <QSignalMapper>
#include <QRegExp>
#include <fmt/format.h>
#include <chrono>
#include <set>
#include <sstream>
#include "gettext.h"

using namespace std;
using namespace cnoid;
namespace filesystem = cnoid::stdx::filesystem;
using fmt::format;

namespace cnoid {

class ItemManagerImpl
{
public:
    ItemManagerImpl(const string& moduleName, MenuManager& menuManager);
    ~ItemManagerImpl();

    class CreationPanelBase;

    struct ClassInfo : public Referenced
    {
        ClassInfo() { creationPanelBase = nullptr; }
        ~ClassInfo() { delete creationPanelBase; }
        string moduleName;
        string className;
        string name; // without the 'Item' suffix
        function<Item*()> factory;
        CreationPanelBase* creationPanelBase;
        vector<ItemFileIOBasePtr> fileIOs;
        ItemPtr singletonInstance;
        bool isSingleton;
    };
    typedef ref_ptr<ClassInfo> ClassInfoPtr;
    
    typedef map<string, ClassInfoPtr> ClassInfoMap;

    typedef list<shared_ptr<ItemManager::CreationPanelFilterBase>> CreationPanelFilterList;
    typedef set<pair<string, shared_ptr<ItemManager::CreationPanelFilterBase>>> CreationPanelFilterSet;
    
    class CreationPanelBase : public QDialog
    {
    public:
        CreationPanelBase(const QString& title, ClassInfo* classInfo, ItemPtr protoItem, bool isSingleton);
        void addPanel(ItemCreationPanel* panel);
        Item* createItem(Item* parentItem);
        CreationPanelFilterList preFilters;
        CreationPanelFilterList postFilters;
    private:
        ClassInfo* classInfo;
        QVBoxLayout* panelLayout;
        ItemPtr protoItem;
        bool isSingleton;
    };

    string moduleName;
    string textDomain;
    MenuManager& menuManager;

    ClassInfoMap classNameToClassInfoMap;
    set<string> registeredTypeIds;
    set<ItemCreationPanel*> registeredCreationPanels;
    CreationPanelFilterSet registeredCreationPanelFilters;
    set<ItemFileIOBasePtr> registeredFileIOs;
    
    QSignalMapper* mapperForNewItemActivated;
    QSignalMapper* mapperForLoadSpecificTypeItemActivated;

    void detachManagedTypeItems(Item* parentItem);
        
    void registerClass(
        function<Item*()>& factory, Item* singletonInstance, const string& typeId, const string& className);
    
    void addCreationPanel(const string& typeId, ItemCreationPanel* panel);
    void addCreationPanelFilter(
        const string& typeId, shared_ptr<ItemManager::CreationPanelFilterBase> filter, bool afterInitializionByPanels);
    CreationPanelBase* getOrCreateCreationPanelBase(const string& typeId);

    void addFileIO(const string& typeId, ItemFileIOBasePtr fileIO);

    static bool load(
        Item* item, const string& filename, Item* parentItem, const string& formatId, const Mapping* options);
    static bool load(ItemFileIOBase* fileIO, Item* item, const string& filename, Item* parentItem);

    static bool save(Item* item, bool useDialogToGetFilename, bool doExport, string filename, const string& formatId);
    static ItemFileIOBasePtr getFileIOAndFilenameFromSaveDialog(
        vector<ItemFileIOBasePtr>& fileIOs, bool doExport,
        const string& itemLabel, const string& formatId, string& io_filename);
    static ItemFileIOBasePtr determineFileIOForSaving(
        vector<ItemFileIOBasePtr>& fileIOs, const string& filename, const string& formatId);
    static bool overwrite(Item* item, bool forceOverwrite, const string& formatId);

    void onNewItemActivated(CreationPanelBase* base);
    void onLoadItemActivated();
    void onLoadSpecificTypeItemActivated(ItemFileIOBasePtr fileIO);
    void onReloadSelectedItemsActivated();
    void onSaveSelectedItemsActivated();
    void onSaveSelectedItemsAsActivated();
    void onSaveAllItemsActivated();
    void onExportSelectedItemsActivated();
};

}

namespace {

ItemClassRegistry* itemClassRegistry = nullptr;

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

MessageView* messageView = nullptr;
bool isStaticMembersInitialized = false;

typedef map<string, ItemManagerImpl::ClassInfoPtr> ClassInfoMap;
ClassInfoMap typeIdToClassInfoMap;
    
typedef map<string, ItemManagerImpl*> ModuleNameToItemManagerImplMap;
ModuleNameToItemManagerImplMap moduleNameToItemManagerImplMap;
    
QWidget* importMenu;

std::map<ItemPtr, ItemPtr> reloadedItemToOriginalItemMap;


vector<string> separateExtensions(const string& multiExtString)
{
    vector<string> extensions;
    const char* str = multiExtString.c_str();
    do {
        const char* begin = str;
        while(*str != ';' && *str) ++str;
        extensions.push_back(string(begin, str));
    } while(0 != *str++);

    return extensions;
}
   

QString makeExtensionFilter(const string& caption, const string& extensions, bool isAnyEnabled = false)
{
    QString filter(caption.c_str());
    auto exts = separateExtensions(extensions);
    if(!exts.empty()){
        QString prefix = " (";
        for(auto& ext : exts){
            filter += prefix;
            filter += "*.";
            filter += ext.c_str();
            prefix = " ";
        }
        filter += ")";
    } else if(isAnyEnabled){
        filter += " (*)";
    }
    return filter;
}


QStringList makeExtensionFilterList(const string& caption, const string& extensions)
{
    QStringList filters;
    QString filter = makeExtensionFilter(caption, extensions);
    if(!filter.isEmpty()){
        filters << filter;
    }
    filters << _("Any files (*)");
    return filters;
}


QString makeExtensionFilterString(const string& caption, const string& extensions)
{
    QString filters = makeExtensionFilter(caption, extensions);
    filters += _(";;Any files (*)");
    return filters;
}
    
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
        /*
        menuManager.addItem(_("Open Item"))
            ->sigTriggered().connect([&](){ onLoadItemActivated(); });
        */
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
    // unregister creation panels
    for(auto it = registeredCreationPanels.begin(); it != registeredCreationPanels.end(); ++it){
        ItemCreationPanel* panel = *it;
        delete panel;
    }

    // unregister fileIO objects
    for(auto& fileIO : registeredFileIOs){
        if(auto classInfo = static_pointer_cast<ClassInfo>(fileIO->impl->classInfo.lock())){
            auto& fileIOs = classInfo->fileIOs;
            fileIOs.erase(std::remove(fileIOs.begin(), fileIOs.end(), fileIO), fileIOs.end());
        }
    }
    for(auto q = registeredTypeIds.begin(); q != registeredTypeIds.end(); ++q){
        const string& id = *q;
        typeIdToClassInfoMap.erase(id);
    }

    // unregister creation panel filters
    for(auto p = registeredCreationPanelFilters.begin(); p != registeredCreationPanelFilters.end(); ++p){
        ClassInfoMap::iterator q = typeIdToClassInfoMap.find(p->first);
        if(q != typeIdToClassInfoMap.end()){
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
        set<string>::iterator p = registeredTypeIds.find(typeid(*item).name());
        if(p != registeredTypeIds.end()){
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


void ItemManager::registerClassSub
(const std::string& className, const std::type_info& type, const std::type_info& superType,
 std::function<Item*()> factory, Item* singletonInstance)
{
    if(factory || singletonInstance){
        impl->registerClass(factory, singletonInstance, type.name(), className);
    }
    itemClassRegistry->registerClassAsTypeInfo(type, superType);
}


void ItemManagerImpl::registerClass
(std::function<Item*()>& factory, Item* singletonInstance, const string& typeId, const string& className)
{
    auto inserted = classNameToClassInfoMap.insert(make_pair(className, ClassInfoPtr()));
    ClassInfoPtr& info = inserted.first->second;
    if(inserted.second){
        info = new ClassInfo;
        info->moduleName = moduleName;
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

    registeredTypeIds.insert(typeId);
    typeIdToClassInfoMap[typeId] = info;
}


bool ItemManager::getClassIdentifier(ItemPtr item, std::string& out_moduleName, std::string& out_className)
{
    bool result;

    auto p = typeIdToClassInfoMap.find(typeid(*item).name());
    if(p != typeIdToClassInfoMap.end()){
        auto& info = p->second;
        out_moduleName = info->moduleName;
        out_className = info->className;
        result = true;
    } else {
        out_moduleName.clear();
        out_className = typeid(*item).name();
        result = false;
    }

    return result;
}


Item* ItemManager::getSingletonInstance(const std::string& typeId)
{
    auto p = typeIdToClassInfoMap.find(typeId);
    if(p != typeIdToClassInfoMap.end()){
        auto& info = p->second;
        if(info->isSingleton){
            return info->singletonInstance;
        }
    }
    return nullptr;
}


Item* ItemManager::createItem(const std::string& moduleName, const std::string& className)
{
    Item* item = nullptr;

    ModuleNameToItemManagerImplMap::iterator p = moduleNameToItemManagerImplMap.find(moduleName);
    if(p != moduleNameToItemManagerImplMap.end()){
        ClassInfoMap& classNameToClassInfoMap = p->second->classNameToClassInfoMap;
        auto q = classNameToClassInfoMap.find(className);
        if(q != classNameToClassInfoMap.end()){
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
    
    auto iter = typeIdToClassInfoMap.find(type.name());
    if(iter == typeIdToClassInfoMap.end()){
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


void ItemManager::addCreationPanelSub(const std::string& typeId, ItemCreationPanel* panel)
{
    impl->addCreationPanel(typeId, panel);
}


void ItemManagerImpl::addCreationPanel(const string& typeId, ItemCreationPanel* panel)
{
    CreationPanelBase* base = getOrCreateCreationPanelBase(typeId);
    if(panel){
        base->addPanel(panel);
    } else {
        base->addPanel(new DefaultCreationPanel(base));
    }
    registeredCreationPanels.insert(panel);
}


void ItemManager::addCreationPanelFilterSub
(const string& typeId, std::shared_ptr<CreationPanelFilterBase> filter, bool afterInitializionByPanels)
{
    impl->addCreationPanelFilter(typeId, filter, afterInitializionByPanels);
}


void ItemManagerImpl::addCreationPanelFilter
(const string& typeId, shared_ptr<ItemManager::CreationPanelFilterBase> filter, bool afterInitializionByPanels)
{
    CreationPanelBase* base = getOrCreateCreationPanelBase(typeId);
    if(!afterInitializionByPanels){
        base->preFilters.push_back(filter);
    } else {
        base->postFilters.push_back(filter);
    }
    registeredCreationPanelFilters.insert(make_pair(typeId, filter));
}


ItemManagerImpl::CreationPanelBase* ItemManagerImpl::getOrCreateCreationPanelBase(const string& typeId)
{
    CreationPanelBase* base = nullptr;
    
    auto p = typeIdToClassInfoMap.find(typeId);
    if(p != typeIdToClassInfoMap.end()){
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


ItemManagerImpl::CreationPanelBase::CreationPanelBase
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


void ItemManagerImpl::CreationPanelBase::addPanel(ItemCreationPanel* panel)
{
    panelLayout->addWidget(panel);
}


Item* ItemManagerImpl::CreationPanelBase::createItem(Item* parentItem)
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

    for(CreationPanelFilterList::iterator p = preFilters.begin(); p != preFilters.end(); ++p){
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
                for(CreationPanelFilterList::iterator p = postFilters.begin(); p != postFilters.end(); ++p){
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
class FileFunctionAdapter : public ItemFileIOBase
{
    std::shared_ptr<ItemManager::FileFunctionBase> func;
    
public:
    FileFunctionAdapter(
        int api, const std::string& caption, const std::string& formatId,
        const std::function<std::string()>& getExtensions, std::shared_ptr<ItemManager::FileFunctionBase> function,
        int priority)
        : ItemFileIOBase(formatId, api),
          func(function)
    {
        setCaption(caption);
        setExtensionFunction(getExtensions);
        if(priority >= ItemManager::PRIORITY_DEFAULT){
            setInterfaceLevel(Standard);
        } else if(priority >= ItemManager::PRIORITY_COMPATIBILITY){
            setInterfaceLevel(Compatibility);
        } else {
            setInterfaceLevel(Conversion);
        }
    }

    virtual bool load(Item* item, const std::string& filename) override
    {
        return (*func)(item, filename, os(), parentItem());
    };

    virtual bool save(Item* item, const std::string& filename) override
    {
        return (*func)(item, filename, os(), parentItem());
    };
};

}


void ItemManager::addLoaderSub
(const std::string& typeId, const std::string& caption, const std::string& formatId,
 std::function<std::string()> getExtensions, std::shared_ptr<FileFunctionBase> function, int priority)
{
    auto adapter = new FileFunctionAdapter(
        ItemFileIOBase::Load, caption, formatId, getExtensions, function, priority);
    impl->addFileIO(typeId, adapter);
}


void ItemManager::addSaverSub
(const std::string& typeId, const std::string& caption, const std::string& formatId,
 std::function<std::string()> getExtensions, std::shared_ptr<FileFunctionBase> function, int priority)
{
    auto adapter = new FileFunctionAdapter(
        ItemFileIOBase::Save, caption, formatId, getExtensions, function, priority);
    impl->addFileIO(typeId, adapter);
}


void ItemManager::addFileIO_(const std::string& typeId, ItemFileIOBase* fileIO)
{
    impl->addFileIO(typeId, fileIO);
}


void ItemManagerImpl::addFileIO(const string& typeId, ItemFileIOBasePtr fileIO)
{
    ClassInfoMap::iterator p = typeIdToClassInfoMap.find(typeId);
    if(p != typeIdToClassInfoMap.end()){
        auto& classInfo = p->second;
        auto& ioImpl = fileIO->impl;
        ioImpl->typeId = typeId;
        ioImpl->classInfo = classInfo;

        registeredFileIOs.insert(fileIO);

        auto& fileIOs = classInfo->fileIOs;
        if(ioImpl->interfaceLevel == ItemFileIOBase::Standard){
            fileIOs.insert(fileIOs.begin(), fileIO);
        } else {
            fileIOs.push_back(fileIO);
        }

        if(ioImpl->api & ItemFileIOBase::Load){
            if(ioImpl->interfaceLevel != ItemFileIOBase::Compatibility){
                if(ioImpl->interfaceLevel == ItemFileIOBase::Standard){
                    menuManager.setPath("/File/Open ...");
                } else {
                    menuManager.setPath("/File/Import ...");
                }
                menuManager.addItem(ioImpl->caption)
                    ->sigTriggered().connect(
                        [=](){ onLoadSpecificTypeItemActivated(fileIO); });
            }
        }
    }
}


bool ItemManager::load
(Item* item, const std::string& filename, Item* parentItem, const std::string& formatId, const Mapping* options)
{
    return ItemManagerImpl::load(item, filename, parentItem, formatId, options);
}


bool ItemManagerImpl::load
(Item* item, const string& filename, Item* parentItem, const string& formatId, const Mapping* options)
{
    if(filename.empty()){
        messageView->putln(
            _("Item with empty filename cannot be loaded."), MessageView::ERROR);
        return false;
    }
        
    ParametricPathProcessor* pathProcessor = ParametricPathProcessor::instance();
    auto expanded = pathProcessor->expand(filename);
    if(!expanded){
        messageView->putln(pathProcessor->errorMessage());
        return false;
    }
        
    filesystem::path filepath = cnoid::getAbsolutePath(*expanded);
            
    string pathString = cnoid::getPathString(filepath);
    
    const string& typeId = typeid(*item).name();
    ClassInfoMap::iterator p = typeIdToClassInfoMap.find(typeId);
    if(p == typeIdToClassInfoMap.end()){
        messageView->putln(
            format(_("\"{0}\" cannot be loaded because item type \"{1}\" is not registered."),
            pathString, typeId),
            MessageView::ERROR);
        return false;
    }
    
    ClassInfoPtr& classInfo = p->second;
    auto& fileIOs = classInfo->fileIOs;
    bool loaded = false;
    ItemFileIOBasePtr targetFileIO;

    if(!formatId.empty()){
        for(auto& fileIO : fileIOs){
            if(fileIO->impl->api & ItemFileIOBase::Load){
                if(fileIO->impl->formatId == formatId){
                    targetFileIO = fileIO;
                    break;
                }
            }
        }
    } else {
        string dotextension = filepath.extension().string();
        if(dotextension.size() >= 2){
            string extension = dotextension.substr(1); // remove dot
            for(auto& fileIO : fileIOs){
                if(fileIO->impl->api & ItemFileIOBase::Load){
                    auto exts = separateExtensions(fileIO->impl->extensionFunction());
                    for(auto& ext : exts){
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
                format(_("\"{}\" cannot be loaded because the file format is unknown."), pathString),
                MessageView::ERROR);
        } else {
            messageView->putln(
                format(_("\"{0}\" cannot be loaded because file format \"{1}\" is unknown."),
                pathString, formatId),
                MessageView::ERROR);
        }
    } else {
        targetFileIO->impl->invocationType = ItemFileIOBase::Direct;
        if(targetFileIO->impl->api & ItemFileIOBase::Options){
            targetFileIO->resetOptions();
            if(options){
                targetFileIO->restoreOptions(options);
            }
        }
        if(load(targetFileIO, item, pathString, parentItem)){
            loaded = true;
        }
    }

    return loaded;
}
        

bool ItemManagerImpl::load(ItemFileIOBase* fileIO, Item* item, const string& filename_, Item* parentItem)
{
    bool loaded = false;
    
    string filename(toActualPathName(filename_));
        
    messageView->notify(format(_("Loading {0} \"{1}\""), fileIO->impl->caption, filename));
    messageView->flush();

    if(!parentItem){
        parentItem = RootItem::instance();
    }
    fileIO->impl->parentItem = parentItem;

    loaded = fileIO->load(item, filename);
    fileIO->impl->os->flush();
        
    if(!loaded){
        messageView->put(MessageView::HIGHLIGHT, _(" -> failed.\n"));
    } else {
        if(item->name().empty()){
            item->setName(filesystem::path(filename).stem().string());
        }
        MappingPtr optionArchive;
        if(fileIO->impl->api & ItemFileIOBase::Options){
            optionArchive = new Mapping;
            fileIO->storeOptions(optionArchive);
        }
        item->updateFileInformation(filename, fileIO->impl->formatId, optionArchive);
        messageView->put(_(" -> ok!\n"));
    }
    messageView->flush();

    return loaded;
}


void ItemManagerImpl::onLoadItemActivated()
{

}


void ItemManagerImpl::onLoadSpecificTypeItemActivated(ItemFileIOBasePtr fileIO)
{
    ItemPtr item;
    auto classInfo = static_pointer_cast<ClassInfo>(fileIO->impl->classInfo.lock());
    if(classInfo->isSingleton){
        item = classInfo->singletonInstance;
        if(item->parentItem()){
            showWarningDialog(format(_("The singleton instance of {} is already loaded."),
            classInfo->className));
            return;
        }
    }

    QDialog dialog(MainWindow::instance());
    dialog.setSizeGripEnabled(true);
    auto vbox = new QVBoxLayout;
    vbox->setSpacing(0);
    dialog.setLayout(vbox);
    
    QFileDialog fileDialog(&dialog);
    fileDialog.setWindowFlags(fileDialog.windowFlags() & ~Qt::Dialog);
    fileDialog.setOption(QFileDialog::DontUseNativeDialog);
    fileDialog.setWindowTitle(QString(_("Load %1")).arg(fileIO->impl->caption.c_str()));
    fileDialog.setSizeGripEnabled(false);
    fileDialog.setViewMode(QFileDialog::List);
    fileDialog.setLabelText(QFileDialog::Accept, _("Open"));
    fileDialog.setLabelText(QFileDialog::Reject, _("Cancel"));
    fileDialog.setDirectory(AppConfig::archive()->get
                        ("currentFileDialogDirectory", shareDirectory()).c_str());

    QObject::connect(&fileDialog, SIGNAL(finished(int)), &dialog, SLOT(done(int)));

    vbox->addWidget(&fileDialog);

    QWidget* optionPanel = nullptr;
    if(fileIO->impl->api & ItemFileIOBase::Options){
        fileIO->resetOptions();
        if(fileIO->impl->api & ItemFileIOBase::OptionPanelForLoading){
            optionPanel = fileIO->optionPanelForLoading();
            if(optionPanel){
                auto hbox = new QHBoxLayout;
                auto style = dialog.style();
                int left = style->pixelMetric(QStyle::PM_LayoutLeftMargin);
                int right = style->pixelMetric(QStyle::PM_LayoutRightMargin);
                int bottom = style->pixelMetric(QStyle::PM_LayoutBottomMargin);
                hbox->setContentsMargins(left, 0, right, bottom);
                hbox->addWidget(optionPanel);
                hbox->addStretch();
                vbox->addLayout(hbox);
            }
        }
    }

    fileDialog.setNameFilters(makeExtensionFilterList(fileIO->impl->caption, fileIO->impl->extensionFunction()));

    if(classInfo->isSingleton){
        fileDialog.setFileMode(QFileDialog::ExistingFile);
    } else {
        fileDialog.setFileMode(QFileDialog::ExistingFiles);
    }

    
    if(dialog.exec() == QDialog::Accepted){
        Mapping* config = AppConfig::archive();

        config->writePath(
            "currentFileDialogDirectory",
            fileDialog.directory().absolutePath().toStdString());
                  
        QStringList filenames = fileDialog.selectedFiles();

        Item* parentItem = RootItem::instance()->selectedItems().toSingle();
        if(!parentItem){
            parentItem = RootItem::instance();
        }

        fileIO->impl->invocationType = ItemFileIOBase::Dialog;

        if(optionPanel){
            fileIO->fetchOptionPanelForLoading();
        }
        
        for(int i=0; i < filenames.size(); ++i){
            if(!classInfo->isSingleton){
                item = classInfo->factory();
            }
            string filename = getNativePathString(filesystem::path(filenames[i].toStdString()));
            if(load(fileIO, item.get(), filename, parentItem)){
                parentItem->addChildItem(item, true);
            }
        }
    }

    if(optionPanel){
        optionPanel->setParent(nullptr);
    }
}


bool ItemManager::save(Item* item, const std::string& filename, const std::string& formatId)
{
    return ItemManagerImpl::save(item, false, false, filename, formatId);
}


bool ItemManagerImpl::save
(Item* item, bool useDialogToGetFilename, bool doExport, string filename, const string& formatId)
{
    item->setTemporal(false);
    
    ClassInfoMap::iterator p = typeIdToClassInfoMap.find(typeid(*item).name());
    if(p == typeIdToClassInfoMap.end()){
        return false;
    }

    ClassInfoPtr& classInfo = p->second;
   
    bool saved = false;
    bool tryToSave = false;

    string itemLabel = classInfo->className + " \"" + item->name() + "\"";

    auto& fileIOs = classInfo->fileIOs;
    ItemFileIOBasePtr targetFileIO;
    
    if(useDialogToGetFilename){
        targetFileIO = getFileIOAndFilenameFromSaveDialog(fileIOs, doExport, itemLabel, formatId, filename);
    } else {
        targetFileIO = determineFileIOForSaving(fileIOs, filename, formatId);
    }
    
    if(targetFileIO && targetFileIO->impl->api & ItemFileIOBase::Save){
        
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
            if(targetFileIO->impl->interfaceLevel == ItemFileIOBase::Conversion){
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


ItemFileIOBasePtr ItemManagerImpl::getFileIOAndFilenameFromSaveDialog
(vector<ItemFileIOBasePtr>& fileIOs, bool doExport, const string& itemLabel, const string& formatId,
 string& io_filename)
{
    QFileDialog dialog(MainWindow::instance());
    dialog.setWindowTitle(QString(_("Save %1 as")).arg(itemLabel.c_str()));
    dialog.setFileMode(QFileDialog::AnyFile);
    dialog.setAcceptMode(QFileDialog::AcceptSave);
    dialog.setViewMode(QFileDialog::List);
    dialog.setLabelText(QFileDialog::Accept, _("Save"));
    dialog.setLabelText(QFileDialog::Reject, _("Cancel"));

    if(!io_filename.empty()){
        dialog.selectFile(io_filename.c_str());
        io_filename.clear();
    }

    QStringList filters;
    vector<ItemFileIOBasePtr> activeFileIOs;
    
    for(auto& fileIO : fileIOs){
        if(!(fileIO->impl->api & ItemFileIOBase::Save)){
            continue;
        }
        if(fileIO->impl->interfaceLevel == ItemFileIOBase::Compatibility){
            continue;
        }
        bool isExporter = (fileIO->impl->interfaceLevel == ItemFileIOBase::Conversion);
        if((doExport && !isExporter) || (!doExport && isExporter)){
            continue;
        }
        if(!formatId.empty() && fileIO->impl->formatId != formatId){
            continue;
        }
        filters << makeExtensionFilter(fileIO->impl->caption, fileIO->impl->extensionFunction(), true);
        activeFileIOs.push_back(fileIO);
    }

    dialog.setNameFilters(filters);

    ItemFileIOBasePtr targetFileIO;

    if(filters.size() > 0){
    
        dialog.setDirectory(AppConfig::archive()->get
                            ("currentFileDialogDirectory", shareDirectory()).c_str());
    
        if(dialog.exec() == QFileDialog::Accepted){

            AppConfig::archive()->writePath(
                "currentFileDialogDirectory",
                dialog.directory().absolutePath().toStdString());

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
                    string extensions = targetFileIO->impl->extensionFunction();
                    // add a lacking extension automatically
                    if(!extensions.empty()){
                        bool hasExtension = false;
                        auto exts = separateExtensions(extensions);
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


ItemFileIOBasePtr ItemManagerImpl::determineFileIOForSaving
(vector<ItemFileIOBasePtr>& fileIOs, const string& filename, const string& formatId)
{
    ItemFileIOBasePtr targetFileIO;

    if(!formatId.empty()){
        for(auto& fileIO : fileIOs){
            if(fileIO->impl->api & ItemFileIOBase::Save){
                if(fileIO->impl->formatId == formatId){
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
                if(fileIO->impl->api & ItemFileIOBase::Save){
                    auto exts = separateExtensions(fileIO->impl->extensionFunction());
                    for(auto& ext : exts){
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
                if(fileIO->impl->api & ItemFileIOBase::Save){
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


namespace cnoid {

string getOpenFileName(const string& caption, const string& extensions)
{
    QString qfilename =
        QFileDialog::getOpenFileName(
            MainWindow::instance(),
            caption.c_str(),
            AppConfig::archive()->get("currentFileDialogDirectory", shareDirectory()).c_str(),
            makeExtensionFilterString(caption, extensions));

    string filename = qfilename.toStdString();

    if(!filename.empty()){
        AppConfig::archive()->writePath(
            "currentFileDialogDirectory",
            filesystem::path(filename).parent_path().string());
    }

    return filename;
}
    
vector<string> getOpenFileNames(const string& caption, const string& extensions)
{
    QStringList qfilenames =
        QFileDialog::getOpenFileNames(
            MainWindow::instance(),
            caption.c_str(),
            AppConfig::archive()->get("currentFileDialogDirectory", shareDirectory()).c_str(),
            makeExtensionFilterString(caption, extensions));

    vector<string> filenames;

    if(!qfilenames.empty()){
        for(int i=0; i < qfilenames.size(); ++i){
            filenames.push_back(qfilenames[i].toStdString());
        }
        AppConfig::archive()->writePath(
            "currentFileDialogDirectory",
            filesystem::path(filenames[0]).parent_path().string());
    }
        
    return filenames;
}

}
