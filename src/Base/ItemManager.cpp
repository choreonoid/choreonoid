/**
   @author Shin'ichiro Nakaoka
*/

#include "ItemManager.h"
#include "Item.h"
#include "RootItem.h"
#include "ItemTreeView.h"
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

    typedef list<shared_ptr<ItemManager::CreationPanelFilterBase>> CreationPanelFilterList;
    typedef set<pair<string, shared_ptr<ItemManager::CreationPanelFilterBase>>> CreationPanelFilterSet;
    
    class CreationPanelBase : public QDialog
    {
    public:
        CreationPanelBase(const QString& title, ItemPtr protoItem, bool isSingleton);
        void addPanel(ItemCreationPanel* panel);
        ItemPtr createItem(ItemPtr parentItem);
        CreationPanelFilterList preFilters;
        CreationPanelFilterList postFilters;
    private:
        QVBoxLayout* panelLayout;
        ItemPtr protoItem;
        bool isSingleton;
    };
    
    struct Saver;
    typedef shared_ptr<Saver> SaverPtr;
    
    struct Loader;
    typedef shared_ptr<Loader> LoaderPtr;

    struct ClassInfo
    {
        ClassInfo() { creationPanelBase = 0; }
        ~ClassInfo() { delete creationPanelBase; }
        string moduleName;
        string className;
        string name; // without the 'Item' suffix
        function<Item*()> factory;
        CreationPanelBase* creationPanelBase;
        list<LoaderPtr> loaders;
        list<SaverPtr> savers;
        ItemPtr singletonInstance;
        bool isSingleton;
    };
    typedef shared_ptr<ClassInfo> ClassInfoPtr;
    
    typedef map<string, ClassInfoPtr> ClassInfoMap;
    
    struct Loader : public QObject
    {
        string typeId;
        string formatId;
        string caption;
        int priority;
        std::shared_ptr<ItemManager::FileFunctionBase> loadingFunction;
        weak_ptr<ClassInfo> classInfo;
        function<string()> getExtensions;
    };
        
    struct Saver
    {
        string typeId;
        string formatId;
        string caption;
        int priority;
        function<string()> getExtensions;
        std::shared_ptr<ItemManager::FileFunctionBase> savingFunction;
    };
    
    string moduleName;
    string textDomain;
    MenuManager& menuManager;

    ClassInfoMap classNameToClassInfoMap;
    set<string> registeredTypeIds;
    set<ItemCreationPanel*> registeredCreationPanels;
    CreationPanelFilterSet registeredCreationPanelFilters;
    set<LoaderPtr> registeredLoaders;
    set<SaverPtr> registeredSavers;
    
    QSignalMapper* mapperForNewItemActivated;
    QSignalMapper* mapperForLoadSpecificTypeItemActivated;

    void detachManagedTypeItems(Item* parentItem);
        
    void registerClass(
        function<Item*()>& factory, Item* singletonInstance, const string& typeId, const string& className);
    
    void addCreationPanel(const string& typeId, ItemCreationPanel* panel);
    void addCreationPanelFilter(
        const string& typeId, shared_ptr<ItemManager::CreationPanelFilterBase> filter, bool afterInitializionByPanels);
    CreationPanelBase* getOrCreateCreationPanelBase(const string& typeId);

    void addLoader
    (const string& typeId, const string& caption, const string& formatId, function<string()>& getExtensions,
     const shared_ptr<ItemManager::FileFunctionBase> function, int priority);

    static bool load(Item* item, const string& filename, Item* parentItem, const string& formatId);
    static bool load(LoaderPtr loader, Item* item, const string& filename, Item* parentItem);

    void addSaver
    (const string& typeId, const string& caption, const string& formatId, function<string()>& getExtensions, 
     shared_ptr<ItemManager::FileFunctionBase> function, int priority);

    static bool save(Item* item, bool useDialogToGetFilename, bool doExport, string filename, const string& formatId);
    static SaverPtr getSaverAndFilenameFromSaveDialog(
        list<SaverPtr>& savers, bool doExport,
        const string& itemLabel, const string& formatId, string& io_filename);
    static SaverPtr determineSaver(list<SaverPtr>& savers, const string& filename, const string& formatId);
    static bool overwrite(Item* item, bool forceOverwrite, const string& formatId);

    static void onNewItemActivated(CreationPanelBase* base);
    void onLoadItemActivated();
    static void onLoadSpecificTypeItemActivated(LoaderPtr loader);
    void onReloadSelectedItemsActivated();
    void onSaveSelectedItemsActivated();
    void onSaveSelectedItemsAsActivated();
    void onSaveAllItemsActivated();
    void onExportSelectedItemsActivated();
};

}

namespace {

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

MessageView* messageView = 0;
bool isStaticMembersInitialized = false;

typedef map<string, ItemManagerImpl::ClassInfoPtr> ClassInfoMap;
ClassInfoMap typeIdToClassInfoMap;
    
typedef map<string, ItemManagerImpl*> ModuleNameToItemManagerImplMap;
ModuleNameToItemManagerImplMap moduleNameToItemManagerImplMap;
    
typedef map<string, ItemManagerImpl::CreationPanelBase*> CreationPanelBaseMap;
CreationPanelBaseMap creationPanelBaseMap;

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

        menuManager.setPath("/File").setPath(N_("New ..."));
        
        menuManager.setPath("/File");
        /*
        menuManager.addItem(_("Open Item"))
            ->sigTriggered().connect(std::bind(&ItemManagerImpl::onLoadItemActivated, this));
        */
        menuManager.setPath(N_("Open ..."));
        menuManager.setPath("/File");
        menuManager.addItem(_("Reload Selected Items"))
            ->sigTriggered().connect(std::bind(&ItemManagerImpl::onReloadSelectedItemsActivated, this));
        
        menuManager.addSeparator();

        menuManager.addItem(_("Save Selected Items"))
            ->sigTriggered().connect(std::bind(&ItemManagerImpl::onSaveSelectedItemsActivated, this));
        menuManager.addItem(_("Save Selected Items As"))
            ->sigTriggered().connect(std::bind(&ItemManagerImpl::onSaveSelectedItemsAsActivated, this));
        menuManager.addItem(_("Save All Items"))
            ->sigTriggered().connect(std::bind(&ItemManagerImpl::onSaveAllItemsActivated, this));

        menuManager.addSeparator();
        
        menuManager.setPath(N_("Import ..."));
        importMenu = menuManager.current();
        
        menuManager.setPath("/File");
        menuManager.addItem(_("Export Selected Items"))
            ->sigTriggered().connect(std::bind(&ItemManagerImpl::onExportSelectedItemsActivated, this));
        
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
    
    // unregister loaders
    for(auto it = registeredLoaders.begin(); it != registeredLoaders.end(); ++it){
        LoaderPtr loader = *it;
        ClassInfoMap::iterator p = typeIdToClassInfoMap.find(loader->typeId);
        if(p != typeIdToClassInfoMap.end()){
            list<LoaderPtr>& loaders = p->second->loaders;
            list<LoaderPtr>::iterator q = loaders.begin();
            while(q != loaders.end()){
                if(loader == *q){
                    q = loaders.erase(q);
                } else {
                    q++;
                }
            }
        }
    }

    // unregister savers
    for(auto it = registeredSavers.begin(); it != registeredSavers.end(); ++it){
        SaverPtr saver = *it;
        ClassInfoMap::iterator p = typeIdToClassInfoMap.find(saver->typeId);
        if(p != typeIdToClassInfoMap.end()){
            list<SaverPtr>& savers = p->second->savers;
            list<SaverPtr>::iterator q = savers.begin();
            while(q != savers.end()){
                if(saver == *q){
                    q = savers.erase(q);
                } else {
                    q++;
                }
            }
        }
    }

    // unregister item class identifiers, CreationPanelBases and savers
    for(auto q = registeredTypeIds.begin(); q != registeredTypeIds.end(); ++q){
        const string& id = *q;
        CreationPanelBaseMap::iterator s = creationPanelBaseMap.find(id);
        if(s != creationPanelBaseMap.end()){
            CreationPanelBase* base = s->second;
            delete base;
            creationPanelBaseMap.erase(s);
        }
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
(std::function<Item*()> factory, Item* singletonInstance, const std::string& typeId, const std::string& className)
{
    impl->registerClass(factory, singletonInstance, typeId, className);
}


void ItemManagerImpl::registerClass
(std::function<Item*()>& factory, Item* singletonInstance, const string& typeId, const string& className)
{
    auto inserted = classNameToClassInfoMap.insert(make_pair(className, ClassInfoPtr()));
    ClassInfoPtr& info = inserted.first->second;
    if(inserted.second){
        info = std::make_shared<ClassInfo>();
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

    ClassInfoMap::iterator p = typeIdToClassInfoMap.find(typeid(*item).name());
    if(p != typeIdToClassInfoMap.end()){
        ItemManagerImpl::ClassInfoPtr& info = p->second;
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
    ClassInfoMap::iterator p = typeIdToClassInfoMap.find(typeId);
    if(p != typeIdToClassInfoMap.end()){
        ItemManagerImpl::ClassInfoPtr& info = p->second;
        if(info->isSingleton){
            return info->singletonInstance;
        }
    }
    return 0;
}


ItemPtr ItemManager::create(const std::string& moduleName, const std::string& className)
{
    ItemPtr item;

    ModuleNameToItemManagerImplMap::iterator p = moduleNameToItemManagerImplMap.find(moduleName);
    if(p != moduleNameToItemManagerImplMap.end()){
        ClassInfoMap& classNameToClassInfoMap = p->second->classNameToClassInfoMap;
        ClassInfoMap::iterator q = classNameToClassInfoMap.find(className);
        if(q != classNameToClassInfoMap.end()){
            ItemManagerImpl::ClassInfoPtr& info = q->second;
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
    CreationPanelBase* base = 0;
    
    ClassInfoMap::iterator p = typeIdToClassInfoMap.find(typeId);
    if(p != typeIdToClassInfoMap.end()){
        ClassInfoPtr& info = p->second;
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
            } else {
                protoItem = info->factory();
                protoItem->setName(info->name);
            }
            base = new CreationPanelBase(title, protoItem, info->isSingleton);
            base->hide();
            menuManager.setPath("/File/New ...").addItem(translatedName)
                ->sigTriggered().connect(std::bind(ItemManagerImpl::onNewItemActivated, base));
            info->creationPanelBase = base;
        }
    }

    return base;
}


void ItemManagerImpl::onNewItemActivated(CreationPanelBase* base)
{
    ItemTreeView* itemTreeView = ItemTreeView::mainInstance();
    ItemList<Item> parentItems = itemTreeView->selectedItems();

    if(parentItems.empty()){
        parentItems.push_back(itemTreeView->rootItem());
    }
    for(size_t i=0; i < parentItems.size(); ++i){
        ItemPtr parentItem = parentItems[i];
        ItemPtr newItem = base->createItem(parentItem);
        if(newItem){
            parentItem->addChildItem(newItem, true);
        }
    }
}


ItemManagerImpl::CreationPanelBase::CreationPanelBase(const QString& title, ItemPtr protoItem, bool isSingleton)
    : QDialog(MainWindow::instance()),
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


ItemPtr ItemManagerImpl::CreationPanelBase::createItem(ItemPtr parentItem)
{
    if(isSingleton){
        if(protoItem->parentItem()){
            return 0;
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

    for(CreationPanelFilterList::iterator p = preFilters.begin(); p != preFilters.end(); ++p){
        auto filter = *p;
        if(!(*filter)(protoItem.get(), parentItem.get())){
            result = false;
            break;
        }
    }

    if(result){
        for(size_t i=0; i < panels.size(); ++i){
            if(!panels[i]->initializePanel(protoItem.get())){
                result = false;
                break;
            }
        }
    }

    if(result){
        if(exec() == QDialog::Accepted){
            for(size_t i=0; i < panels.size(); ++i){
                if(!panels[i]->initializeItem(protoItem.get())){
                    result = false;
                    break;
                }
            }
            if(result){
                for(CreationPanelFilterList::iterator p = postFilters.begin(); p != postFilters.end(); ++p){
                    auto filter = *p;
                    if(!(*filter)(protoItem.get(), parentItem.get())){
                        result = false;
                        break;
                    }
                }
            }
        } else {
            result = false;
        }
    }
    
    ItemPtr newItem;
    if(result){
        if(isSingleton){
            newItem = protoItem;
        } else {
            newItem = protoItem->duplicate();
        }
    }
    
    return newItem;
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
    return 0;
}


void ItemManager::addLoaderSub
(const std::string& typeId, const std::string& caption, const std::string& formatId,
 std::function<std::string()> getExtensions, std::shared_ptr<FileFunctionBase> function, int priority)
{
    impl->addLoader(typeId, caption, formatId, getExtensions, function, priority);
}


void ItemManagerImpl::addLoader
(const string& typeId, const string& caption, const string& formatId,
 function<string()>& getExtensions, shared_ptr<ItemManager::FileFunctionBase> function, int priority)
{
    ClassInfoMap::iterator p = typeIdToClassInfoMap.find(typeId);
    if(p != typeIdToClassInfoMap.end()){

        ClassInfoPtr& classInfo = p->second;
        
        LoaderPtr loader = make_shared<Loader>();
        loader->typeId = typeId;
        loader->caption = caption;
        loader->formatId = formatId;
        loader->priority = priority;
        loader->loadingFunction = function;
        loader->classInfo = classInfo;
        loader->getExtensions = getExtensions;

        if(priority != ItemManager::PRIORITY_COMPATIBILITY){
            bool isImporter = (priority <= ItemManager::PRIORITY_CONVERSION);

            if(!isImporter){
                menuManager.setPath("/File/Open ...");
            } else {
                menuManager.setPath("/File/Import ...");
            }
            menuManager.addItem(caption.c_str())
                ->sigTriggered().connect(
                    std::bind(&ItemManagerImpl::onLoadSpecificTypeItemActivated, loader));
        }
        
        registeredLoaders.insert(loader);

        // insert loader to a proper position of the list considering priorities 
        list<LoaderPtr>& loaders = classInfo->loaders;
        list<LoaderPtr>::iterator it = loaders.begin();
        while(true){
            if(it == loaders.end()){
                loaders.push_back(loader);
                break;
            }
            LoaderPtr loader2 = *it;
            if(loader->priority > loader2->priority){
                loaders.insert(it, loader);
                break;
            }
            ++it;
        }
    }
}


bool ItemManager::load(Item* item, const std::string& filename, Item* parentItem, const std::string& formatId)
{
    return ItemManagerImpl::load(item, filename, parentItem, formatId);
}


bool ItemManagerImpl::load(Item* item, const string& filename, Item* parentItem, const string& formatId)
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
    list<LoaderPtr>& loaders = classInfo->loaders;
    bool loaded = false;
    LoaderPtr targetLoader;

    if(!formatId.empty()){
        for(list<LoaderPtr>::iterator p = loaders.begin(); p != loaders.end(); ++p){
            LoaderPtr& loader = *p;
            if(loader->formatId == formatId){
                targetLoader = loader;
                break;
            }
        }
    } else {
        string dotextension = filepath.extension().string();
        if(dotextension.size() >= 2){
            string extension = dotextension.substr(1); // remove dot
            for(list<LoaderPtr>::iterator p = loaders.begin(); p != loaders.end(); ++p){
                LoaderPtr& loader = *p;
                auto exts = separateExtensions(loader->getExtensions());
                for(auto& ext : exts){
                    if(ext == extension){
                        targetLoader = loader;
                        break;
                    }
                }
            }
        }
    }

    if(!targetLoader){
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
        if(load(targetLoader, item, pathString, parentItem)){
            loaded = true;
        }
    }

    return loaded;
}
        

bool ItemManagerImpl::load(LoaderPtr loader, Item* item, const string& filename_, Item* parentItem)
{
    bool loaded = false;
    
    if(loader->loadingFunction){

        string filename(toActualPathName(filename_));
        
        messageView->notify(format(_("Loading {0} \"{1}\""), loader->caption, filename));
        messageView->flush();

        if(!parentItem){
            parentItem = RootItem::mainInstance();
        }

        ostream& os = messageView->cout(true);
        loaded = (*loader->loadingFunction)(item, filename, os, parentItem);
        os.flush();
        
        if(!loaded){
            messageView->put(MessageView::HIGHLIGHT, _(" -> failed.\n"));
        } else {
            if(item->name().empty()){
                item->setName(filesystem::path(filename).stem().string());
            }
            item->updateFileInformation(filename, loader->formatId);
            messageView->put(_(" -> ok!\n"));
        }
        messageView->flush();
    }

    return loaded;
}


void ItemManagerImpl::onLoadItemActivated()
{
}


void ItemManagerImpl::onLoadSpecificTypeItemActivated(LoaderPtr loader)
{
    ItemPtr item;
    ClassInfoPtr classInfo = loader->classInfo.lock();
    if(classInfo->isSingleton){
        item = classInfo->singletonInstance;
        if(item->parentItem()){
            showWarningDialog(format(_("The singleton instance of {} is already loaded."),
            classInfo->className));
            return;
        }
    }
    
    QFileDialog dialog(MainWindow::instance());
    //dialog.setOption(QFileDialog::DontUseNativeDialog);
    dialog.setWindowTitle(QString(_("Load %1")).arg(loader->caption.c_str()));
    dialog.setViewMode(QFileDialog::List);
    dialog.setLabelText(QFileDialog::Accept, _("Open"));
    dialog.setLabelText(QFileDialog::Reject, _("Cancel"));
    dialog.setDirectory(AppConfig::archive()->get
                        ("currentFileDialogDirectory", shareDirectory()).c_str());

    static const char* checkConfigKey = "defaultChecked";

    bool isCheckedByDefault = false;
    CheckBox checkCheckBox(_("Check the item(s) in ItemTreeView"));
    QGridLayout* layout = dynamic_cast<QGridLayout*>(dialog.layout());

    if(layout){
        Mapping* conf = AppConfig::archive()->findMapping("ItemTreeView");
        if(conf->isValid()){
            conf = conf->findMapping(checkConfigKey);
            if(conf->isValid()){
                conf = conf->findMapping(classInfo->moduleName);
                if(conf->isValid() && conf->read(classInfo->className, isCheckedByDefault)){
                    checkCheckBox.setChecked(isCheckedByDefault);
                }
            }
        }
        layout->addWidget(&checkCheckBox, 4, 0, 1, 3);
    }

    dialog.setNameFilters(makeExtensionFilterList(loader->caption, loader->getExtensions()));

    if(classInfo->isSingleton){
        dialog.setFileMode(QFileDialog::ExistingFile);
    } else {
        dialog.setFileMode(QFileDialog::ExistingFiles);
    }
    
    if(dialog.exec()){
        Mapping* config = AppConfig::archive();

        config->writePath(
            "currentFileDialogDirectory",
            dialog.directory().absolutePath().toStdString());

        if(checkCheckBox.isChecked() != isCheckedByDefault){
            Mapping* checkConfig = config
                ->openMapping("ItemTreeView")
                ->openMapping(checkConfigKey)
                ->openMapping(classInfo->moduleName);
            checkConfig->write(classInfo->className, checkCheckBox.isChecked());
            AppConfig::flush();
        }
                  
        QStringList filenames = dialog.selectedFiles();

        ItemTreeView* itemTreeView = ItemTreeView::instance();
        Item* parentItem = itemTreeView->selectedItem<Item>();
        if(!parentItem){
            parentItem = RootItem::instance();
        }

        for(int i=0; i < filenames.size(); ++i){
            if(!classInfo->isSingleton){
                item = classInfo->factory();
            }
            string filename = getNativePathString(filesystem::path(filenames[i].toStdString()));
            if(load(loader, item.get(), filename, parentItem)){
                parentItem->addChildItem(item, true);

                if(checkCheckBox.isChecked()){
                    itemTreeView->checkItem(item);
                }
            }
        }
    }
}


void ItemManager::addSaverSub
(const std::string& typeId, const std::string& caption, const std::string& formatId,
 std::function<std::string()> getExtensions, std::shared_ptr<FileFunctionBase> function, int priority)
{
    impl->addSaver(typeId, caption, formatId, getExtensions, function, priority);
}


void ItemManagerImpl::addSaver
(const string& typeId, const string& caption, const string& formatId, function<string()>& getExtensions,
 shared_ptr<ItemManager::FileFunctionBase> function, int priority)
{
    ClassInfoMap::iterator p = typeIdToClassInfoMap.find(typeId);
    if(p != typeIdToClassInfoMap.end()){

        SaverPtr saver = make_shared<Saver>();
        
        saver->typeId = typeId;
        saver->formatId = formatId;
        saver->caption = caption;
        saver->priority = priority;
        saver->savingFunction = function;
        saver->getExtensions = getExtensions;

        // insert saver to a proper position of the list considering priorities
        list<SaverPtr>& savers = p->second->savers;
        list<SaverPtr>::iterator it = savers.begin();
        while(true){
            if(it == savers.end()){
                savers.push_back(saver);
                break;
            }
            SaverPtr saver2 = *it;
            if(saver->priority > saver2->priority){
                savers.insert(it, saver);
                break;
            }
            ++it;
        }

        registeredSavers.insert(saver);
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
    list<SaverPtr>& savers = classInfo->savers;
    SaverPtr targetSaver;
    
    if(useDialogToGetFilename){
        targetSaver = getSaverAndFilenameFromSaveDialog(savers, doExport, itemLabel, formatId, filename);
    } else {
        targetSaver = determineSaver(savers, filename, formatId);
    }
    
    if(targetSaver && targetSaver->savingFunction){
        
        tryToSave = true;
        
        if(!doExport){
            messageView->notify(format(_("Saving {0} to \"{1}\""), itemLabel, filename));
        } else {
            messageView->notify(format(_("Exporting {0} into \"{1}\""), itemLabel, filename));
        }
        
        Item* parentItem = item->parentItem();
        if(!parentItem){
            parentItem = RootItem::mainInstance();
        }

        ostream& os = messageView->cout(true);
        saved = (*targetSaver->savingFunction)(item, filename, os, parentItem);
        os.flush();
        
        if(!saved){
            messageView->put(MessageView::HIGHLIGHT, _(" -> failed.\n"));
        } else {
            bool isExporter = (targetSaver->priority <= ItemManager::PRIORITY_CONVERSION);
            if(!isExporter){
                item->updateFileInformation(filename, targetSaver->formatId);
            }
            messageView->put(_(" -> ok!\n"));
        }
    }
    
    if(!tryToSave){
        string actualFormatId = targetSaver ? targetSaver->formatId : formatId;
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


ItemManagerImpl::SaverPtr ItemManagerImpl::getSaverAndFilenameFromSaveDialog
(list<SaverPtr>& savers, bool doExport, const string& itemLabel, const string& formatId, string& io_filename)
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
    vector<SaverPtr> activeSavers;
    
    for(list<SaverPtr>::iterator p = savers.begin(); p != savers.end(); ++p){

        SaverPtr& saver = *p;

        if(saver->priority == ItemManager::PRIORITY_COMPATIBILITY){
            continue;
        }

        bool isExporter = (saver->priority <= ItemManager::PRIORITY_CONVERSION);
        if((doExport && !isExporter) || (!doExport && isExporter)){
            continue;
        }

        if(!formatId.empty() && saver->formatId != formatId){
            continue;
        }

        filters << makeExtensionFilter(saver->caption, saver->getExtensions(), true);
        
        activeSavers.push_back(saver);
    }

    dialog.setNameFilters(filters);

    SaverPtr targetSaver;

    if(filters.size() > 0){
    
        dialog.setDirectory(AppConfig::archive()->get
                            ("currentFileDialogDirectory", shareDirectory()).c_str());
    
        if(dialog.exec() == QFileDialog::Accepted){

            AppConfig::archive()->writePath(
                "currentFileDialogDirectory",
                dialog.directory().absolutePath().toStdString());

            io_filename = dialog.selectedFiles()[0].toStdString();
            if(!io_filename.empty()){
                int saverIndex = -1;
                QString selectedFilter = dialog.selectedNameFilter();
                for(int i=0; i < filters.size(); ++i){
                    if(filters[i] == selectedFilter){
                        saverIndex = i;
                        break;
                    }
                }
                if(saverIndex >= 0){
                    targetSaver = activeSavers[saverIndex];
                    string extensions = targetSaver->getExtensions();
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

    return targetSaver;
}


ItemManagerImpl::SaverPtr ItemManagerImpl::determineSaver
(list<SaverPtr>& savers, const string& filename, const string& formatId)
{
    SaverPtr targetSaver;

    if(!formatId.empty()){
        for(list<SaverPtr>::iterator p = savers.begin(); p != savers.end(); ++p){
            SaverPtr& saver = *p;
            if(saver->formatId == formatId){
                targetSaver = saver;
                break;
            }
        }
    } else {
        string dotextension = filesystem::path(filename).extension().string();
        if(!dotextension.empty()){
            string extension = dotextension.substr(1);
            for(list<SaverPtr>::iterator p = savers.begin(); p != savers.end(); ++p){
                SaverPtr& saver = *p;
                auto exts = separateExtensions(saver->getExtensions());
                for(auto& ext : exts){
                    if(ext == extension){
                        targetSaver = saver;
                        break;
                    }
                }
            }
        }
        if(!targetSaver && !savers.empty()){
            targetSaver = savers.front();
        }
    }

    return targetSaver;
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
    ItemManager::reloadItems(ItemTreeView::mainInstance()->selectedItems());
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
    const ItemList<>& selectedItems = ItemTreeView::mainInstance()->selectedItems();
    for(size_t i=0; i < selectedItems.size(); ++i){
        overwrite(selectedItems.get(i), true, "");
    }
}


void ItemManagerImpl::onSaveSelectedItemsAsActivated()
{
    const ItemList<>& selectedItems = ItemTreeView::mainInstance()->selectedItems();
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
    const ItemList<>& selectedItems = ItemTreeView::mainInstance()->selectedItems();
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
