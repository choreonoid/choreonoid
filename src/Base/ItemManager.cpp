/**
   @author Shin'ichiro Nakaoka
*/

#include "ItemManagerImpl.h"
#include <cnoid/RootItem>
#include <cnoid/ItemTreeView>
#include <cnoid/AppConfig>
#include <cnoid/MainWindow>
#include <cnoid/FileUtil>
#include <cnoid/ExecutablePath>
#include <cnoid/FileUtil>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QDialogButtonBox>
#include <QFileDialog>
#include <QRegExp>
#include <boost/bind.hpp>
#include <boost/tokenizer.hpp>
#include <boost/make_shared.hpp>
#include <sstream>
#include "gettext.h"

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

typedef map<string, ItemManagerImpl::LoaderPtr> LoaderMap;
LoaderMap extToLoaderMap;

QWidget* importMenu;

void expandExtensionsToVector(const string& extensions, vector<string>& out_extensions)
{
    typedef tokenizer< char_separator<char> > tokenizer;
    char_separator<char> sep(";");
    tokenizer tokens(extensions, sep);
    for(tokenizer::iterator it = tokens.begin(); it != tokens.end(); ++it){
        out_extensions.push_back(*it);
    }
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
    using boost::bind;
    
    if(!isStaticMembersInitialized){

        menuManager.setPath("/File").setPath(N_("New ..."));
        
        menuManager.setPath("/File");
        menuManager.addItem(_("Open Item"))
            ->sigTriggered().connect(bind(&ItemManagerImpl::onLoadItemActivated, this));
        menuManager.setPath(N_("Open ..."));
        menuManager.setPath("/File");
        menuManager.addItem(_("Reload Selected Items"))
            ->sigTriggered().connect(bind(&ItemManagerImpl::onReloadSelectedItemsActivated, this));
        
        menuManager.addSeparator();

        menuManager.addItem(_("Save Selected Items"))
            ->sigTriggered().connect(bind(&ItemManagerImpl::onSaveSelectedItemsActivated, this));
        menuManager.addItem(_("Save Selected Items As"))
            ->sigTriggered().connect(bind(&ItemManagerImpl::onSaveSelectedItemsAsActivated, this));
        menuManager.addItem(_("Save All Items"))
            ->sigTriggered().connect(bind(&ItemManagerImpl::onSaveAllItemsActivated, this));

        menuManager.addSeparator();
        
        menuManager.setPath(N_("Import ..."));
        importMenu = menuManager.current();
        
        menuManager.setPath("/File");
        menuManager.addItem(_("Export Selected Items"))
            ->sigTriggered().connect(bind(&ItemManagerImpl::onExportSelectedItemsActivated, this));
        
        menuManager.addSeparator();

        messageView = MessageView::mainInstance();

        isStaticMembersInitialized = true;
    }

    moduleNameToItemManagerImplMap[moduleName] = this;
}


void ItemManager::addMenuItemToImport(const std::string& caption, boost::function<void()> slot)
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
    for(set<ItemCreationPanel*>::iterator it = registeredCreationPanels.begin(); it != registeredCreationPanels.end(); ++it){
        ItemCreationPanel* panel = *it;
        delete panel;
    }
    
    // unregister loaders
    for(set<LoaderPtr>::iterator it = registeredLoaders.begin(); it != registeredLoaders.end(); ++it){

        LoaderPtr loader = *it;

        for(size_t i=0; i < loader->extensions.size(); ++i){
            const string& ext = loader->extensions[i];
            LoaderMap::iterator p = extToLoaderMap.find(ext);
            if(p != extToLoaderMap.end()){
                if(p->second == loader){
                    extToLoaderMap.erase(ext);
                }
            }
        }

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
    for(set<SaverPtr>::iterator it = registeredSavers.begin(); it != registeredSavers.end(); ++it){
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
    for(set<string>::iterator q = registeredTypeIds.begin(); q != registeredTypeIds.end(); ++q){

        const string& id = *q;

        Item::sigClassUnregistered_(id.c_str());

        CreationPanelBaseMap::iterator s = creationPanelBaseMap.find(id);
        if(s != creationPanelBaseMap.end()){
            CreationPanelBase* base = s->second;
            delete base;
            creationPanelBaseMap.erase(s);
        }

        typeIdToClassInfoMap.erase(id);
    }

    // unregister creation panel filters
    for(CreationPanelFilterSet::iterator p = registeredCreationPanelFilters.begin();
        p != registeredCreationPanelFilters.end(); ++p){

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


void ItemManager::registerClassSub(FactoryBase* factory, const std::string& typeId, const std::string& className)
{
    impl->registerClass(factory, typeId, className);
}


void ItemManagerImpl::registerClass(ItemManager::FactoryBase* factory, const string& typeId, const string& className)
{
    pair<ClassInfoMap::iterator, bool> ret = classNameToClassInfoMap.insert(make_pair(className, ClassInfoPtr()));
    ClassInfoPtr& info = ret.first->second;
    if(ret.second){
        info = boost::make_shared<ClassInfo>();
        info->moduleName = moduleName;
        info->className = className;
        info->factory = factory;
    } else {
        info->factory = factory;
    }
    registeredTypeIds.insert(typeId);
    typeIdToClassInfoMap[typeId] = info;
}


bool ItemManager::getClassIdentifier(ItemPtr item, std::string& out_moduleName, std::string& out_className)
{
    return ItemManagerImpl::getClassIdentifier(item, out_moduleName, out_className);
}


bool ItemManagerImpl::getClassIdentifier(ItemPtr item, string& out_moduleName, string& out_className)
{
    bool result;

    ClassInfoMap::iterator p = typeIdToClassInfoMap.find(typeid(*item).name());
    if(p != typeIdToClassInfoMap.end()){
        ClassInfoPtr& info = p->second;
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


ItemPtr ItemManager::create(const std::string& moduleName, const std::string& className)
{
    return ItemManagerImpl::create(moduleName, className);
}


ItemPtr ItemManagerImpl::create(const string& moduleName, const string& className)
{
    ItemPtr item;

    ModuleNameToItemManagerImplMap::iterator p = moduleNameToItemManagerImplMap.find(moduleName);
    if(p != moduleNameToItemManagerImplMap.end()){
        ClassInfoMap& classNameToClassInfoMap = p->second->classNameToClassInfoMap;
        ClassInfoMap::iterator q = classNameToClassInfoMap.find(className);
        if(q != classNameToClassInfoMap.end()){
            ClassInfoPtr& info = q->second;
            ItemManager::FactoryBase* factory = info->factory;
            if(factory){
                item = factory->create();
            }
        }
    }

    return item;
}


void ItemManager::addCreationPanelSub(const std::string& typeId, ItemCreationPanel* panel)
{
    impl->addCreationPanel(typeId, panel);
}


void ItemManagerImpl::addCreationPanel(const std::string& typeId, ItemCreationPanel* panel)
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
(const std::string& typeId, CreationPanelFilterBasePtr filter, bool afterInitializionByPanels)
{
    impl->addCreationPanelFilter(typeId, filter, afterInitializionByPanels);
}


void ItemManagerImpl::addCreationPanelFilter
(const std::string& typeId, ItemManager::CreationPanelFilterBasePtr filter, bool afterInitializionByPanels)
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
            QString className(info->className.c_str());
            QString translatedClassName(dgettext(textDomain.c_str(), info->className.c_str()));

            // set the class name without the "Item" suffix
            QString name(className);
            name.replace(QRegExp("Item$"), "");
            QString translatedName(translatedClassName);
            translatedName.replace(QRegExp(_("Item$")), "");
            
            QString title(QString(_("Create New %1")).arg(translatedClassName));
            
            ItemPtr protoItem = info->factory->create();
            protoItem->setName(name.toStdString());
            
            base = new CreationPanelBase(title, protoItem);
            base->hide();
            menuManager.setPath("/File/New ...").addItem(translatedName)
                ->sigTriggered().connect(boost::bind(ItemManagerImpl::onNewItemActivated, base));
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


ItemManagerImpl::CreationPanelBase::CreationPanelBase(const QString& title, ItemPtr protoItem)
    : QDialog(MainWindow::instance()),
      protoItem(protoItem)
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
        ItemManager::CreationPanelFilterBasePtr filter = *p;
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
                    ItemManager::CreationPanelFilterBasePtr filter = *p;
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
        newItem = protoItem->duplicate();
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
 const std::string& extensions, FileFunctionBasePtr function, int priority)
{
    impl->addLoader(typeId, caption, formatId, extensions, function, priority);
}


void ItemManagerImpl::addLoader
(const std::string& typeId, const std::string& caption, const std::string& formatId,
 const std::string& extensions, ItemManager::FileFunctionBasePtr function, int priority)
{
    ClassInfoMap::iterator p = typeIdToClassInfoMap.find(typeId);
    if(p != typeIdToClassInfoMap.end()){

        ClassInfoPtr& classInfo = p->second;
        
        LoaderPtr loader = boost::make_shared<Loader>();
        loader->typeId = typeId;
        loader->caption = caption;
        loader->formatId = formatId;
        loader->priority = priority;
        loader->loadingFunction = function;
        loader->classInfo = classInfo;

        expandExtensionsToVector(extensions, loader->extensions);

        for(size_t i=0; i < loader->extensions.size(); ++i){
            const string& ext = loader->extensions[i];
            extToLoaderMap[ext] = loader;
        }

        bool isImporter = (priority <= ItemManager::PRIORITY_CONVERSION);

        if(!isImporter){
            menuManager.setPath("/File/Open ...");
        } else {
            menuManager.setPath("/File/Import ...");
        }
        menuManager.addItem(caption.c_str())
            ->sigTriggered().connect(
                boost::bind(&ItemManagerImpl::onLoadSpecificTypeItemActivated, loader));
        
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


bool ItemManagerImpl::load(Item* item, const std::string& filename, Item* parentItem, const std::string& formatId)
{
    const string& typeId = typeid(*item).name();
    ClassInfoMap::iterator p = typeIdToClassInfoMap.find(typeId);
    if(p == typeIdToClassInfoMap.end()){
        messageView->putln(fmt(_("\"%1%\" cannot be loaded because item type \"%2%\" is not registered."))
                           % filename % typeId);
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
        string extension = filesystem::extension(filesystem::path(filename));
        if(extension.size() >= 2){
            string ext = extension.substr(1); // remove dot
            for(list<LoaderPtr>::iterator p = loaders.begin(); p != loaders.end(); ++p){
                LoaderPtr& loader = *p;
                for(size_t i=0; i < loader->extensions.size(); ++i){
                    if(ext == loader->extensions[i]){
                        targetLoader = loader;
                        break;
                    }
                }
            }
        }
    }

    if(!targetLoader){
        string message;
        if(formatId.empty()){
            message = str(fmt(_("\"%1%\" cannot be loaded because the file format is unknown."))
                          % filename);
        } else {
            message = str(fmt(_("\"%1%\" cannot be loaded because file format \"%2%\" is unknown."))
                          % filename % formatId);
        }
        messageView->putln(message);
    } else {
        if(load(targetLoader, item, filename, parentItem)){
            loaded = true;
        }
    }

    return loaded;
}
        

bool ItemManagerImpl::load(LoaderPtr loader, Item* item, const std::string& filename_, Item* parentItem)
{
    bool loaded = false;
    
    if(loader->loadingFunction){

        string filename(toActualPathName(filename_));
        
        format f(fmt(_("Loading %1% \"%2%\"")));
        messageView->notify(str(fmt(_("Loading %1% \"%2%\"")) % loader->caption % filename));
        messageView->flush();

        if(!parentItem){
            parentItem = RootItem::mainInstance();
        }
        if((*loader->loadingFunction)(item, filename, messageView->cout(true), parentItem)){
            if(item->name().empty()){
                item->setName(filesystem::basename(filesystem::path(filename)));
            }
            item->updateFileInformation(filename, loader->formatId);
            loaded = true;
        }

        messageView->put(loaded ? _(" -> ok!\n") : _(" -> failed.\n"));
        messageView->flush();
    }

    return loaded;
}


void ItemManagerImpl::onLoadItemActivated()
{
}


void ItemManagerImpl::onLoadSpecificTypeItemActivated(LoaderPtr loader)
{
    QFileDialog dialog(MainWindow::instance());
    dialog.setWindowTitle(str(fmt(_("Load %1%")) % loader->caption).c_str());
    dialog.setFileMode(QFileDialog::ExistingFiles);
    dialog.setViewMode(QFileDialog::List);
    dialog.setLabelText(QFileDialog::Accept, _("Open"));
    dialog.setLabelText(QFileDialog::Reject, _("Cancel"));
    
    dialog.setDirectory(AppConfig::archive()->get
                        ("currentFileDialogDirectory", shareDirectory()).c_str());

    QStringList filters;
    if(!loader->extensions.empty()){
        string filterText = loader->caption + " (";
        for(size_t i=0; i < loader->extensions.size(); ++i){
            if(i > 0){
                filterText += " ";
            }
            string extension = string("*.") + loader->extensions[i];
            filterText += extension;
        }
        filterText += ")";
        filters << filterText.c_str();
    }
    filters << _("Any files (*)");
    dialog.setNameFilters(filters);
    
    if(dialog.exec()){
        AppConfig::archive()->writePath(
            "currentFileDialogDirectory",
            dialog.directory().absolutePath().toStdString());

        QStringList filenames = dialog.selectedFiles();

        Item* parentItem = ItemTreeView::mainInstance()->selectedItem<Item>();
        if(!parentItem){
            parentItem = RootItem::mainInstance();
        }

        ClassInfoPtr classInfo = loader->classInfo.lock();
        ItemManager::FactoryBase* factory = classInfo->factory;
        for(int i=0; i < filenames.size(); ++i){
            ItemPtr item = factory->create();
            string filename = getNativePathString(filesystem::path(filenames[i].toStdString()));
            if(load(loader, item.get(), filename, parentItem)){
                parentItem->addChildItem(item, true);
            }
        }
    }
}


void ItemManager::addSaverSub(const std::string& typeId, const std::string& caption, const std::string& formatId,
                              const std::string& extensions, FileFunctionBasePtr function, int priority)
{
    impl->addSaver(typeId, caption, formatId, extensions, function, priority);
}


void ItemManagerImpl::addSaver
(const string& typeId, const string& caption, const string& formatId, const string& extensions,
 ItemManager::FileFunctionBasePtr function, int priority)
{
    ClassInfoMap::iterator p = typeIdToClassInfoMap.find(typeId);
    if(p != typeIdToClassInfoMap.end()){

        SaverPtr saver = boost::make_shared<Saver>();
        
        saver->typeId = typeId;
        saver->formatId = formatId;
        saver->caption = caption;
        saver->priority = priority;
        saver->savingFunction = function;

        expandExtensionsToVector(extensions, saver->extensions);

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
(Item* item, bool useDialogToGetFilename, bool doExport, std::string filename, const std::string& formatId)
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
            messageView->put(str(fmt(_("Saving %1% to \"%2%\"")) % itemLabel % filename));
        } else {
            messageView->put(str(fmt(_("Exporting %1% into \"%2%\"")) % itemLabel % filename));
        }
        
        Item* parentItem = item->parentItem();
        if(!parentItem){
            parentItem = RootItem::mainInstance();
        }

        if((*targetSaver->savingFunction)(item, filename, messageView->cout(true), parentItem)){
            saved = true;
            bool isExporter = (targetSaver->priority <= ItemManager::PRIORITY_CONVERSION);
            if(!isExporter){
                item->updateFileInformation(filename, targetSaver->formatId);
            }
        }
        
        messageView->put(saved ? _(" -> ok!\n") : _(" -> failed.\n"));
    }
    
    if(!tryToSave){
        string actualFormatId = targetSaver ? targetSaver->formatId : formatId;
        if(actualFormatId.empty()){
            if(!doExport){
                messageView->put(str(fmt(_("%1% cannot be saved.\n")) % itemLabel));
            } else {
                messageView->put(str(fmt(_("%1% cannot be exported.\n")) % itemLabel));
            }
        } else {
            if(!doExport){
                messageView->put(str(fmt(_("%1% cannot be saved as the %2% format.\n")) % itemLabel % actualFormatId));
            } else {
                messageView->put(str(fmt(_("%1% cannot be exported into the %2% format.\n")) % itemLabel % actualFormatId));
            }
        }
    }

    return saved;
}


ItemManagerImpl::SaverPtr ItemManagerImpl::getSaverAndFilenameFromSaveDialog
(list<SaverPtr>& savers, bool doExport, const string& itemLabel, const string& formatId, string& io_filename)
{
    QFileDialog dialog(MainWindow::instance());
    dialog.setWindowTitle(str(fmt(_("Save %1% as")) % itemLabel).c_str());
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

        bool isExporter = (saver->priority <= ItemManager::PRIORITY_CONVERSION);
        if((doExport && !isExporter) || (!doExport && isExporter)){
            continue;
        }

        if(!formatId.empty() && saver->formatId != formatId){
            continue;
        }

        string filterText = saver->caption + " (";
        if(saver->extensions.empty()){
            filterText += "*";
        } else {
            for(size_t i=0; i < saver->extensions.size(); ++i){
                if(i > 0){
                    filterText += " ";
                }
                string extension = string("*.") + saver->extensions[i];
                filterText += extension;
            }
        }
        filterText += ")";
        filters << filterText.c_str();
        
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
                    // add a lacking extension automatically
                    if(!targetSaver->extensions.empty()){
                        string ext = filesystem::extension(filesystem::path(io_filename));
                        bool extLacking = true;
                        if(!ext.empty()){
                            ext = ext.substr(1); // remove the first dot
                            for(size_t i=0; i < targetSaver->extensions.size(); ++i){
                                if(ext == targetSaver->extensions[i]){
                                    extLacking = false;
                                    break;
                                }
                            }
                        }
                        if(extLacking){
                            io_filename += ".";
                            io_filename += targetSaver->extensions.front();
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
        string extension = filesystem::extension(filesystem::path(filename));
        for(list<SaverPtr>::iterator p = savers.begin(); p != savers.end(); ++p){
            SaverPtr& saver = *p;
            for(size_t i=0; i < saver->extensions.size(); ++i){
                if(saver->extensions[i] == extension){
                    targetSaver = saver;
                    break;
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


bool ItemManagerImpl::overwrite(Item* item, bool forceOverwrite, const std::string& formatId)
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
               filesystem::last_write_time(fpath) > item->fileModificationTime()){
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
    for(size_t i=0; i < items.size(); ++i){

        Item* item = items.get(i);

        if(item->parentItem() && !item->isSubItem() &&
           !item->filePath().empty() && !item->fileFormat().empty()){

            ItemPtr reloaded = item->duplicate();
            if(reloaded){
                if(reloaded->load(item->filePath(), item->parentItem(), item->fileFormat())){

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
                    messageView->putln(format(_("\"%1%\" has been reloaded.")) % reloaded->name());
                    
                    item->detachFromParentItem();
                }
            }
        }
    }
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

QString makeFilterString(const std::string& caption, const std::string& extensions_)
{
    vector<string> extensions;
    expandExtensionsToVector(extensions_, extensions);
        
    QString filter;
    if(!extensions.empty()){
        filter = caption.c_str();
        filter += " (";
        for(size_t i=0; i < extensions.size(); ++i){
            if(i > 0){
                filter += " ";
            }
            filter += "*.";
            filter += extensions[i].c_str();
        }
        filter += ")";
    }
    filter += _(";;Any files (*)");
        
    return filter;
}
    
std::string getOpenFileName(const std::string& caption, const std::string& extensions)
{
    QString qfilename =
        QFileDialog::getOpenFileName(
            MainWindow::instance(),
            caption.c_str(),
            AppConfig::archive()->get("currentFileDialogDirectory", shareDirectory()).c_str(),
            makeFilterString(caption, extensions));

    string filename = qfilename.toStdString();

    if(!filename.empty()){
        AppConfig::archive()->writePath(
            "currentFileDialogDirectory",
            filesystem::path(filename).parent_path().string());
    }

    return filename;
}
    
std::vector<std::string> getOpenFileNames(const std::string& caption, const std::string& extensions)
{
    QStringList qfilenames =
        QFileDialog::getOpenFileNames(
            MainWindow::instance(),
            caption.c_str(),
            AppConfig::archive()->get("currentFileDialogDirectory", shareDirectory()).c_str(),
            makeFilterString(caption, extensions));

    std::vector<std::string> filenames;

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
