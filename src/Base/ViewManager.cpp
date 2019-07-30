/**
   @author Shin'ichiro Nakaoka
*/

#include "ViewManager.h"
#include "PluginManager.h"
#include "MenuManager.h"
#include "MainWindow.h"
#include "ViewArea.h"
#include "MessageView.h"
#include "Dialog.h"
#include "Archive.h"
#include <QBoxLayout>
#include <QLineEdit>
#include <QLabel>
#include <QDialogButtonBox>
#include <QPushButton>
#include <fmt/format.h>
#include <list>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

MainWindow* mainWindow = nullptr;
Menu* showViewMenu = nullptr;
Menu* createViewMenu = nullptr;
Menu* deleteViewMenu = nullptr;

Signal<void(View* view)> sigViewCreated_;
Signal<void(View* view)> sigViewActivated_;
Signal<void(View* view)> sigViewDeactivated_;
Signal<void(View* view)> sigViewRemoved_;

class ViewInfo;
typedef std::shared_ptr<ViewInfo> ViewInfoPtr;

typedef map<ViewInfo*, vector<View*>> ViewInfoToViewsMap;

class InstanceInfo;
typedef std::shared_ptr<InstanceInfo> InstanceInfoPtr;
typedef list<InstanceInfoPtr> InstanceInfoList;

class InstanceInfo {
public:
    View* view;
    ViewInfo* viewInfo;
    InstanceInfoList::iterator iterInViewInfo;
    InstanceInfoList::iterator iterInViewManager;

    InstanceInfo(ViewInfo* viewInfo, View* view) : view(view), viewInfo(viewInfo) {
    }
    ~InstanceInfo();
    void remove();
};

class ViewInfo : public ViewClass
{
public:
    const std::type_info& view_type_info;
    string className_;
    string textDomain;
    string translatedClassName;
    string defaultInstanceName;
    string translatedDefaultInstanceName; // temporary.
    ViewManager::InstantiationType itype;
    ViewManager::FactoryBase* factory;

    InstanceInfoList instances;
    InstanceInfoList& instancesInViewManager;

    virtual const std::string& className() const { return className_; }

    ViewInfo(ViewManagerImpl* managerImpl,
             const type_info& view_type_info, const string& className, const string& defaultInstanceName,
             const string& textDomain, ViewManager::InstantiationType itype, ViewManager::FactoryBase* factory);

    ~ViewInfo(){
        delete factory;
    }

    bool hasDefaultInstance() const {
        return itype == ViewManager::SINGLE_DEFAULT || itype == ViewManager::MULTI_DEFAULT;
    }

    bool isSingleton() const {
        return itype == ViewManager::SINGLE_DEFAULT || itype == ViewManager::SINGLE_OPTIONAL;
    }

    bool checkIfDefaultInstance(View* view){
        return hasDefaultInstance() && !instances.empty() && (instances.front()->view == view);
    }

    bool checkIfPrimalInstance(View* view){
        return !instances.empty() && (instances.front()->view == view);
    }

private:
    View* createView() {
        View* view = factory->create();
        InstanceInfoPtr instance = std::make_shared<InstanceInfo>(this, view);
            
        instances.push_back(instance);
        instance->iterInViewInfo = instances.end();
        --instance->iterInViewInfo;
            
        instancesInViewManager.push_back(instance);
        instance->iterInViewManager = instancesInViewManager.end();
        --instance->iterInViewManager;

        view->sigActivated().connect([view](){ sigViewActivated_(view); });
        view->sigDeactivated().connect([view](){ sigViewDeactivated_(view); });

        return view;
    }

public:
    View* getOrCreateView(bool doMountCreatedView = false){
        if(instances.empty()){
            View* view = createView();
            view->setName(defaultInstanceName);
            view->setWindowTitle(translatedDefaultInstanceName.c_str());
            sigViewCreated_(view);
            if(doMountCreatedView){
                mainWindow->viewArea()->addView(view);
            }
        }
        return instances.front()->view;
    }
        
    View* createView(const string& name, bool setTranslatedNameToWindowTitle = false){
        if(name.empty()){
            return nullptr;
        }
        View* view = createView();
        view->setName(name);
        if(setTranslatedNameToWindowTitle){
            view->setWindowTitle(dgettext(textDomain.c_str(), name.c_str()));
        }
        sigViewCreated_(view);
        return view;
    }

    View* findView(const string& name){
        if(name.empty()){
            return instances.empty() ? 0 : instances.front()->view;
        } else {
            for(InstanceInfoList::iterator p = instances.begin(); p != instances.end(); ++p){
                if((*p)->view->name() == name){
                    return (*p)->view;
                }
            }
            return nullptr;
        }
    }

    View* getOrCreateView(const string& name, bool doMountCreatedView = false){
        View* view = findView(name);
        if(!view){
            view = createView(name);
            sigViewCreated_(view);
            if(doMountCreatedView){
                mainWindow->viewArea()->addView(view);
            }
        }
        return view;
    }
};

struct compare_type_info {
    bool operator ()(const type_info* a, const type_info* b) const {
        return a->before(*b);
    }
};
typedef std::map<const type_info*, ViewInfoPtr, compare_type_info> TypeToViewInfoMap;
TypeToViewInfoMap typeToViewInfoMap;

typedef map<string, ViewInfoPtr> ClassNameToViewInfoMap;
typedef std::shared_ptr<ClassNameToViewInfoMap> ClassNameToViewInfoMapPtr;

typedef map<string, ClassNameToViewInfoMapPtr> ModuleNameToClassNameToViewInfoMap;
ModuleNameToClassNameToViewInfoMap moduleNameToClassNameToViewInfoMap;

map<string, string> classNameAliasMap;

}


namespace cnoid {

class ViewManagerImpl
{
public:
    const string& moduleName;
    const string& textDomain;
    MenuManager& menuManager;
    ClassNameToViewInfoMapPtr classNameToViewInfoMap;
    InstanceInfoList instances;

    ViewManagerImpl(ExtensionManager* ext);
    ~ViewManagerImpl();
    static void notifySigRemoved(View* view){
        view->notifySigRemoved();
    }
    static void deactivateView(View* view){
        if(view->isActive()){
            view->onDeactivated();
        }
    }
};

}


ViewInfo::ViewInfo
(ViewManagerImpl* managerImpl,
 const type_info& view_type_info, const string& className, const string& defaultInstanceName,
 const string& textDomain, ViewManager::InstantiationType itype, ViewManager::FactoryBase* factory)
    : view_type_info(view_type_info),
      className_(className),
      textDomain(textDomain),
      defaultInstanceName(defaultInstanceName),
      itype(itype),
      factory(factory),
      instancesInViewManager(managerImpl->instances)
{
    translatedClassName = dgettext(textDomain.c_str(), className_.c_str());
    translatedDefaultInstanceName = dgettext(textDomain.c_str(), defaultInstanceName.c_str());
}


namespace {

InstanceInfo::~InstanceInfo()
{
    if(view){
        ViewManagerImpl::deactivateView(view);
        delete view;
    }
}

void InstanceInfo::remove()
{
    if(view){
        ViewManagerImpl::deactivateView(view);
        ViewManagerImpl::notifySigRemoved(view);
        sigViewRemoved_(view);
        delete view;
        view = nullptr;
    }
    viewInfo->instances.erase(iterInViewInfo);
    iterInViewInfo = viewInfo->instances.end();
    viewInfo->instancesInViewManager.erase(iterInViewManager);
    //iterInViewManager = viewInfo->instancesInViewManager.end();
}


class ViewCreationDialog : public Dialog
{
public:
    QLineEdit nameEdit;
        
    ViewCreationDialog(ViewInfoPtr viewInfo) {

        QVBoxLayout* vbox = new QVBoxLayout();
            
        QHBoxLayout* hbox = new QHBoxLayout();
        hbox->addWidget(new QLabel(_("Name:")));
        hbox->addWidget(&nameEdit);
        vbox->addLayout(hbox);
        
        QDialogButtonBox* buttonBox = new QDialogButtonBox(this);
        
        QPushButton* createButton = new QPushButton(_("&Create"));
        createButton->setDefault(true);
        buttonBox->addButton(createButton, QDialogButtonBox::AcceptRole);
        connect(buttonBox,SIGNAL(accepted()), this, SLOT(accept()));
        
        QPushButton* cancelButton = new QPushButton(_("&Cancel"));
        buttonBox->addButton(cancelButton, QDialogButtonBox::RejectRole);
        connect(buttonBox,SIGNAL(rejected()), this, SLOT(reject()));
        
        vbox->addWidget(buttonBox);
        setLayout(vbox);
        
        setWindowTitle(QString(_("Create %1")).arg(viewInfo->translatedClassName.c_str()));
        
        if(viewInfo->instances.empty()){
            nameEdit.setText(viewInfo->translatedDefaultInstanceName.c_str());
        } else {
            nameEdit.setText(
                QString("%1 %2")
                .arg(viewInfo->translatedDefaultInstanceName.c_str())
                .arg(viewInfo->instances.size() + 1));
        }
    }
};
    

void onShowViewToggled(ViewInfoPtr viewInfo, View* view, bool on)
{
    if(on){
        if(!view){
            view = viewInfo->getOrCreateView();
        }
        mainWindow->viewArea()->addView(view);
        view->bringToFront();
    } else {
        if(view->viewArea()){
            view->viewArea()->removeView(view);
        }
    }
}

void onCreateViewTriggered(ViewInfoPtr viewInfo)
{
    ViewCreationDialog dialog(viewInfo);

    while(dialog.exec() == QDialog::Accepted){
        string name = dialog.nameEdit.text().toStdString();
        if(name.empty()){
            showWarningDialog(_("Please specify the name of the new view."));
        } else {
            View* view = viewInfo->createView(name);
            mainWindow->viewArea()->addView(view);
            view->bringToFront();
            break;
        }
    }
}

void onDeleteViewTriggered(InstanceInfoPtr instance)
{
    instance->remove();
}

void deleteAllInvisibleViews()
{
    for(TypeToViewInfoMap::iterator p = typeToViewInfoMap.begin(); p != typeToViewInfoMap.end(); ++p){
        ViewInfo& info = *p->second;
        InstanceInfoList::iterator q = info.instances.begin();
        while(q != info.instances.end()){
            InstanceInfoPtr& instance = (*q++);
            if(!instance->view->viewArea()){
                instance->remove();
            }
        }
    }
}
    
void onViewMenuAboutToShow(Menu* menu)
{
    menu->clear();
    bool needSeparator = false;
        
    ModuleNameToClassNameToViewInfoMap::iterator p;

    if(menu == deleteViewMenu){
        Action* action = new Action(menu);
        action->setText(_("Delete All Invisible Views"));
        action->sigTriggered().connect([](){ deleteAllInvisibleViews(); });
        menu->addAction(action);
        needSeparator = true;
    }
        
    for(p = moduleNameToClassNameToViewInfoMap.begin(); p != moduleNameToClassNameToViewInfoMap.end(); ++p){

        if(needSeparator){
            QAction* separator = new QAction(menu);
            separator->setSeparator(true);
            menu->addAction(separator);
            needSeparator = false;
        }
            
        ClassNameToViewInfoMap& viewInfoMap = *p->second;
        ClassNameToViewInfoMap::iterator q;
        for(q = viewInfoMap.begin(); q != viewInfoMap.end(); ++q){
            ViewInfoPtr& viewInfo = q->second;
            InstanceInfoList& instances = viewInfo->instances;

            if(menu == showViewMenu){
                View* view = nullptr;
                if(instances.empty()){
                    Action* action = new Action(menu);
                    action->setText(viewInfo->translatedDefaultInstanceName.c_str());
                    action->setCheckable(true);
                    action->sigToggled().connect([=](bool on){ onShowViewToggled(viewInfo, view, on); });
                    menu->addAction(action);
                } else {
                    for(InstanceInfoList::iterator p = instances.begin(); p != instances.end(); ++p){
                        InstanceInfoPtr& instance = (*p);
                        view = instance->view;
                        Action* action = new Action(menu);
                        action->setText(view->windowTitle());
                        action->setCheckable(true);
                        action->setChecked(view->viewArea());
                        action->sigToggled().connect([=](bool on){ onShowViewToggled(viewInfo, view, on); });
                        menu->addAction(action);
                    }
                }
            } else if(menu == createViewMenu){
                if((viewInfo->itype == ViewManager::SINGLE_OPTIONAL && viewInfo->instances.empty()) ||
                   (viewInfo->itype == ViewManager::MULTI_DEFAULT || viewInfo->itype == ViewManager::MULTI_OPTIONAL)){
                    Action* action = new Action(menu);
                    action->setText(viewInfo->translatedDefaultInstanceName.c_str());
                    action->sigTriggered().connect([=](){ onCreateViewTriggered(viewInfo); });
                    menu->addAction(action);
                }
            } else if(menu == deleteViewMenu){
                InstanceInfoList::iterator p = instances.begin();
                if(viewInfo->hasDefaultInstance() && p != instances.end()){
                    ++p;
                }
                while(p != instances.end()){
                    InstanceInfoPtr& instance = (*p++);
                    Action* action = new Action(menu);
                    action->setText(instance->view->windowTitle());
                    action->sigTriggered().connect([=](){ onDeleteViewTriggered(instance); });
                    menu->addAction(action);
                }
            }
            needSeparator = true;
        }
    }
}
}


void ViewManager::initializeClass(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        
        mainWindow = MainWindow::instance();
        MenuManager& mm = ext->menuManager();
        QWidget* viewMenu = mm.setPath("/View").current();

        QAction* showViewAction = mm.findItem("Show View");
        showViewMenu = new Menu(viewMenu);
        showViewMenu->sigAboutToShow().connect([=](){ onViewMenuAboutToShow(showViewMenu); });
        showViewAction->setMenu(showViewMenu);

        QAction* createViewAction = mm.setCurrent(viewMenu).findItem("Create View");
        createViewMenu = new Menu(viewMenu);
        createViewMenu->sigAboutToShow().connect([=](){ onViewMenuAboutToShow(createViewMenu); });
        createViewAction->setMenu(createViewMenu);

        QAction* deleteViewAction = mm.setCurrent(viewMenu).findItem("Delete View");
        deleteViewMenu = new Menu(viewMenu);
        deleteViewMenu->sigAboutToShow().connect([=](){ onViewMenuAboutToShow(deleteViewMenu); });
        deleteViewAction->setMenu(deleteViewMenu);
        
        initialized = true;
    }
}


ViewManager::ViewManager(ExtensionManager* ext)
{
    impl = new ViewManagerImpl(ext);
}


ViewManagerImpl::ViewManagerImpl(ExtensionManager* ext)
    : moduleName(ext->name()),
      textDomain(ext->textDomain()),
      menuManager(ext->menuManager())
{
    classNameToViewInfoMap = std::make_shared<ClassNameToViewInfoMap>();
    moduleNameToClassNameToViewInfoMap[moduleName] = classNameToViewInfoMap;
}


ViewManager::~ViewManager()
{
    // This must be done before deleting impl because
    // ViewManager's functions may be called when a view is destroyed.
    while(!impl->instances.empty()){
        InstanceInfoPtr& instance = *(impl->instances.rbegin());
        instance->remove();
    };

    delete impl;
}


ViewManagerImpl::~ViewManagerImpl()
{
    ClassNameToViewInfoMap::iterator p;
    for(p = classNameToViewInfoMap->begin(); p != classNameToViewInfoMap->end(); ++p){
        ViewInfoPtr& info = p->second;
        typeToViewInfoMap.erase(&info->view_type_info);
    }
    
    moduleNameToClassNameToViewInfoMap.erase(moduleName);
    showViewMenu->clear();
    createViewMenu->clear();
}


View* ViewManager::registerClassSub
(const type_info& view_type_info, const std::string& className, const std::string& defaultInstanceName,
 ViewManager::InstantiationType itype, FactoryBase* factory)
{
    ViewInfoPtr info = std::make_shared<ViewInfo>(
        impl, view_type_info, className, defaultInstanceName, impl->textDomain, itype, factory);
    
    (*impl->classNameToViewInfoMap)[className] = info;
    typeToViewInfoMap[&view_type_info] = info;

    if(itype == ViewManager::SINGLE_DEFAULT || itype == ViewManager::MULTI_DEFAULT){
        View* view = info->getOrCreateView();
        mainWindow->viewArea()->addView(view);
        return view;
    }
    return nullptr;
}


void ViewManager::registerClassAlias(const std::string& alias, const std::string& orgClassName)
{
    classNameAliasMap[alias] = orgClassName;
}


ViewClass* ViewManager::viewClass(const std::type_info& view_type_info)
{
    ViewClass* viewClass = nullptr;
    TypeToViewInfoMap::iterator p = typeToViewInfoMap.find(&view_type_info);
    if(p != typeToViewInfoMap.end()){
        viewClass = p->second.get();
    }
    return viewClass;
}


namespace {
ViewInfo* findViewInfo(const std::string& moduleName, const std::string& className)
{
    ViewInfo* info = nullptr;
    auto p = moduleNameToClassNameToViewInfoMap.find(moduleName);
    if(p != moduleNameToClassNameToViewInfoMap.end()){
        auto& infoMap = *p->second;
        auto q = infoMap.find(className);
        if(q == infoMap.end()){
            auto r = classNameAliasMap.find(className);
            if(r != classNameAliasMap.end()){
                q = infoMap.find(r->second);
            }
        }
        if(q != infoMap.end()){
            info = q->second.get();
        }
    }
    return info;
}
}
        

View* ViewManager::getOrCreateView(const std::string& moduleName, const std::string& className)
{
    ViewInfo* info = findViewInfo(moduleName, className);
    if(info){
        return info->getOrCreateView();
    }
    return nullptr;
}


View* ViewManager::getOrCreateView(const std::string& moduleName, const std::string& className, const std::string& instanceName)
{
    ViewInfo* info = findViewInfo(moduleName, className);
    if(info){
        return info->getOrCreateView(instanceName);
    }
    return nullptr;
}


/**
   This is implemented for the compatibility to version 1.4 or earlier.
   This is only used for loading the view layout configuration created by those verions.
*/
View* ViewManager::getOrCreateViewOfDefaultName(const std::string& defaultName)
{
    for(TypeToViewInfoMap::iterator p = typeToViewInfoMap.begin(); p != typeToViewInfoMap.end(); ++p){
        ViewInfo& info = *p->second;
        if(info.defaultInstanceName == defaultName){
            return info.getOrCreateView();
        }
    }
    return nullptr;
}


std::vector<View*> ViewManager::allViews()
{
    std::vector<View*> views;
    for(TypeToViewInfoMap::iterator p = typeToViewInfoMap.begin(); p != typeToViewInfoMap.end(); ++p){
        InstanceInfoList& instances = p->second->instances;
        for(InstanceInfoList::iterator p = instances.begin(); p != instances.end(); ++p){
            views.push_back((*p)->view);
        }
    }
    return views;
}


std::vector<View*> ViewManager::activeViews()
{
    std::vector<View*> views;
    for(TypeToViewInfoMap::iterator p = typeToViewInfoMap.begin(); p != typeToViewInfoMap.end(); ++p){
        InstanceInfoList& instances = p->second->instances;
        for(InstanceInfoList::iterator p = instances.begin(); p != instances.end(); ++p){
            View* view = (*p)->view;
            if(view->isActive()){
                views.push_back(view);
            }
        }
    }
    return views;
}


View* ViewManager::getOrCreateSpecificTypeView
(const std::type_info& view_type_info, const std::string& instanceName, bool doMountCreatedView)
{
    TypeToViewInfoMap::iterator p = typeToViewInfoMap.find(&view_type_info);
    if(p != typeToViewInfoMap.end()){
        ViewInfo& info = *p->second;
        if(instanceName.empty()){
            return info.getOrCreateView(doMountCreatedView);
        } else {
            return info.getOrCreateView(instanceName, doMountCreatedView);
        }
    }
    return nullptr;
}


View* ViewManager::findSpecificTypeView(const std::type_info& view_type_info, const std::string& instanceName)
{
    TypeToViewInfoMap::iterator p = typeToViewInfoMap.find(&view_type_info);
    if(p != typeToViewInfoMap.end()){
        ViewInfo& info = *p->second;
        return info.findView(instanceName);
    }
    return nullptr;
}    


void ViewManager::deleteView(View* view)
{
    for(auto&& info : impl->instances){
        if(info->view == view){
            info->remove();
            break;
        }
    }
}


bool ViewManager::isPrimalInstance(View* view)
{
    TypeToViewInfoMap::iterator p = typeToViewInfoMap.find(&typeid(*view));
    if(p != typeToViewInfoMap.end()){
        ViewInfo& info = *p->second;
        return info.checkIfPrimalInstance(view);
    }
    return false;
}


namespace {

ArchivePtr storeView(Archive& parentArchive, const string& moduleName, ViewInfo& viewInfo, View* view)
{
    ArchivePtr archive;
        
    ArchivePtr state = new Archive();
    state->inheritSharedInfoFrom(parentArchive);

    if(view->storeState(*state)){
            
        archive = new Archive();
        archive->inheritSharedInfoFrom(parentArchive);

        archive->write("id", state->getViewId(view));
            
        if(!viewInfo.checkIfDefaultInstance(view)){
            archive->write("name", view->name(), DOUBLE_QUOTED);
        }
        archive->write("plugin", moduleName);
        archive->write("class", viewInfo.className_);

        if(view->viewArea()){
            archive->write("mounted", true);
        }

        if(!state->empty()){
            archive->insert("state", state);
        }
    }

    return archive;
}
}


bool ViewManager::storeViewStates(ArchivePtr archive, const std::string& key)
{
    // assign view ids first
    int id = 0;
    ModuleNameToClassNameToViewInfoMap::iterator p;
    for(p = moduleNameToClassNameToViewInfoMap.begin(); p != moduleNameToClassNameToViewInfoMap.end(); ++p){
        ClassNameToViewInfoMap& viewInfoMap = *p->second;
        for(ClassNameToViewInfoMap::iterator q = viewInfoMap.begin(); q != viewInfoMap.end(); ++q){
            ViewInfoPtr& viewInfo = q->second;
            InstanceInfoList& instances = viewInfo->instances;
            for(InstanceInfoList::iterator p = instances.begin(); p != instances.end(); ++p){            
                archive->registerViewId((*p)->view, id++);
            }
        }
    }
    
    ListingPtr viewList = new Listing();

    for(p = moduleNameToClassNameToViewInfoMap.begin(); p != moduleNameToClassNameToViewInfoMap.end(); ++p){
        const std::string& moduleName = p->first;
        ClassNameToViewInfoMap& viewInfoMap = *p->second;
        for(ClassNameToViewInfoMap::iterator q = viewInfoMap.begin(); q != viewInfoMap.end(); ++q){
            ViewInfoPtr& viewInfo = q->second;
            InstanceInfoList& instances = viewInfo->instances;
            for(InstanceInfoList::iterator p = instances.begin(); p != instances.end(); ++p){            
                View* view = (*p)->view;
                ArchivePtr viewArchive = storeView(*archive, moduleName, *viewInfo, view);
                if(viewArchive){
                    viewList->append(viewArchive);
                }
            }
        }
    }

    if(!viewList->empty()){
        archive->insert(key, viewList);
        return true;
    }
    return false;
}


namespace {

struct ViewState {
    View* view;
    ArchivePtr state;
    ViewState(View* view, ArchivePtr state) : view(view), state(state) { }
};

}


ViewManager::ViewStateInfo::ViewStateInfo()
{
    data = nullptr;
}


ViewManager::ViewStateInfo::~ViewStateInfo()
{
    if(data){
        delete reinterpret_cast< vector<ViewState>* >(data);
    }
}


static View* restoreView(Archive* archive, const string& moduleName, const string& className, ViewInfoToViewsMap& remainingViewsMap)
{
    View* view = nullptr;
    string instanceName;
                    
    if(!archive->read("name", instanceName)){
        view = ViewManager::getOrCreateView(moduleName, className);
    } else {
        // get one of the view instances having the instance name, or create a new instance.
        // Different instances are assigned even if there are instances with the same name in the archive
        ViewInfo* info = findViewInfo(moduleName, className);
        if(info){
            vector<View*>* remainingViews;

            auto p = remainingViewsMap.find(info);
            if(p != remainingViewsMap.end()){
                remainingViews = &p->second;
            } else {
                remainingViews = &remainingViewsMap[info];
                InstanceInfoList& instances = info->instances;
                remainingViews->reserve(instances.size());
                InstanceInfoList::iterator q = instances.begin();
                if(info->hasDefaultInstance() && q != instances.end()){
                    ++q;
                }
                while(q != instances.end()){
                    remainingViews->push_back((*q++)->view);
                }
            }
            for(vector<View*>::iterator q = remainingViews->begin(); q != remainingViews->end(); ++q){
                if((*q)->name() == instanceName){
                    view = *q;
                    remainingViews->erase(q);
                    break;
                }
            }
            if(!view){
                if(!info->isSingleton() || info->instances.empty()){
                    view = info->createView(instanceName, true);
                } else {
                    MessageView::instance()->putln(
                        MessageView::ERROR,
                        fmt::format(
                            _("A singleton view \"{0}\" of the {1} type cannot be created because its singleton instance has already been created."),
                        instanceName, info->className()));
                }
            }
        }
    }

    return view;
}


bool ViewManager::restoreViews(ArchivePtr archive, const std::string& key, ViewManager::ViewStateInfo& out_viewStateInfo)
{
    bool restored = false;
    
    Listing* viewList = archive->findListing(key);
    
    if(viewList->isValid() && !viewList->empty()){

        ViewInfoToViewsMap remainingViewsMap;
        vector<ViewState>* viewsToRestoreState = new vector<ViewState>();        
        out_viewStateInfo.data = viewsToRestoreState;
        int id;
        string moduleName;
        string className;
        
        for(int i=0; i < viewList->size(); ++i){
            Archive* viewArchive = dynamic_cast<Archive*>(viewList->at(i)->toMapping());
            if(viewArchive){
                bool isHeaderValid =
                    viewArchive->read("id", id) &&
                    viewArchive->read("plugin", moduleName) &&
                    viewArchive->read("class", className);
            
                if(isHeaderValid){
                    const char* actualModuleName = PluginManager::instance()->guessActualPluginName(moduleName);
                    if(!actualModuleName){
                        MessageView::instance()->putln(
                            MessageView::ERROR,
                            fmt::format(_("The \"{0}\" plugin for \"{1}\" is not found. The view cannot be restored."),
                            moduleName, className));
                        
                    } else {
                        View* view = restoreView(viewArchive, actualModuleName, className, remainingViewsMap);
                        if(view){
                            archive->registerViewId(view, id);
                        
                            ArchivePtr state = viewArchive->findSubArchive("state");
                            if(state->isValid()){
                                state->inheritSharedInfoFrom(*archive);
                                viewsToRestoreState->push_back(ViewState(view, state));
                            }

                            if(viewArchive->get("mounted", false)){
                                mainWindow->viewArea()->addView(view);
                            }
                            restored = true;
                        }
                    }
                }
            }
        }
    }
    return restored;
}


bool ViewManager::restoreViewStates(ViewStateInfo& info)
{
    if(info.data){
        vector<ViewState>* viewsToRestoreState = reinterpret_cast<vector<ViewState>*>(info.data);
        for(size_t i=0; i < viewsToRestoreState->size(); ++i){
            ViewState& vs = (*viewsToRestoreState)[i];
            vs.view->restoreState(*vs.state);
        }
        return true;
    }
    return false;
}


SignalProxy<void(View* view)> ViewManager::sigViewCreated()
{
    return sigViewCreated_;
}

SignalProxy<void(View* view)> ViewManager::sigViewActivated()
{
    return sigViewActivated_;
}

SignalProxy<void(View* view)> ViewManager::sigViewDeactivated()
{
    return sigViewDeactivated_;
}

SignalProxy<void(View* view)> ViewManager::sigViewRemoved()
{
    return sigViewRemoved_;
}
