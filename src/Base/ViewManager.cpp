/**
   @author Shin'ichiro Nakaoka
*/

#include "ViewManager.h"
#include "PluginManager.h"
#include "MainWindow.h"
#include "MainMenu.h"
#include "ViewArea.h"
#include "MessageView.h"
#include "Dialog.h"
#include "Archive.h"
#include "Action.h"
#include <QBoxLayout>
#include <QLineEdit>
#include <QLabel>
#include <QDialogButtonBox>
#include <QPushButton>
#include <fmt/format.h>
#include <list>
#include <regex>
#include <unordered_set>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

MainWindow* mainWindow = nullptr;

enum ViewMenuId { ShowView, CreateView, DeleteView };

unordered_set<string> pluginWhitelist;

typedef unordered_map<string, ViewManager::WhiteListElement> ViewWhiteListMap;
ViewWhiteListMap viewWhiteListMap;

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

    InstanceInfo(ViewInfo* viewInfo, View* view);
    ~InstanceInfo();
    void remove();
};

class ViewInfo
{
public:
    const std::type_info& view_type_info;
    string className;
    string textDomain;
    string translatedClassName;
    string defaultInstanceName;
    string translatedDefaultInstanceName; // temporary.
    bool isSingleton;
    bool hasDefaultInstance;
    bool isEnabled;
    ViewManager::FactoryBase* factory;

    InstanceInfoList instances;
    InstanceInfoList& instancesInViewManager;

    ViewInfo(
        ViewManager::Impl* managerImpl,
        const type_info& view_type_info, const string& className, const string& defaultInstanceName,
        const string& textDomain, int instantiationFlags, bool isEnabled, ViewManager::FactoryBase* factory);

    ~ViewInfo();

    bool checkIfDefaultInstance(View* view){
        return hasDefaultInstance && !instances.empty() && (instances.front()->view == view);
    }

    bool checkIfPrimalInstance(View* view){
        return (hasDefaultInstance || isSingleton) && !instances.empty() && (instances.front()->view == view);
    }

    View* createView();
    View* getOrCreateView(bool doMountCreatedView = false);
    View* createView(const string& name, bool setTranslatedNameToWindowTitle = false);
    View* findView(const string& name);
    View* getOrCreateView(const string& name, bool doMountCreatedView = false);
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

class ViewManager::Impl
{
public:
    const string& moduleName;
    const string& textDomain;
    ClassNameToViewInfoMapPtr classNameToViewInfoMap;
    InstanceInfoList instances;
    bool doCheckViewWhitelist;

    Impl(ExtensionManager* ext);
    ~Impl();

    static void setViewClassName(View* view, const string& name){
        view->setClassName(name);
    }
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


namespace {

InstanceInfo::InstanceInfo(ViewInfo* viewInfo, View* view)
    : view(view), viewInfo(viewInfo)
{

}


InstanceInfo::~InstanceInfo()
{
    if(view){
        ViewManager::Impl::deactivateView(view);
        delete view;
    }
}

void InstanceInfo::remove()
{
    if(view){
        ViewManager::Impl::deactivateView(view);
        ViewManager::Impl::notifySigRemoved(view);
        sigViewRemoved_(view);
        delete view;
        view = nullptr;
    }
    viewInfo->instances.erase(iterInViewInfo);
    iterInViewInfo = viewInfo->instances.end();
    viewInfo->instancesInViewManager.erase(iterInViewManager);
    //iterInViewManager = viewInfo->instancesInViewManager.end();
}


ViewInfo::ViewInfo
(ViewManager::Impl* managerImpl,
 const type_info& view_type_info, const string& className, const string& defaultInstanceName,
 const string& textDomain, int instantiationFlags, bool isEnabled, ViewManager::FactoryBase* factory)
    : view_type_info(view_type_info),
      className(className),
      textDomain(textDomain),
      defaultInstanceName(defaultInstanceName),
      isEnabled(isEnabled),
      factory(factory),
      instancesInViewManager(managerImpl->instances)
{
    isSingleton = (instantiationFlags & ViewManager::Multiple) ? false : true;
    hasDefaultInstance = instantiationFlags & ViewManager::Default;
    translatedClassName = dgettext(textDomain.c_str(), className.c_str());
    translatedDefaultInstanceName = dgettext(textDomain.c_str(), defaultInstanceName.c_str());
}


ViewInfo::~ViewInfo()
{
    delete factory;
}


View* ViewInfo::createView()
{
    View* view = factory->create();
    ViewManager::Impl::setViewClassName(view, className);
    
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


View* ViewInfo::getOrCreateView(bool doMountCreatedView)
{
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


View* ViewInfo::createView(const string& name, bool setTranslatedNameToWindowTitle)
{
    if(name.empty()){
        return nullptr;
    }
    View* view = createView();
    if(isSingleton){
        view->setName(defaultInstanceName);
        view->setWindowTitle(translatedDefaultInstanceName.c_str());
    } else {
        view->setName(name);
        if(setTranslatedNameToWindowTitle){
            view->setWindowTitle(dgettext(textDomain.c_str(), name.c_str()));
        }
    }
    sigViewCreated_(view);
    return view;
}


View* ViewInfo::findView(const string& name)
{
    if(name.empty()){
        return instances.empty() ? nullptr : instances.front()->view;
    } else {
        for(auto p = instances.begin(); p != instances.end(); ++p){
            if((*p)->view->name() == name){
                return (*p)->view;
            }
        }
        return nullptr;
    }
}


View* ViewInfo::getOrCreateView(const string& name, bool doMountCreatedView)
{
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


class ViewCreationDialog : public Dialog
{
public:
    QLineEdit nameEdit;
        
    ViewCreationDialog(ViewInfoPtr viewInfo) {

        auto vbox = new QVBoxLayout;
            
        auto hbox = new QHBoxLayout;
        hbox->addWidget(new QLabel(_("Name:")));
        hbox->addWidget(&nameEdit);
        vbox->addLayout(hbox);
        
        auto buttonBox = new QDialogButtonBox(this);
        
        auto createButton = new QPushButton(_("&Create"));
        createButton->setDefault(true);
        buttonBox->addButton(createButton, QDialogButtonBox::AcceptRole);
        connect(buttonBox,SIGNAL(accepted()), this, SLOT(accept()));
        
        auto cancelButton = new QPushButton(_("&Cancel"));
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
    for(auto p = typeToViewInfoMap.begin(); p != typeToViewInfoMap.end(); ++p){
        ViewInfo& info = *p->second;
        auto q = info.instances.begin();
        while(q != info.instances.end()){
            InstanceInfoPtr& instance = (*q++);
            if(!instance->view->viewArea()){
                instance->remove();
            }
        }
    }
}
    
void onViewMenuAboutToShow(Menu* menu, ViewMenuId viewMenuId)
{
    menu->clear();
    bool needSeparator = false;
        
    for(auto p = moduleNameToClassNameToViewInfoMap.begin(); p != moduleNameToClassNameToViewInfoMap.end(); ++p){

        if(needSeparator){
            menu->addSeparator();
            needSeparator = false;
        }
            
        ClassNameToViewInfoMap& viewInfoMap = *p->second;

        for(auto q = viewInfoMap.begin(); q != viewInfoMap.end(); ++q){

            ViewInfoPtr& viewInfo = q->second;
            if(!viewInfo->isEnabled){
                continue;
            }
            InstanceInfoList& instances = viewInfo->instances;

            if(viewMenuId == ShowView){
                View* view = nullptr;
                if(instances.empty()){
                    auto action = new Action(menu);
                    action->setText(viewInfo->translatedDefaultInstanceName.c_str());
                    action->setCheckable(true);
                    action->sigToggled().connect([=](bool on){ onShowViewToggled(viewInfo, view, on); });
                    menu->addAction(action);
                } else {
                    for(auto p = instances.begin(); p != instances.end(); ++p){
                        InstanceInfoPtr& instance = (*p);
                        view = instance->view;
                        auto action = new Action(menu);
                        action->setText(view->windowTitle());
                        action->setCheckable(true);
                        action->setChecked(view->viewArea());
                        action->sigToggled().connect([=](bool on){ onShowViewToggled(viewInfo, view, on); });
                        menu->addAction(action);
                    }
                }
            } else if(viewMenuId == CreateView){
                if(!viewInfo->isSingleton || 
                   (!viewInfo->hasDefaultInstance && viewInfo->instances.empty())){
                    auto action = new Action(menu);
                    action->setText(viewInfo->translatedDefaultInstanceName.c_str());
                    action->sigTriggered().connect([=](){ onCreateViewTriggered(viewInfo); });
                    menu->addAction(action);
                }
            } else if(viewMenuId == DeleteView){
                auto p = instances.begin();
                if(viewInfo->hasDefaultInstance && p != instances.end()){
                    ++p;
                }
                while(p != instances.end()){
                    InstanceInfoPtr& instance = (*p++);
                    auto action = new Action(menu);
                    action->setText(instance->view->windowTitle());
                    action->sigTriggered().connect([=](){ onDeleteViewTriggered(instance); });
                    menu->addAction(action);
                }
            }
            needSeparator = true;
        }
    }

    if(viewMenuId == DeleteView){
        if(needSeparator){
            menu->addSeparator();
        }
        auto action = new Action(menu);
        action->setText(_("Delete All Invisible Views"));
        action->sigTriggered().connect([](){ deleteAllInvisibleViews(); });
        menu->addAction(action);
    }
}

}


void ViewManager::initializeClass(ExtensionManager* ext)
{
    static bool initialized = false;

    if(!initialized){
        View::initializeClass();
        
        mainWindow = MainWindow::instance();

        auto mainMenu = MainMenu::instance();
        mainMenu->sig_View_Show_MenuAboutToShow().connect(
            [=](Menu* menu){ onViewMenuAboutToShow(menu, ShowView); });
        mainMenu->sig_View_Create_MenuAboutToShow().connect(
            [=](Menu* menu){ onViewMenuAboutToShow(menu, CreateView); });
        mainMenu->sig_View_Delete_MenuAboutToShow().connect(
            [=](Menu* menu){ onViewMenuAboutToShow(menu, DeleteView); });
        
        initialized = true;
    }
}


void ViewManager::setPluginWhitelist(const std::vector<const char*>& pluginNames)
{
    for(auto& name : pluginNames){
        pluginWhitelist.insert(name);
    }
}


void ViewManager::setViewWhitelist(const std::vector<WhiteListElement>& elements)
{
    for(auto& element : elements){
        viewWhiteListMap.insert(ViewWhiteListMap::value_type(element.viewClassName, element));
    };
}


ViewManager::ViewManager(ExtensionManager* ext)
{
    impl = new Impl(ext);
}


ViewManager::Impl::Impl(ExtensionManager* ext)
    : moduleName(ext->name()),
      textDomain(ext->textDomain())
{
    classNameToViewInfoMap = std::make_shared<ClassNameToViewInfoMap>();
    moduleNameToClassNameToViewInfoMap[moduleName] = classNameToViewInfoMap;

    doCheckViewWhitelist = false;
    if(!pluginWhitelist.empty() || !viewWhiteListMap.empty()){
        if(pluginWhitelist.find(moduleName) == pluginWhitelist.end()){
            doCheckViewWhitelist = true;
        }
    }
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


ViewManager::Impl::~Impl()
{
    for(auto p = classNameToViewInfoMap->begin(); p != classNameToViewInfoMap->end(); ++p){
        ViewInfoPtr& info = p->second;
        typeToViewInfoMap.erase(&info->view_type_info);
    }
    
    moduleNameToClassNameToViewInfoMap.erase(moduleName);
}


void ViewManager::registerClass_
(const type_info& view_type_info,
 const std::string& className, const std::string& defaultInstanceName, int instantiationFlags,
 FactoryBase* factory)
{
    bool isEnabled = true;
    WhiteListElement* whiteListElement = nullptr;
    
    if(impl->doCheckViewWhitelist){
        auto iter = viewWhiteListMap.find(className);
        if(iter != viewWhiteListMap.end()){
            whiteListElement = &iter->second;
        } else {
            isEnabled = false;
        }
    }

    if(!isEnabled){
        if(!(instantiationFlags & Default)){
            return;
        }
    }

    if(whiteListElement && whiteListElement->hasCustomInstantiationFlags){
        instantiationFlags = whiteListElement->instantiationFlags;
    }
    
    ViewInfoPtr info = std::make_shared<ViewInfo>(
        impl, view_type_info, className, defaultInstanceName, impl->textDomain, instantiationFlags, isEnabled, factory);
    
    (*impl->classNameToViewInfoMap)[className] = info;
    typeToViewInfoMap[&view_type_info] = info;

    if((instantiationFlags & Default) && isEnabled){
        auto view = info->getOrCreateView();
        mainWindow->viewArea()->addView(view);
    }
}


ViewManager& ViewManager::registerClassAlias(const std::string& alias, const std::string& orgClassName)
{
    classNameAliasMap[alias] = orgClassName;
    return *this;
}


namespace {

ViewInfo* findViewInfo(const std::string& moduleName, const std::string& className, bool checkAlias = true)
{
    ViewInfo* info = nullptr;
    auto p = moduleNameToClassNameToViewInfoMap.find(moduleName);
    if(p != moduleNameToClassNameToViewInfoMap.end()){
        auto& infoMap = *p->second;
        auto q = infoMap.find(className);
        if(q == infoMap.end() && checkAlias){
            auto r = classNameAliasMap.find(className);
            if(r != classNameAliasMap.end()){
                auto& actualClass = r->second;
                static regex re("^(.+)::(.+)$");
                std::smatch match;
                if(!regex_match(actualClass, match, re)){
                    q = infoMap.find(actualClass);
                } else {
                    return findViewInfo(match.str(1), match.str(2), false);
                }
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


std::vector<View*> ViewManager::allViews()
{
    std::vector<View*> views;
    for(auto p = typeToViewInfoMap.begin(); p != typeToViewInfoMap.end(); ++p){
        InstanceInfoList& instances = p->second->instances;
        for(auto q = instances.begin(); q != instances.end(); ++q){
            views.push_back((*q)->view);
        }
    }
    return views;
}


std::vector<View*> ViewManager::activeViews()
{
    std::vector<View*> views;
    for(auto p = typeToViewInfoMap.begin(); p != typeToViewInfoMap.end(); ++p){
        InstanceInfoList& instances = p->second->instances;
        for(auto q = instances.begin(); q != instances.end(); ++q){
            View* view = (*q)->view;
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
    auto p = typeToViewInfoMap.find(&view_type_info);
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
    auto p = typeToViewInfoMap.find(&view_type_info);
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
    auto p = typeToViewInfoMap.find(&typeid(*view));
    if(p != typeToViewInfoMap.end()){
        ViewInfo& info = *p->second;
        return info.checkIfPrimalInstance(view);
    }
    return false;
}


static ArchivePtr storeView
(Archive& parentArchive, const string& moduleName, ViewInfo& viewInfo, View* view, bool isLayoutMode)
{
    ArchivePtr archive;
    ArchivePtr state;
    bool isValid = true;
    bool isPrimalInstance = viewInfo.checkIfPrimalInstance(view);

    if(!isLayoutMode || !isPrimalInstance){
        state = new Archive;
        state->inheritSharedInfoFrom(parentArchive);
        if(!view->storeState(*state)){
            isValid = false;
        }
    }
        
    if(isValid){
        archive = new Archive;
        archive->inheritSharedInfoFrom(parentArchive);

        archive->write("id", archive->getViewId(view));
            
        if(!isPrimalInstance){
            archive->write("name", view->name(), DOUBLE_QUOTED);
        }
        archive->write("plugin", moduleName);
        archive->write("class", viewInfo.className);

        if(view->viewArea()){
            archive->write("mounted", true);
        }

        if(state && !state->empty()){
            archive->insert("state", state);
        }
    }

    return archive;
}


bool ViewManager::storeViewStates(Archive* archive, const std::string& key, bool isLayoutMode)
{
    // assign view ids first
    int id = 0;
    for(auto p = moduleNameToClassNameToViewInfoMap.begin(); p != moduleNameToClassNameToViewInfoMap.end(); ++p){
        ClassNameToViewInfoMap& viewInfoMap = *p->second;
        for(auto q = viewInfoMap.begin(); q != viewInfoMap.end(); ++q){
            ViewInfoPtr& viewInfo = q->second;
            InstanceInfoList& instances = viewInfo->instances;
            for(auto p = instances.begin(); p != instances.end(); ++p){            
                archive->registerViewId((*p)->view, id++);
            }
        }
    }
    
    ListingPtr viewList = new Listing;

    for(auto p = moduleNameToClassNameToViewInfoMap.begin(); p != moduleNameToClassNameToViewInfoMap.end(); ++p){
        const std::string& moduleName = p->first;
        ClassNameToViewInfoMap& viewInfoMap = *p->second;
        for(auto q = viewInfoMap.begin(); q != viewInfoMap.end(); ++q){
            ViewInfoPtr& viewInfo = q->second;
            InstanceInfoList& instances = viewInfo->instances;
            for(auto p = instances.begin(); p != instances.end(); ++p){            
                View* view = (*p)->view;
                ArchivePtr viewArchive = storeView(*archive, moduleName, *viewInfo, view, isLayoutMode);
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


static View* restoreView
(Archive* archive, const string& moduleName, const string& className, ViewInfoToViewsMap& remainingViewsMap)
{
    ViewInfo* info = findViewInfo(moduleName, className);
    View* view = nullptr;
    string instanceName;

    if(!info){
        MessageView::instance()->putln(
            format(_("{0} is not registered in {1}."), className, moduleName),
            MessageView::Error);

    } else {
        archive->read("name", instanceName);

        if(instanceName.empty() || info->isSingleton){
            view = info->getOrCreateView();
            
        } else {
            // get one of the view instances having the instance name, or create a new instance.
            // Different instances are assigned even if there are instances with the same name in the archive
            vector<View*>* remainingViews;

            auto p = remainingViewsMap.find(info);
            if(p != remainingViewsMap.end()){
                remainingViews = &p->second;
            } else {
                remainingViews = &remainingViewsMap[info];
                InstanceInfoList& instances = info->instances;
                remainingViews->reserve(instances.size());
                auto q = instances.begin();
                if(info->hasDefaultInstance && q != instances.end()){
                    ++q;
                }
                while(q != instances.end()){
                    remainingViews->push_back((*q++)->view);
                }
            }
            for(auto q = remainingViews->begin(); q != remainingViews->end(); ++q){
                if((*q)->name() == instanceName){
                    view = *q;
                    remainingViews->erase(q);
                    break;
                }
            }
            if(!view){
                if(!info->isSingleton || info->instances.empty()){
                    view = info->createView(instanceName, true);
                }
            }
        }
    }
    if(!view){
        if(instanceName.empty()){
            MessageView::instance()->putln(
                format(_("{0} cannot be restored."), className), MessageView::Error);
        } else {
            MessageView::instance()->putln(
                format(_("The \"{0}\" view of {1} cannot be restored."), instanceName, className),
                MessageView::Error);
        }
    }

    return view;
}


bool ViewManager::restoreViews
(Archive* archive, const std::string& key, ViewManager::ViewStateInfo& out_viewStateInfo,
 const std::set<std::string>* optionalPlugins, bool enableMissingPluginWarnings)
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
                        if(enableMissingPluginWarnings &&
                           optionalPlugins->find(moduleName) == optionalPlugins->end()){
                            MessageView::instance()->putln(
                                format(_("The \"{0}\" plugin for \"{1}\" is not found. The view cannot be restored."),
                                       moduleName, className),
                                MessageView::Error);
                        }
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


bool ViewManager::restoreViews
(Archive* archive, const std::string& key, ViewStateInfo& out_viewStateInfo,  bool enableMissingPluginWarnings)
{
    return restoreViews(archive, key, out_viewStateInfo, nullptr, false);
}


bool ViewManager::restoreViews
(Archive* archive, const std::string& key, ViewManager::ViewStateInfo& out_viewStateInfo,
 const std::set<std::string>& optionalPlugins)
{
    return restoreViews(archive, key, out_viewStateInfo, &optionalPlugins, true);
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
