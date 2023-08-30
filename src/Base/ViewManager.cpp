#include "ViewManager.h"
#include "PluginManager.h"
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
#include <unordered_map>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

unordered_set<string> pluginWhitelist;

typedef unordered_map<string, ViewManager::WhiteListElement> ViewWhiteListMap;
ViewWhiteListMap viewWhiteListMap;

Signal<void(View* view)> sigViewCreated_;
Signal<void(View* view)> sigViewActivated_;
Signal<void(View* view)> sigViewDeactivated_;
Signal<void(View* view)> sigViewRemoved_;

class ViewClassImpl;

class InstanceInfo : public Referenced {
public:
    View* view;
    ViewClassImpl* viewClass;

    InstanceInfo(ViewClassImpl* viewClass, View* view);
    ~InstanceInfo();
    void remove();
};

typedef ref_ptr<InstanceInfo> InstanceInfoPtr;


class ViewClassImpl : public ViewManager::ViewClass
{
public:
    ViewManager::Impl* manager;
    const std::type_info& view_type_info;
    string className;
    string translatedClassName;
    string defaultInstanceName_;
    string translatedDefaultInstanceName_; // temporary
    bool isSingleton_;
    bool hasPermanentInstance_;
    bool isEnabled;
    ViewManager::FactoryBase* factory;
    vector<InstanceInfoPtr> instanceInfos;

    ViewClassImpl(
        ViewManager::Impl* manager,
        const type_info& view_type_info, const string& className, const string& defaultInstanceName,
        int instantiationFlags, bool isEnabled, ViewManager::FactoryBase* factory);

    ~ViewClassImpl();

    virtual const std::string& moduleName() const override;

    virtual const std::string& name() const override {
        return className;
    }
    virtual const std::string& translatedName() const override {
        return translatedClassName;
    }
    virtual const std::string& defaultInstanceName() const override {
        return defaultInstanceName_;
    }
    virtual const std::string& translatedDefaultInstanceName() const override {
        return translatedDefaultInstanceName_;
    }
    virtual bool isSingleton() const override {
        return isSingleton_;
    }
    virtual bool hasPermanentInstance() const override {
        return hasPermanentInstance_;
    }
    virtual std::vector<View*> instances() const override;
    
    bool checkIfPermanentInstance(View* view){
        return hasPermanentInstance_ && !instanceInfos.empty() && (instanceInfos.front()->view == view);
    }

    bool checkIfPrimalInstance(View* view){
        return (hasPermanentInstance_ || isSingleton_) && !instanceInfos.empty() && (instanceInfos.front()->view == view);
    }

    View* createView();
    virtual View* getOrCreateView() override;
    View* createView(const string& name, bool setTranslatedNameToWindowTitle = false);
    View* findView(const string& name);
    virtual View* getOrCreateView(const std::string& name) override;
    virtual View* createViewWithDialog() override;
};

typedef ref_ptr<ViewClassImpl> ViewClassImplPtr;

typedef map<ViewClassImpl*, vector<View*>> ViewClassImplToViewsMap;

struct compare_type_info {
    bool operator ()(const type_info* a, const type_info* b) const {
        return a->before(*b);
    }
};
typedef std::map<const type_info*, ViewClassImplPtr, compare_type_info> TypeToViewClassImplMap;
TypeToViewClassImplMap typeToViewClassImplMap;

typedef map<string, ViewClassImplPtr> ClassNameToViewClassImplMap;
typedef std::shared_ptr<ClassNameToViewClassImplMap> ClassNameToViewClassImplMapPtr;

typedef map<string, ClassNameToViewClassImplMapPtr> ModuleNameToClassNameToViewClassImplMap;
ModuleNameToClassNameToViewClassImplMap moduleNameToClassNameToViewClassImplMap;

unordered_map<string, string> classNameAliasMap;


class ViewCreationDialog : public Dialog
{
public:
    QLineEdit nameEdit;
    ViewCreationDialog(ViewClassImpl* viewClass);
};

}

namespace cnoid {

class ViewManager::Impl
{
public:
    const string& moduleName;
    const string& textDomain;
    ClassNameToViewClassImplMapPtr classNameToViewClassImplMap;
    vector<InstanceInfoPtr> instanceInfos;
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

InstanceInfo::InstanceInfo(ViewClassImpl* viewClass, View* view)
    : view(view), viewClass(viewClass)
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
    auto& infos1 = viewClass->instanceInfos;
    auto it1 = std::find(infos1.begin(), infos1.end(), this);
    if(it1 != infos1.end()){
        infos1.erase(it1);
    }
    auto& infos2 = viewClass->manager->instanceInfos;
    auto it2 = std::find(infos2.begin(), infos2.end(), this);
    if(it2 != infos2.end()){
        infos2.erase(it2);
    }
}


ViewClassImpl::ViewClassImpl
(ViewManager::Impl* manager,
 const type_info& view_type_info, const string& className, const string& defaultInstanceName,
 int instantiationFlags, bool isEnabled, ViewManager::FactoryBase* factory)
    : manager(manager),
      view_type_info(view_type_info),
      className(className),
      defaultInstanceName_(defaultInstanceName),
      isEnabled(isEnabled),
      factory(factory)
{
    isSingleton_ = (instantiationFlags & ViewManager::Multiple) ? false : true;
    hasPermanentInstance_ = isEnabled && (instantiationFlags & ViewManager::Permanent);
    translatedClassName = dgettext(manager->textDomain.c_str(), className.c_str());
    translatedDefaultInstanceName_ = dgettext(manager->textDomain.c_str(), defaultInstanceName_.c_str());
}


ViewClassImpl::~ViewClassImpl()
{
    delete factory;
}


const std::string& ViewClassImpl::moduleName() const
{
    return manager->moduleName;
}


std::vector<View*> ViewClassImpl::instances() const
{
    vector<View*> instances_;
    instances_.reserve(instanceInfos.size());
    for(auto& info : instanceInfos){
        instances_.push_back(info->view);
    }
    return instances_;
}
    

View* ViewClassImpl::createView()
{
    View* view = factory->create();
    ViewManager::Impl::setViewClassName(view, className);
    
    auto instanceInfo = new InstanceInfo(this, view);
    instanceInfos.push_back(instanceInfo);
    manager->instanceInfos.push_back(instanceInfo);
    
    view->sigActivated().connect([view](){ sigViewActivated_(view); });
    view->sigDeactivated().connect([view](){ sigViewDeactivated_(view); });
    
    return view;
}


View* ViewClassImpl::getOrCreateView()
{
    if(instanceInfos.empty()){
        View* view = createView();
        view->setName(defaultInstanceName_);
        view->setWindowTitle(translatedDefaultInstanceName_.c_str());
        sigViewCreated_(view);
    }
    return instanceInfos.front()->view;
}


View* ViewClassImpl::createView(const string& name, bool setTranslatedNameToWindowTitle)
{
    if(name.empty()){
        return nullptr;
    }
    View* view = createView();
    if(isSingleton_){
        view->setName(defaultInstanceName_);
        view->setWindowTitle(translatedDefaultInstanceName_.c_str());
    } else {
        view->setName(name);
        if(setTranslatedNameToWindowTitle){
            view->setWindowTitle(dgettext(manager->textDomain.c_str(), name.c_str()));
        }
    }
    sigViewCreated_(view);
    return view;
}


View* ViewClassImpl::findView(const string& name)
{
    if(name.empty()){
        return instanceInfos.empty() ? nullptr : instanceInfos.front()->view;
    } else {
        for(auto& instanceInfo : instanceInfos){
            if(instanceInfo->view->name() == name){
                return instanceInfo->view;
            }
        }
        return nullptr;
    }
}


View* ViewClassImpl::getOrCreateView(const std::string& name)
{
    View* view = findView(name);
    if(!view){
        view = createView(name);
        sigViewCreated_(view);
    }
    return view;
}


View* ViewClassImpl::createViewWithDialog()
{
    View* view = nullptr;
    ViewCreationDialog dialog(this);
    while(dialog.exec() == QDialog::Accepted){
        string name = dialog.nameEdit.text().toStdString();
        if(name.empty()){
            showWarningDialog(_("Please specify the name of the new view."));
        } else {
            view = createView(name);
            break;
        }
    }
    return view;
}


ViewCreationDialog::ViewCreationDialog(ViewClassImpl* viewClass)
{
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
    
    setWindowTitle(QString(_("Create %1")).arg(viewClass->translatedClassName.c_str()));
    
    if(viewClass->instanceInfos.empty()){
        nameEdit.setText(viewClass->translatedDefaultInstanceName_.c_str());
    } else {
        nameEdit.setText(
            QString("%1 %2")
            .arg(viewClass->translatedDefaultInstanceName_.c_str())
            .arg(viewClass->instanceInfos.size() + 1));
    }
}
    
}


void ViewManager::initializeClass(ExtensionManager* ext)
{
    static bool initialized = false;

    if(!initialized){
        View::initializeClass();
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


void ViewManager::setClassAlias(const std::string& alias, const std::string& orgClassName)
{
    classNameAliasMap[alias] = orgClassName;
}


ViewManager::ViewManager(ExtensionManager* ext)
{
    impl = new Impl(ext);
}


ViewManager::Impl::Impl(ExtensionManager* ext)
    : moduleName(ext->name()),
      textDomain(ext->textDomain())
{
    classNameToViewClassImplMap = std::make_shared<ClassNameToViewClassImplMap>();
    moduleNameToClassNameToViewClassImplMap[moduleName] = classNameToViewClassImplMap;

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
    while(!impl->instanceInfos.empty()){
        InstanceInfo* instanceInfo = *(impl->instanceInfos.rbegin());
        instanceInfo->remove();
    };

    delete impl;
}


ViewManager::Impl::~Impl()
{
    for(auto& kv : *classNameToViewClassImplMap){
        ViewClassImpl* info = kv.second;
        typeToViewClassImplMap.erase(&info->view_type_info);
    }
    
    moduleNameToClassNameToViewClassImplMap.erase(moduleName);
}


void ViewManager::registerClass_
(const type_info& view_type_info,
 const std::string& className, const std::string& defaultInstanceName, int instantiationFlags,
 FactoryBase* factory)
{
    bool isEnabled = true;
    WhiteListElement* whiteListElement = nullptr;
    
    if(impl->doCheckViewWhitelist){
        auto it = viewWhiteListMap.find(className);
        if(it != viewWhiteListMap.end()){
            whiteListElement = &it->second;
        } else {
            isEnabled = false;
        }
    }

    if(whiteListElement && whiteListElement->hasCustomInstantiationFlags){
        instantiationFlags = whiteListElement->instantiationFlags;
    }
    
    auto info = new ViewClassImpl(
        impl, view_type_info, className, defaultInstanceName, instantiationFlags, isEnabled, factory);
    
    (*impl->classNameToViewClassImplMap)[className] = info;
    typeToViewClassImplMap[&view_type_info] = info;

    if((instantiationFlags & Permanent) && isEnabled){
        if(auto view = info->getOrCreateView()){
            // Note that a permanent view should be mounted on the main window by default
            // even if it is not included in the builtin project, but the following function
            // does not work for that purpose. Refer to ViewManager::restoreViewStates function
            // to consider an implementation that achives it. Note that the permenent view
            // should also be mounted when the view layout in the main window is reset by the
            // ViewArea::resetLayout.

            // view->mountOnMainWindow(false);
        }
    }
}


ViewManager& ViewManager::registerClassAlias(const std::string& alias, const std::string& orgClassName)
{
    setClassAlias(alias, orgClassName);
    return *this;
}


std::vector<ViewManager::ViewClass*> ViewManager::viewClasses()
{
    std::vector<ViewManager::ViewClass*> classes;
    classes.reserve(typeToViewClassImplMap.size());
        
    for(auto& kv1 : moduleNameToClassNameToViewClassImplMap){
        for(auto& kv2 : *kv1.second){
            ViewClassImpl* viewClass = kv2.second;
            if(viewClass->isEnabled){
                classes.push_back(viewClass);
            }
        }
    }
    return classes;
}


namespace {

ViewClassImpl* findViewClassImpl(const std::string& moduleName, const std::string& className, bool checkAlias = true)
{
    ViewClassImpl* info = nullptr;

    const string* pActualClassName = &className;

    if(checkAlias){
        auto it = classNameAliasMap.find(className);
        if(it != classNameAliasMap.end()){
            pActualClassName = &it->second;
            static regex re("^(.+)::(.+)$");
            std::smatch match;
            if(regex_match(*pActualClassName, match, re)){
                return findViewClassImpl(match.str(1), match.str(2), false);
            }
        }
    }

    auto it1 = moduleNameToClassNameToViewClassImplMap.find(moduleName);
    if(it1 != moduleNameToClassNameToViewClassImplMap.end()){
        auto& infoMap = *it1->second;
        auto it2 = infoMap.find(*pActualClassName);
        if(it2 != infoMap.end()){
            info = it2->second;
        }
    }
    
    return info;
}

}
        

View* ViewManager::getOrCreateView(const std::string& moduleName, const std::string& className)
{
    ViewClassImpl* info = findViewClassImpl(moduleName, className);
    if(info){
        return info->getOrCreateView();
    }
    return nullptr;
}


std::vector<View*> ViewManager::allViews()
{
    std::vector<View*> views;
    for(auto& kv : typeToViewClassImplMap){
        for(auto& instanceInfo : kv.second->instanceInfos){
            views.push_back(instanceInfo->view);
        }
    }
    return views;
}


std::vector<View*> ViewManager::activeViews()
{
    std::vector<View*> views;
    for(auto& kv : typeToViewClassImplMap){
        for(auto& instanceInfo : kv.second->instanceInfos){
            View* view = instanceInfo->view;
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
    View* view = nullptr;
    auto p = typeToViewClassImplMap.find(&view_type_info);
    if(p != typeToViewClassImplMap.end()){
        ViewClassImpl& info = *p->second;
        if(instanceName.empty()){
            view = info.getOrCreateView();
        } else {
            view = info.getOrCreateView(instanceName);
        }
    }
    if(view && doMountCreatedView){
        view->mountOnMainWindow(false);
    }
    return view;
}


View* ViewManager::findSpecificTypeView(const std::type_info& view_type_info, const std::string& instanceName)
{
    auto p = typeToViewClassImplMap.find(&view_type_info);
    if(p != typeToViewClassImplMap.end()){
        ViewClassImpl& info = *p->second;
        return info.findView(instanceName);
    }
    return nullptr;
}    


void ViewManager::deleteView(View* view)
{
    for(auto& kv : typeToViewClassImplMap){
        ViewClassImpl* viewClass = kv.second;
        for(auto& instanceInfo : viewClass->instanceInfos){
            if(instanceInfo->view == view){
                instanceInfo->remove();
                break;
            }
        }
    }
}


void ViewManager::deleteUnmountedViews()
{
    for(auto& kv : typeToViewClassImplMap){
        ViewClassImpl* viewClass = kv.second;
        auto it = viewClass->instanceInfos.begin();
        while(it != viewClass->instanceInfos.end()){
            InstanceInfo* instanceInfo = *it++;
            if(!instanceInfo->view->isMounted()){
                instanceInfo->remove();
            }
        }
    }
}


bool ViewManager::isPrimalInstance(View* view)
{
    auto p = typeToViewClassImplMap.find(&typeid(*view));
    if(p != typeToViewClassImplMap.end()){
        ViewClassImpl& info = *p->second;
        return info.checkIfPrimalInstance(view);
    }
    return false;
}


static ArchivePtr storeView
(Archive& parentArchive, const string& moduleName, ViewClassImpl* viewClass, View* view, bool isLayoutMode)
{
    ArchivePtr archive;
    ArchivePtr state;
    bool isValid = true;
    bool isPrimalInstance = viewClass->checkIfPrimalInstance(view);

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
        archive->write("class", viewClass->className);

        if(view->isMounted()){
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
    for(auto& kv1 : moduleNameToClassNameToViewClassImplMap){
        ClassNameToViewClassImplMap& viewClassMap = *kv1.second;
        for(auto& kv2 : viewClassMap){
            ViewClassImpl* viewClass = kv2.second;
            for(auto& instanceInfo : viewClass->instanceInfos){
                archive->registerViewId(instanceInfo->view, id++);
            }
        }
    }
    
    ListingPtr viewList = new Listing;

    for(auto& kv1 : moduleNameToClassNameToViewClassImplMap){
        const std::string& moduleName = kv1.first;
        ClassNameToViewClassImplMap& viewClassMap = *kv1.second;
        for(auto& kv2 : viewClassMap){
            ViewClassImpl* viewClass = kv2.second;
            for(auto& instanceInfo : viewClass->instanceInfos){
                View* view = instanceInfo->view;
                ArchivePtr viewArchive = storeView(*archive, moduleName, viewClass, view, isLayoutMode);
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
    bool doMount;
    ViewState(View* view, ArchivePtr state, bool doMount)
        : view(view), state(state), doMount(doMount) { }
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
(Archive* archive, const string& moduleName, const string& className, ViewClassImplToViewsMap& remainingViewsMap)
{
    ViewClassImpl* info = findViewClassImpl(moduleName, className);
    View* view = nullptr;
    string instanceName;
    string errorMessage;

    if(!info){
        errorMessage = format(_("{0} is not registered in {1}."), className, moduleName);

    } else if(!info->isEnabled){
        errorMessage = format(_("{0} is disabled by the application."), className);

    } else {
        archive->read("name", instanceName);

        if(instanceName.empty() || info->isSingleton_){
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
                auto& instanceInfos = info->instanceInfos;
                remainingViews->reserve(instanceInfos.size());
                auto q = instanceInfos.begin();
                if(info->hasPermanentInstance_ && q != instanceInfos.end()){
                    ++q;
                }
                while(q != instanceInfos.end()){
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
                if(!info->isSingleton_ || info->instanceInfos.empty()){
                    view = info->createView(instanceName, true);
                }
            }
        }
    }
    if(!view){
        string message;
        if(instanceName.empty()){
            if(errorMessage.empty()){
                message = format(_("{0} cannot be restored."), className);
            } else {
                message = format(_("{0} cannot be restored: {1}"), className, errorMessage);
            }
        } else {
            if(errorMessage.empty()){
                message = format(_("The \"{0}\" view of {1} cannot be restored."), instanceName, className);
            } else {
                message = format(_("The \"{0}\" view of {1} cannot be restored: {2}"), instanceName, className, errorMessage);
            }
        }
        MessageView::instance()->putln(message, MessageView::Error);
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

        ViewClassImplToViewsMap remainingViewsMap;
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
                            } else {
                                state.reset();
                            }
                            bool doMount = viewArchive->get("mounted", false);
                            viewsToRestoreState->push_back(ViewState(view, state, doMount));
                            
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
    if(!info.data){
        return false;
    }
    auto viewsToRestoreState = reinterpret_cast<vector<ViewState>*>(info.data);
    for(auto& viewState : *viewsToRestoreState){
        auto view = viewState.view;
        if(viewState.state){
            view->restoreState(*viewState.state);
        }
        view->onRestored(static_cast<bool>(viewState.state));
        if(viewState.doMount && !view->isMounted()){
            view->mountOnMainWindow(false);
        }
    }
    return true;
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
