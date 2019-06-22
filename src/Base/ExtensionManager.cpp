/**
   @author Shin'ichiro NAKAOKA
*/

#include "ExtensionManager.h"
#include "ItemManager.h"
#include "ViewManager.h"
#include "MenuManager.h"
#include "OptionManager.h"
#include "ProjectManager.h"
#include "Plugin.h"
#include "Item.h"
#include "View.h"
#include "ToolBar.h"
#include "MainWindow.h"
#include "TimeSyncItemEngine.h"
#include "LazyCaller.h"
#include <cnoid/GettextUtil>
#include <cnoid/ExecutablePath>
#include <cnoid/FileUtil>
#include <QMenuBar>
#include <boost/algorithm/string.hpp>
#include <set>
#include <stack>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class ExtensionManagerImpl
{
public:
    ExtensionManagerImpl(ExtensionManager* self, const std::string& moduleName);
    ~ExtensionManagerImpl();

    void setVersion(const std::string& version, bool isPlugin);

    void deleteManagedObjects();

    ExtensionManager* self;
    string moduleName;
    string textDomain;
    stack<ExtensionManager::PtrHolderBase*> pointerHolders;
    Signal<void()> sigSystemUpdated;
    Signal<void()> sigReleaseRequest;
    std::unique_ptr<MenuManager> menuManager;
    std::unique_ptr<ItemManager> itemManager;
    std::unique_ptr<TimeSyncItemEngineManager> timeSyncItemEngineManger;
    std::unique_ptr<ViewManager> viewManager;
};

}

namespace {
    
set<ExtensionManagerImpl*> extensionManagerImpls;

void emitSigSystemUpdated()
{
    set<ExtensionManagerImpl*>::iterator p = extensionManagerImpls.begin();
    while(p != extensionManagerImpls.end()){
        (*p)->sigSystemUpdated();
        ++p;
    }
}

LazyCaller emitSigSystemUpdatedLater(emitSigSystemUpdated);
}


ExtensionManager::ExtensionManager(const std::string& moduleName, bool isPlugin)
{
    impl = new ExtensionManagerImpl(this, moduleName);
    impl->setVersion(CNOID_FULL_VERSION_STRING, isPlugin);
}


ExtensionManager::ExtensionManager(const std::string& moduleName, const std::string& version, bool isPlugin)
{
    impl = new ExtensionManagerImpl(this, moduleName);
    impl->setVersion(version, isPlugin);
}
    
    
ExtensionManagerImpl::ExtensionManagerImpl(ExtensionManager* self, const std::string& moduleName)
    : self(self),
      moduleName(moduleName)
{
    extensionManagerImpls.insert(this);
}


ExtensionManager::~ExtensionManager()
{
    delete impl;
}


ExtensionManagerImpl::~ExtensionManagerImpl()
{
    if(itemManager){
        itemManager->detachAllManagedTypeItemsFromRoot();
    }
    
    ProjectManager::instance()->resetArchivers(moduleName);
    deleteManagedObjects();
    sigReleaseRequest();
    extensionManagerImpls.erase(this);
}


const std::string& ExtensionManager::name() const
{
    return impl->moduleName;
}


const std::string& ExtensionManager::textDomain() const
{
    return impl->textDomain;
}


ItemManager& ExtensionManager::itemManager()
{
    if(!impl->itemManager){
        impl->itemManager.reset(new ItemManager(impl->moduleName, menuManager()));
        impl->itemManager->bindTextDomain(impl->textDomain);
    }
    return *impl->itemManager;
}


TimeSyncItemEngineManager& ExtensionManager::timeSyncItemEngineManger()
{
    if(!impl->timeSyncItemEngineManger){
        impl->timeSyncItemEngineManger.reset(new TimeSyncItemEngineManager(impl->moduleName));
    }
    return *impl->timeSyncItemEngineManger;
}


ViewManager& ExtensionManager::viewManager()
{
    if(!impl->viewManager){
        impl->viewManager.reset(new ViewManager(this));
    }
    return *impl->viewManager;
}


MenuManager& ExtensionManager::menuManager()
{
    if(!impl->menuManager){
        impl->menuManager.reset(new MenuManager(MainWindow::instance()->menuBar()));
        impl->menuManager->bindTextDomain(impl->textDomain);
    }
    return *impl->menuManager;
}


OptionManager& ExtensionManager::optionManager()
{
    static OptionManager optionManager;
    return optionManager;
}


void ExtensionManagerImpl::setVersion(const std::string& version, bool isPlugin)
{
    vector<string> v;
    boost::algorithm::split(v, version, boost::is_any_of("."));

    textDomain = string("Cnoid") + moduleName;
    if(isPlugin){
        textDomain += "Plugin";
    }
    if(!v.empty()){
        textDomain += "-";
        textDomain += v[0];
        if(v.size() >= 2){
            textDomain += string(".") + v[1];
        }
    }
    bindGettextDomain(textDomain.c_str());

#ifdef CNOID_ENABLE_GETTEXT
    bind_textdomain_codeset(textDomain.c_str(), "utf-8");
#endif
}


ExtensionManager::PtrHolderBase::~PtrHolderBase()
{

}


void ExtensionManager::manageSub(PtrHolderBase* holder)
{
    impl->pointerHolders.push(holder);
}


void ExtensionManager::addToolBar(ToolBar* toolBar)
{
    toolBar->setWindowTitle(dgettext(impl->textDomain.c_str(), toolBar->objectName().toUtf8()));

    manage(toolBar);
    MainWindow::instance()->addToolBar(toolBar);
}


void ExtensionManagerImpl::deleteManagedObjects()
{
    while(!pointerHolders.empty()){
        ExtensionManager::PtrHolderBase* holder = pointerHolders.top();
        delete holder;
        pointerHolders.pop();
    }
}


SignalProxy<void()> ExtensionManager::sigSystemUpdated()
{
    return impl->sigSystemUpdated;
}


void ExtensionManager::notifySystemUpdate()
{
    emitSigSystemUpdatedLater();
}


SignalProxy<void()> ExtensionManager::sigReleaseRequest()
{
    return impl->sigReleaseRequest;
}


void ExtensionManager::setProjectArchiver(
    const std::string& name,
    std::function<bool(Archive&)> storeFunction,
    std::function<void(const Archive&)> restoreFunction)
{
    ProjectManager::instance()->setArchiver(impl->moduleName, name, storeFunction, restoreFunction);
}


void ExtensionManager::setProjectArchiver(
    std::function<bool(Archive&)> storeFunction,
    std::function<void(const Archive&)> restoreFunction)
{
    ProjectManager::instance()->setArchiver(impl->moduleName, "", storeFunction, restoreFunction);
}
