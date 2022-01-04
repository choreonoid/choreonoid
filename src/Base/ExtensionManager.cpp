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
#include <cnoid/GettextUtil>
#include <cnoid/ExecutablePath>
#include <QMenuBar>
#include <set>
#include <deque>
#include <unordered_set>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

unordered_set<string> pluginWhitelistForToolBars;
unordered_set<string> toolBarWhitelist;

}

namespace cnoid {

class ExtensionManager::Impl
{
public:
    ExtensionManager* self;
    string moduleName;
    string textDomain;
    deque<PtrHolderBase*> pointerHolders;
    std::unique_ptr<MenuManager> menuManager;
    std::unique_ptr<ItemManager> itemManager;
    std::unique_ptr<ViewManager> viewManager;
    bool isBeforeAddingToolBars;
    bool doCheckToolBarWhitelist;

    static set<Impl*> instances;

    Impl(ExtensionManager* self, const std::string& moduleName, bool isPlugin);
    ~Impl();
    deque<PtrHolderBase*>::iterator deleteManagedObject(deque<PtrHolderBase*>::iterator iter);
    void deleteManagedObjects();
};

set<ExtensionManager::Impl*> ExtensionManager::Impl::instances;

}


ExtensionManager::ExtensionManager(const std::string& moduleName, bool isPlugin)
{
    impl = new Impl(this, moduleName, isPlugin);
}


ExtensionManager::Impl::Impl(ExtensionManager* self, const std::string& moduleName, bool isPlugin)
    : self(self),
      moduleName(moduleName)
{
    instances.insert(this);

    textDomain = bindModuleTextDomain(isPlugin ? (moduleName + "Plugin") : moduleName);

    isBeforeAddingToolBars = true;
    doCheckToolBarWhitelist = false;
}


ExtensionManager::~ExtensionManager()
{
    delete impl;
}


ExtensionManager::Impl::~Impl()
{
    if(itemManager){
        itemManager->detachAllManagedTypeItemsFromRoot();
    }
    
    ProjectManager::instance()->resetArchivers(moduleName);
    deleteManagedObjects();
    instances.erase(this);
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
        impl->itemManager.reset(new ItemManager(impl->moduleName));
        impl->itemManager->bindTextDomain(impl->textDomain);
    }
    return *impl->itemManager;
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


ExtensionManager::PtrHolderBase::~PtrHolderBase()
{

}


void ExtensionManager::manageSub(PtrHolderBase* holder)
{
    impl->pointerHolders.push_back(holder);
}


void ExtensionManager::setPluginWhitelistForToolBars(const std::vector<const char*>& pluginNames)
{
    for(auto& name : pluginNames){
        pluginWhitelistForToolBars.insert(name);
    }
}


void ExtensionManager::setToolBarWhitelist(const std::vector<const char*>& toolBarNames)
{
    for(auto& name : toolBarNames){
        toolBarWhitelist.insert(name);
    }
}


void ExtensionManager::addToolBar(ToolBar* toolBar)
{
    if(impl->isBeforeAddingToolBars){
        if(!pluginWhitelistForToolBars.empty() || !toolBarWhitelist.empty()){
            if(pluginWhitelistForToolBars.find(impl->moduleName) == pluginWhitelistForToolBars.end()){
                impl->doCheckToolBarWhitelist = true;
            }
        }
        impl->isBeforeAddingToolBars = false;
    }
    
    manage(toolBar);

    bool isEnabled = true;
    if(impl->doCheckToolBarWhitelist){
        if(toolBarWhitelist.find(toolBar->name()) == toolBarWhitelist.end()){
            isEnabled = false;
        }
    }
    if(isEnabled){
        toolBar->setWindowTitle(dgettext(impl->textDomain.c_str(), toolBar->name().c_str()));
        MainWindow::instance()->addToolBar(toolBar);
    }
}


void ExtensionManager::mountToolBar(ToolBar* toolBar)
{
    toolBar->setVisibleByDefault();

    // Remove the existing tool bars that have the same name with the new tool bar
    auto p = impl->pointerHolders.begin();
    while(p != impl->pointerHolders.end()){
        auto toolBarHolder = dynamic_cast<PtrHolder<ToolBar*>*>(*p);
        if(toolBarHolder){
            auto existingToolBar = toolBarHolder->pointer;
            if(existingToolBar->name() == toolBar->name()){
                MainWindow::instance()->removeToolBar(existingToolBar);
                p = impl->deleteManagedObject(p);
                continue;
            }
        }
        ++p;
    }

    addToolBar(toolBar);
}


deque<ExtensionManager::PtrHolderBase*>::iterator
ExtensionManager::Impl::deleteManagedObject(deque<PtrHolderBase*>::iterator iter)
{
    auto holder = *iter;
    delete holder;
    return pointerHolders.erase(iter);
}



void ExtensionManager::Impl::deleteManagedObjects()
{
    auto p = pointerHolders.begin();
    while(p != pointerHolders.end()){
        p = deleteManagedObject(p);
    }
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
