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
#include "LazyCaller.h"
#include <cnoid/GettextUtil>
#include <cnoid/ExecutablePath>
#include <QMenuBar>
#include <boost/algorithm/string.hpp>
#include <set>
#include <deque>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class ExtensionManager::Impl
{
public:
    ExtensionManager* self;
    string moduleName;
    string textDomain;
    deque<PtrHolderBase*> pointerHolders;
    Signal<void()> sigSystemUpdated;
    Signal<void()> sigReleaseRequest;
    std::unique_ptr<MenuManager> menuManager;
    std::unique_ptr<ItemManager> itemManager;
    std::unique_ptr<ViewManager> viewManager;

    static set<Impl*> instances;
    static LazyCaller emitSigSystemUpdatedLater;

    Impl(ExtensionManager* self, const std::string& moduleName, bool isPlugin);
    ~Impl();
    deque<PtrHolderBase*>::iterator deleteManagedObject(deque<PtrHolderBase*>::iterator iter);
    void deleteManagedObjects();
    static void emitSigSystemUpdated();
};

set<ExtensionManager::Impl*> ExtensionManager::Impl::instances;

LazyCaller ExtensionManager::Impl::emitSigSystemUpdatedLater(
    [](){ ExtensionManager::Impl::emitSigSystemUpdated(); });

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
    sigReleaseRequest();
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
        impl->itemManager.reset(new ItemManager(impl->moduleName, menuManager()));
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


void ExtensionManager::addToolBar(ToolBar* toolBar)
{
    toolBar->setWindowTitle(dgettext(impl->textDomain.c_str(), toolBar->name().c_str()));

    manage(toolBar);
    MainWindow::instance()->addToolBar(toolBar);
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
            if(existingToolBar->objectName() == toolBar->objectName()){
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


SignalProxy<void()> ExtensionManager::sigSystemUpdated()
{
    return impl->sigSystemUpdated;
}


void ExtensionManager::notifySystemUpdate()
{
    Impl::emitSigSystemUpdatedLater();
}


void ExtensionManager::Impl::emitSigSystemUpdated()
{
    for(auto& instance : instances){
        instance->sigSystemUpdated();
    }
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
