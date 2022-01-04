/**
   @author Shin'ichiro NAKAOKA
*/

#ifndef CNOID_BASE_EXTENSION_MANAGER_H
#define CNOID_BASE_EXTENSION_MANAGER_H

#include <string>
#include <vector>
#include <functional>
#include "exportdecl.h"

namespace cnoid {

class ToolBar;
class Archive;
class ItemManager;
class ViewManager;
class MenuManager;
class OptionManager;

class CNOID_EXPORT ExtensionManager
{
public:

    ExtensionManager(const std::string& moduleName, bool isPlugin);
    ExtensionManager(const std::string& moduleName, const std::string& version, bool isPlugin);
    virtual ~ExtensionManager();

    const std::string& name() const;
    const std::string& textDomain() const;

    ItemManager& itemManager();
    ViewManager& viewManager();
    MenuManager& menuManager();
    OptionManager& optionManager();

    //! \note This function can only be called before calling the App::initialize function.
    static void setPluginWhitelistForToolBars(const std::vector<const char*>& pluginNames);
    //! \note This function can only be called before calling the App::initialize function.
    static void setToolBarWhitelist(const std::vector<const char*>& toolBarNames);
    
    void addToolBar(ToolBar* toolBar);

    /**
       This function is similar to the addToolBar function, but it always shows the tool bar
       and replaces an existing tool bar if the tool bar has the same name with the new tool bar.
       The function is mainly provided for the script use. A script may be executed repeatedly
       without restarting Choreonoid, especially during its development. Even in such a situation,
       the script can easily introduce a tool bar just by using this function without coding the
       operation to show the tool bar and remove the tool bar instance that has been added by the
       script.
    */
    void mountToolBar(ToolBar* toolBar);

    template <class PointerType> PointerType manage(PointerType pointer) {
        manageSub(new PtrHolder<PointerType>(pointer));
        return pointer;
    }

    void setProjectArchiver(
        const std::string& name,
        std::function<bool(Archive&)> storeFunction,
        std::function<void(const Archive&)> restoreFunction);

    void setProjectArchiver(
        std::function<bool(Archive&)> storeFunction,
        std::function<void(const Archive&)> restoreFunction);
        
private:
    ExtensionManager(const ExtensionManager& org);

    struct CNOID_EXPORT PtrHolderBase {
        virtual ~PtrHolderBase();
    };

    // smart pointer version
    template <class PointerType> struct PtrHolder : public PtrHolderBase {
        PtrHolder(PointerType pointer) : pointer(pointer) { }
        virtual ~PtrHolder() { }
        PointerType pointer;
    };

    // raw pointer version
    template <class Object> struct PtrHolder<Object*> : public PtrHolderBase {
        PtrHolder(Object* pointer) : pointer(pointer) { }
        virtual ~PtrHolder() { delete pointer; }
        Object* pointer;
    };

    void manageSub(PtrHolderBase* holder);

    class Impl;
    Impl* impl;
};

}

#endif
