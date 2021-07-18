/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_VIEW_MANAGER_H
#define CNOID_BASE_VIEW_MANAGER_H

#include "View.h"
#include "ExtensionManager.h"
#include "Archive.h"
#include <cnoid/Signal>
#include <set>
#include "exportdecl.h"

namespace cnoid {

class ExtensionManager;

class CNOID_EXPORT ViewManager
{
public:
    static void initializeClass(ExtensionManager* ext);

    ViewManager(ExtensionManager* ext);
    ViewManager(const ViewManager&) = delete;
    ~ViewManager();

    class FactoryBase {
    public:
        virtual View* create() = 0;
        virtual ~FactoryBase() { }
    };

    template <class ViewType> class Factory : public FactoryBase {
    public:
        virtual View* create() { return new ViewType(); }
    };

    enum InstantiationFlags {
        Single = 0,
        Multiple = 1,
        Default = 2,
        SINGLE_OPTIONAL = Single,
        SINGLE_DEFAULT = Single | Default,
        MULTI_OPTIONAL = Multiple,
        MULTI_DEFAULT = Multiple | Default
    };

    template <class ViewType>
    ViewManager& registerClass(
        const std::string& className, const std::string& defaultInstanceName, int instantiationFlags = Single) {
        registerClass_(
            typeid(ViewType), className, defaultInstanceName, instantiationFlags, new Factory<ViewType>());
        return *this;
    }

    ViewManager& registerClassAlias(const std::string& alias, const std::string& orgClassName);
    
    // get or create the primal instance of the specified view type
    static View* getOrCreateView(const std::string& moduleName, const std::string& className);

    // get or create the instance of the specified view type and instance name
    static View* getOrCreateView(
        const std::string& moduleName, const std::string& className, const std::string& instanceName);

    static std::vector<View*> allViews();
    static std::vector<View*> activeViews();
        
    template <class ViewType> static ViewType* getOrCreateView(bool doMountCreatedView = false) {
        return static_cast<ViewType*>(getOrCreateSpecificTypeView(typeid(ViewType), std::string(), doMountCreatedView));
    }

    template <class ViewType> static ViewType* getOrCreateView(const std::string& instanceName, bool doMountCreatedView = false) {
        return static_cast<ViewType*>(getOrCreateSpecificTypeView(typeid(ViewType), instanceName, doMountCreatedView));
    }

    template <class ViewType> static ViewType* findView() {
        return static_cast<ViewType*>(findSpecificTypeView(typeid(ViewType), std::string()));
    }

    template <class ViewType> static ViewType* findView(const std::string& instanceName) {
        return static_cast<ViewType*>(findSpecificTypeView(typeid(ViewType), instanceName));
    }

    void deleteView(View* view);

    static bool isPrimalInstance(View* view);

    static bool storeViewStates(Archive* archive, const std::string& key, bool isLayoutMode);

    class ViewStateInfo {
    public:
        ViewStateInfo();
        ~ViewStateInfo();
        operator bool() const { return (data != 0); }
    private:
        void* data;
        friend class ViewManager;
    };

    static bool restoreViews(
        Archive* archive, const std::string& key, ViewStateInfo& out_viewStateInfo,
        bool enableMissingPluginWarnings = true);
    static bool restoreViews(
        Archive* archive, const std::string& key, ViewStateInfo& out_viewStateInfo,
        const std::set<std::string>& optionalPlugins);
    static bool restoreViewStates(ViewStateInfo& info);

    static SignalProxy<void(View* view)> sigViewCreated();
    static SignalProxy<void(View* view)> sigViewActivated();
    static SignalProxy<void(View* view)> sigViewDeactivated();
    static SignalProxy<void(View* view)> sigViewRemoved();

    class Impl;

private:
    void registerClass_(
        const std::type_info& view_type_info,
        const std::string& className, const std::string& defaultInstanceName, int instantiationFlags,
        FactoryBase* factory);
    static View* getOrCreateSpecificTypeView(
        const std::type_info& view_type_info, const std::string& instanceName, bool doMountCreatedView);
    static View* findSpecificTypeView(const std::type_info& view_type_info, const std::string& instanceName);
    static bool restoreViews(
        Archive* archive, const std::string& key, ViewManager::ViewStateInfo& out_viewStateInfo,
        const std::set<std::string>* optionalPlugins, bool enableMissingPluginWarnings);

    Impl* impl;
};

}

#endif
