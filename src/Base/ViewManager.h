/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_VIEW_MANAGER_H
#define CNOID_BASE_VIEW_MANAGER_H

#include "View.h"
#include "ExtensionManager.h"
#include <cnoid/Signal>
#include <string>
#include <vector>
#include <set>
#include <typeinfo>
#include "exportdecl.h"

namespace cnoid {

class Archive;

class CNOID_EXPORT ViewManager
{
public:
    static void initializeClass(ExtensionManager* ext);

    //! \note This function can only be called before calling the App::initialize function.
    static void setPluginWhitelist(const std::vector<const char*>& pluginNames);

    enum InstantiationFlags {
        Single = 0,
        Multiple = 1,
        Permanent= 2,
        // deprecated
        SINGLE_OPTIONAL = Single,
        SINGLE_DEFAULT = Single | Permanent,
        MULTI_OPTIONAL = Multiple,
        MULTI_DEFAULT = Multiple | Permanent
    };

    struct WhiteListElement {
        const char* viewClassName;
        int instantiationFlags;
        bool hasCustomInstantiationFlags;

        WhiteListElement(const char* viewClassName)
            : viewClassName(viewClassName),
              hasCustomInstantiationFlags(false) { }
        WhiteListElement(const char* viewClassName, int instantiationFlags)
            : viewClassName(viewClassName),
              instantiationFlags(instantiationFlags),
              hasCustomInstantiationFlags(true) { }
    };

    //! \note This function can only be called before calling the App::initialize function.
    static void setViewWhitelist(const std::vector<WhiteListElement>& elements);

    static void setClassAlias(const std::string& alias, const std::string& orgClassName);

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

    template <class ViewType>
    ViewManager& registerClass(
        const std::string& className, const std::string& defaultInstanceName, int instantiationFlags = Single) {
        registerClass_(
            typeid(ViewType), className, defaultInstanceName, instantiationFlags, new Factory<ViewType>());
        return *this;
    }

    [[deprecated("Use setClassAlias")]]
    ViewManager& registerClassAlias(const std::string& alias, const std::string& orgClassName);

    class ViewClass : public Referenced
    {
    public:
        virtual const std::string& moduleName() const = 0;
        virtual const std::string& name() const = 0;
        virtual const std::string& translatedName() const = 0;
        virtual const std::string& defaultInstanceName() const = 0;
        virtual const std::string& translatedDefaultInstanceName() const = 0;
        virtual bool isSingleton() const = 0;
        virtual bool hasPermanentInstance() const = 0;
        virtual std::vector<View*> instances() const = 0;
        virtual View* getOrCreateView() = 0;
        virtual View* getOrCreateView(const std::string& name) = 0;
        virtual View* createViewWithDialog() = 0;

    protected:
        ViewClass() = default;
        ViewClass(const ViewClass& org) = delete;
    };

    static std::vector<ViewClass*> viewClasses();

    // get or create the primal instance of the specified view type
    static View* getOrCreateView(const std::string& moduleName, const std::string& className);

    template <class ViewType> static ViewType* getOrCreateView(bool doMountCreatedView = false) {
        return static_cast<ViewType*>(getOrCreateSpecificTypeView(typeid(ViewType), std::string(), doMountCreatedView));
    }

    template <class ViewType> static ViewType* getOrCreateView(const std::string& instanceName, bool doMountCreatedView = false) {
        return static_cast<ViewType*>(getOrCreateSpecificTypeView(typeid(ViewType), instanceName, doMountCreatedView));
    }

    // The following functions will be deprecated. Use the viewClasses function to get view instances.
    static std::vector<View*> allViews();
    static std::vector<View*> activeViews();
    
    template <class ViewType> static ViewType* findView() {
        return static_cast<ViewType*>(findSpecificTypeView(typeid(ViewType), std::string()));
    }

    template <class ViewType> static ViewType* findView(const std::string& instanceName) {
        return static_cast<ViewType*>(findSpecificTypeView(typeid(ViewType), instanceName));
    }

    static void deleteView(View* view);
    static void deleteUnmountedViews();

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
