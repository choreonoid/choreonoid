/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_VIEW_MANAGER_H
#define CNOID_BASE_VIEW_MANAGER_H

#include "View.h"
#include "ExtensionManager.h"
#include "Archive.h"
#include <cnoid/Signal>
#include "exportdecl.h"

namespace cnoid {

class ExtensionManager;
class ViewManagerImpl;


class CNOID_EXPORT ViewManager
{
public:
    static void initializeClass(ExtensionManager* ext);

    ViewManager(ExtensionManager* ext);
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

    enum InstantiationType {
        SINGLE_DEFAULT,
        SINGLE_OPTIONAL,
        MULTI_DEFAULT,
        MULTI_OPTIONAL
    };

    /**
       \return If itype is SINGLE_DEFAULT or MULTI_DEFAULT, the default instance created by
       this function is returned. Otherwise null pointer is returned.
    */
    template <class ViewType> ViewType* registerClass(
        const std::string& className, const std::string& defaultInstanceName,
        ViewManager::InstantiationType itype = ViewManager::SINGLE_OPTIONAL) {
        return static_cast<ViewType*>(
            registerClassSub(typeid(ViewType), className, defaultInstanceName, itype, new Factory<ViewType>()));
    }

    void registerClassAlias(const std::string& alias, const std::string& orgClassName);
    
    static ViewClass* viewClass(const std::type_info& view_type_info);

    // get or create the primal instance of the specified view type
    static View* getOrCreateView(const std::string& moduleName, const std::string& className);

    // get or create the instance of the specified view type and instance name
    static View* getOrCreateView(
        const std::string& moduleName, const std::string& className, const std::string& instanceName);

    // for loading the view layout format of the version 1.4 or earlier
    static View* getOrCreateViewOfDefaultName(const std::string& defaultName);

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

    static bool storeViewStates(ArchivePtr archive, const std::string& key);

    class ViewStateInfo {
    public:
        ViewStateInfo();
        ~ViewStateInfo();
        operator bool() const { return (data != 0); }
    private:
        void* data;
        friend class ViewManager;
    };

    static bool restoreViews(ArchivePtr archive, const std::string& key, ViewStateInfo& out_viewStateInfo);
    static bool restoreViewStates(ViewStateInfo& info);

    static SignalProxy<void(View* view)> sigViewCreated();
    static SignalProxy<void(View* view)> sigViewActivated();
    static SignalProxy<void(View* view)> sigViewDeactivated();
    static SignalProxy<void(View* view)> sigViewRemoved();

private:
    ViewManager(const ViewManager&) { }
    
    View* registerClassSub(
        const std::type_info& view_type_info, const std::string& className, const std::string& defaultInstanceName,
        InstantiationType itype, FactoryBase* factory);
    static View* getOrCreateSpecificTypeView(
        const std::type_info& view_type_info, const std::string& instanceName, bool doMountCreatedView);
    static View* findSpecificTypeView(const std::type_info& view_type_info, const std::string& instanceName);
        
    ViewManagerImpl* impl;
};

}

#endif
