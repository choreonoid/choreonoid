#ifndef CNOID_OPENRTM_PLUGIN_RTS_NAME_SERVER_VIEW_H
#define CNOID_OPENRTM_PLUGIN_RTS_NAME_SERVER_VIEW_H

#include "RTCWrapper.h"
#include "RTSCommonUtil.h"
#include <cnoid/CorbaUtil>
#include <cnoid/TreeWidget>
#include <cnoid/MenuManager>
#include <cnoid/View>

namespace cnoid {

class RTSNameServerViewImpl;
class RTSVItem;

class RTSNameServerView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
    static RTSNameServerView* instance();

    RTSNameServerView();
    virtual ~RTSNameServerView();

    SignalProxy<void(const std::list<NamingContextHelper::ObjectInfo>&)> sigSelectionChanged();

    std::list<NamingContextHelper::ObjectInfo> getSelection();

    //Proxy to RTSNameServerViewImpl
    void updateView();
    void setSelection(std::string RTCname, std::string RTCfullPath, NamingContextHelper::ObjectInfo nsInfo);

protected:
    virtual void onActivated() override;
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;

private:
    RTSNameServerViewImpl * impl;
};


class RTSNameTreeWidget : public TreeWidget
{
    Q_OBJECT
private:
    MenuManager menuManager;

    void mouseMoveEvent(QMouseEvent* event);
    void mousePressEvent(QMouseEvent* event);

    void showIOR();
    void deleteFromView();
    void deleteFromNameService();
    void addContext();
    void addObject();

    void activateComponent();
    void deactivateComponent();
    void resetComponent();
    void finalizeComponent();
    void startExecutionContext();
    void stopExecutionContext();

    void restartManager();
    void shutdownManager();
};


enum CORBA_KIND { KIND_CATEGORY, KIND_HOST, KIND_MANAGER, KIND_RTC_MANAGER, KIND_MODULE, KIND_SERVER, KIND_RTC, KIND_FOLDER, KIND_OTHER };


class RTSVItem : public QTreeWidgetItem, public RTCWrapper
{
public:
    RTSVItem();
    RTSVItem(const NamingContextHelper::ObjectInfo& info, RTC::RTObject_ptr rtc = 0);

    NamingContextHelper::ObjectInfo info_;
    NameServerInfo nsInfo_;
    CORBA_KIND kind_;
    bool removing_;
};

}

#endif
