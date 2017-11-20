/**
   @author Shin'ichiro Nakaoka
   @author Hisashi Ikari
*/

#ifndef CNOID_OPENRTM_PLUGIN_RTS_NAME_SERVER_VIEW_H_INCLUDED
#define CNOID_OPENRTM_PLUGIN_RTS_NAME_SERVER_VIEW_H_INCLUDED

#include <cnoid/View>
#include <cnoid/PolymorphicPointerArray>
#include <cnoid/CorbaUtil>
#include <cnoid/TreeWidget>

#include <cnoid/MenuManager>
#include "OpenRTMItem.h"
#include <rtm/idl/RTC.hh>

using namespace cnoid;

namespace cnoid
{
class RTSNameServerViewImpl;

/*!
 * @brief It is a screen of RTC list.
 */
class RTSNameServerView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
    static RTSNameServerView* instance();

    RTSNameServerView();
    virtual ~RTSNameServerView();

    SignalProxy<void(const std::list<NamingContextHelper::ObjectInfo>&)>
    	sigSelectionChanged();
    SignalProxy<void(std::string, int)> sigLocationChanged();

    const std::string getHost();
    int getPort();
    std::list<NamingContextHelper::ObjectInfo> getSelection();
    void setSelection(std::string RTCname);
    void updateView();

private:
    RTSNameServerViewImpl* impl;
};

class RTSNameTreeWidget : public TreeWidget {
Q_OBJECT

private :
	MenuManager menuManager;

  void mouseMoveEvent(QMouseEvent* event);
	void mousePressEvent(QMouseEvent* event);

	void activateComponent();
	void deactivateComponent();
	void resetComponent();
	void finalizeComponent();
	void startExecutionContext();
	void stopExecutionContext();
};

class RTSVItem : public QTreeWidgetItem, public RTCWrapper {
public:
	RTSVItem(const NamingContextHelper::ObjectInfo& info, RTC::RTObject_ptr rtc = 0);

	NamingContextHelper::ObjectInfo info_;
};

}

#endif
