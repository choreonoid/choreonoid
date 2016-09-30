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

class RTSNameTreeWidget : public TreeWidget
{
    Q_OBJECT

private :
    void mouseMoveEvent(QMouseEvent *event);
};

}

#endif
