/**
   @author Shin'ichiro Nakaoka
   @author Hisashi Ikari
*/

#ifndef CNOID_OPENRTM_PLUGIN_RTS_NAME_SERVER_VIEW_H_INCLUDED
#define CNOID_OPENRTM_PLUGIN_RTS_NAME_SERVER_VIEW_H_INCLUDED

#include <cnoid/View>
#include <cnoid/CorbaUtil>

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

//    virtual void onActivated();
//    TreeWidget* getTreeWidget();
//    void setConnection();

private:
    RTSNameServerViewImpl* impl;
};
}

#endif
