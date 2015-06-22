/*!
 * @brief  This is a definition of RTSystemEditorPlugin.
 * @author Hisashi Ikari 
 * @file
 */
#ifndef CNOID_OPENRTM_PLUGIN_RTS_ROPERTIES_VIEW_H_INCLUDED
#define CNOID_OPENRTM_PLUGIN_RTS_ROPERTIES_VIEW_H_INCLUDED

#include "RTSNameServerView.h"
#include <cnoid/View>
#include <rtm/idl/RTC.hh>
//#include <boost/shared_ptr.hpp>

namespace cnoid 
{
class   RTSPropertiesViewImpl;

class RTSPropertiesView : public cnoid::View
{
public:
    static void initializeClass(ExtensionManager* ext);
    static RTSPropertiesView* instance();

    RTSPropertiesView();
    ~RTSPropertiesView();

    void showConnectionProperties(RTC::PortService_var port, std::string id);

    //void setConnection  (std::string name, int port);
    //void showProperties (std::string name);
    //void showProperties (std::string srtc, std::string sport, std::string trtc, std::string tport, std::string id);
    //void clearProperties();

private:
    RTSPropertiesViewImpl* impl;
};

};
#endif
