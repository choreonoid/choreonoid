/*!
 * @brief  This is a definition of RTSystemEditorPlugin.
 * @author Hisashi Ikari
 * @file
 */
#ifndef CNOID_OPENRTM_PLUGIN_RTS_ROPERTIES_VIEW_H_INCLUDED
#define CNOID_OPENRTM_PLUGIN_RTS_ROPERTIES_VIEW_H_INCLUDED

#include <cnoid/View>
#include <rtm/idl/RTC.hh>

#include <cnoid/Dialog>
#include <cnoid/CheckBox>
#include <cnoid/ComboBox>
#include <cnoid/LineEdit>

namespace cnoid {
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

class SettingDialog : public Dialog
{
public:
    SettingDialog();

private:
    CheckBox* chkLog;
    ComboBox* cmbLogLevel;

    LineEdit* leSetting;

    LineEdit* leName;
    LineEdit* leVersion;
#ifndef OPENRTM_VERSION110
    LineEdit* leHeartBeat;
#endif

    void oKClicked();
    void rejected();
    void logChanged(bool state);
};

};
#endif
