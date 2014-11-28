/*!
 * @brief  This is a definition of RTSystemEditorPlugin.
 * @author Hisashi Ikari 
 * @file
 */
#ifndef CNOID_OPENRTM_PLUGIN_RTS_ROPERTIES_VIEW_H_INCLUDED
#define CNOID_OPENRTM_PLUGIN_RTS_ROPERTIES_VIEW_H_INCLUDED

#include <cnoid/View>
//#include <boost/shared_ptr.hpp>

namespace cnoid 
{
class   RTSPropertiesViewImpl;
    //typedef boost::shared_ptr<RTSPropertiesViewImpl> RTSPropertiesViewImplPtr;

/*!
 * @brief It is a screen of RTC property.
 */
class RTSPropertiesView : public cnoid::View
{
	//friend class RTSPropertyTreeWidget;
public:
	static void initializeClass(ExtensionManager* ext);
	static RTSPropertiesView* instance();

	RTSPropertiesView();
	~RTSPropertiesView();

	void setSelectionChangedConnection();
	void clearSelectionChangedConnection();
	//virtual void onActivated();
	//virtual void onDeactivated();

	//void setConnection  (std::string name, int port);
	//void showProperties (std::string name);
	//void showProperties (std::string srtc, std::string sport, std::string trtc, std::string tport, std::string id);
	//void clearProperties();

private:
	RTSPropertiesViewImpl* impl;
};

}; // end of namespace
 
#endif
