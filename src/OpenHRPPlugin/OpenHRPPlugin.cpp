/*! @file
  @author Shin'ichiro Nakaoka
*/

#include "OpenHRPControllerItem.h"
#include "OnlineViewerServer.h"
#include "OpenHRPOnlineViewerItem.h"
#include "OpenHRPClockGeneratorItem.h"

#ifdef OPENHRP_3_1
#include "OpenHRPInterpreterServiceItem.h"
#endif

#include <cnoid/Plugin>
#include <cnoid/ItemManager>
#include "gettext.h"

using namespace cnoid;

namespace {
    
#ifdef OPENHRP_3_0
#define PLUGIN_NAME "OpenHRP3.0"
#define ITEM_NAME N_("OpenHRP3.0ControllerItem")
#elif OPENHRP_3_1
#define PLUGIN_NAME "OpenHRP3.1"
#define ITEM_NAME N_("OpenHRP3.1ControllerItem")
#endif
    
class OpenHRPPlugin : public Plugin
{
public:
    OpenHRPPlugin() : Plugin(PLUGIN_NAME) {
        require("Body");
        require("OpenRTM");
    }
        
    virtual bool initialize() {

        itemManager().registerClass<OpenHRPControllerItem>(ITEM_NAME);
        itemManager().addCreationPanel<OpenHRPControllerItem>();

        OpenHRPOnlineViewerItem::initializeClass(this);

#ifdef OPENHRP_3_1
        OpenHRPInterpreterServiceItem::initializeClass(this);
        OpenHRPClockGeneratorItem::initialize(this);
#endif

        return true;
    }
        
    virtual bool finalize() {
        return true;
    }
};

}


CNOID_IMPLEMENT_PLUGIN_ENTRY(OpenHRPPlugin);
