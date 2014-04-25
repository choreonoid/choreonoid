/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "GRobotBar.h"
#include "GRobotControllerItem.h"
#include <cnoid/App>
#include <cnoid/Config>
#include <cnoid/Plugin>
#include <cnoid/ItemManager>
#include "gettext.h"

namespace cnoid {

class GRobotPlugin : public Plugin
{
public:
        
    GRobotPlugin() : Plugin("GRobot") {
        require("Body");
    }
    
    virtual bool initialize() {

        addToolBar(GRobotBar::instance());

        itemManager().registerClass<GRobotControllerItem>(N_("GRobotControllerItem"));
        itemManager().addCreationPanel<GRobotControllerItem>();
            
        return true;
    }

    virtual const char* description() {
        static std::string text =
            str(fmt(_("GRobot Plugin Version %1%\n")) % CNOID_FULL_VERSION_STRING) +
            "\n" +
            _("This plugin has been developed by Shin'ichiro Nakaoka and Choreonoid Development Team, AIST, "
              "and is distributed as a part of the Choreonoid package.\n"
              "\n") +
            LGPLtext();
        return text.c_str();
    }
};
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(cnoid::GRobotPlugin)
