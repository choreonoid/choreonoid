#include "YawMomentCompensationDialog.h"
#include <cnoid/Plugin>
#include <cnoid/MainMenu>
#include <cnoid/MenuManager>
#include <cnoid/BodyMotionGenerationBar>
#include <QIcon>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

class BodyMotionFilterPlugin : public Plugin
{
public:
    BodyMotionFilterPlugin()
        : Plugin("BodyMotionFilter")
    {
        require("PoseSeq");
    }

    virtual bool initialize()
    {
        YawMomentCompensationDialog::initialize(this);

        auto mainMenu = MainMenu::instance();
        mainMenu->add_Filters_Item(
            _("Yaw moment compensation"),
            [this](){ YawMomentCompensationDialog::instance()->show(); });

        BodyMotionGenerationBar::instance()->addMotionFilter(
            "yaw_moment_compensation",
            QIcon(":/BodyMotionFilter/icon/yaw-moment-compensation.svg"),
            _("Enable yaw moment compensation"),
            [](BodyItem* bodyItem, BodyMotionItem* motionItem){
                return YawMomentCompensationDialog::instance()->applyFilter(bodyItem, motionItem);
            });

        return true;
    }

    virtual bool finalize()
    {
        return true;
    }
};

}

CNOID_IMPLEMENT_PLUGIN_ENTRY(BodyMotionFilterPlugin);
