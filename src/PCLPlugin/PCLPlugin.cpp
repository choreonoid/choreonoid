#include "PCLPlugin.h"
#include "PointSetToMeshConversionDialog.h"
#include <cnoid/MainMenu>
#include <cnoid/MenuManager>
#include <cnoid/ItemTreeView>
#include <cnoid/PointSetItem>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

PCLPlugin* instance_ = nullptr;

}


PCLPlugin::PCLPlugin()
    : Plugin("PCL")
{
    require("Body");    
    instance_ = this;
}


PCLPlugin* PCLPlugin::instance()
{
    return instance_;
}


bool PCLPlugin::initialize()
{
    PointSetToMeshConversionDialog::initializeClass(this);
    
    auto mainMenu = MainMenu::instance();

    mainMenu->add_Filters_Item(
        _("Convert point set to mesh"),
        [this](){ PointSetToMeshConversionDialog::instance()->show(); });

    ItemTreeView::customizeContextMenu<PointSetItem>(
        [this](PointSetItem* item, MenuManager& menu, ItemFunctionDispatcher menuFunction){
            menu.addItem(_("Convert to mesh"))->sigTriggered().connect(
                [this, item](){ PointSetToMeshConversionDialog::instance()->show(item); });
            menu.addSeparator();
            menuFunction.dispatchAs<Item>(item);
        });
    
    return true;
}


CNOID_IMPLEMENT_PLUGIN_ENTRY(PCLPlugin);
