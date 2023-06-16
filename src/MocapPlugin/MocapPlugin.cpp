#include "MarkerMotionItem.h"
#include "SkeletonMotionItem.h"
#include "MocapMappingItem.h"
#include "SkeletonToMarkerMotionConverter.h"
#include "MocapConversionToBodyMotionDialog.h"
#include <cnoid/Plugin>
#include <cnoid/Config>
#include <cnoid/MainMenu>
#include <cnoid/MenuManager>
#include <cnoid/ItemTreeView>
#include <cnoid/MessageView>
#include <cnoid/RootItem>
#include <cnoid/BodyItem>
#include <cnoid/MessageOut>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {
  
class MocapPlugin : public Plugin
{
public:
    MocapPlugin();
    virtual bool initialize() override;
    void convertSelectedSkeletonMotionItemsToMarkerMotionItems();
    void convertSkeletonMotionItemToMarkerMotionItem(SkeletonMotionItem* skeletonMotionItem);
    void onContextMenuItemToConvertToBodyMotionActivated(Item* motionItem);
};

}


MocapPlugin::MocapPlugin()
    : Plugin("Mocap")
{
    require("Body");
}


bool MocapPlugin::initialize()
{
    SkeletonMotionItem::initialize(this);
    MarkerMotionItem::initialize(this);
    MocapMappingItem::initialize(this);
    MocapConversionToBodyMotionDialog::initialize(this);
    
    auto mainMenu = MainMenu::instance();
    
    // This importer is registered last to avoid the duplicated separators
    mainMenu->add_File_Import_Item(
        _("Acclaim ASF/AMC mocap file"),
        [](){ SkeletonMotionItem::loadAMCfiles(); });
    
    mainMenu->add_Filters_Item(
        _("Convert skeleton motion to marker motion"),
        [this](){ convertSelectedSkeletonMotionItemsToMarkerMotionItems(); });

    mainMenu->add_Filters_Item(
        _("Convert skeleton / marker motion to body motion"),
        [](){ MocapConversionToBodyMotionDialog::getOrCreateInstance()->show(); });

    ItemTreeView::customizeContextMenu<SkeletonMotionItem>(
        [this](SkeletonMotionItem* item, MenuManager& menu, ItemFunctionDispatcher menuFunction){
            menu.addItem(_("Convert to marker motion"))->sigTriggered().connect(
                [this, item](){ convertSelectedSkeletonMotionItemsToMarkerMotionItems(); });
            menu.addItem(_("Convert to body motion"))->sigTriggered().connect(
                [this, item](){ onContextMenuItemToConvertToBodyMotionActivated(item); });
            menu.addSeparator();
            menuFunction.dispatchAs<Item>(item);
        });

    ItemTreeView::customizeContextMenu<MarkerMotionItem>(
        [this](MarkerMotionItem* item, MenuManager& menu, ItemFunctionDispatcher menuFunction){
            menu.addItem(_("Convert to body motion"))->sigTriggered().connect(
                [this, item](){ onContextMenuItemToConvertToBodyMotionActivated(item); });
            menu.addSeparator();
            menuFunction.dispatchAs<Item>(item);
        });

    return true;
}


void MocapPlugin::convertSelectedSkeletonMotionItemsToMarkerMotionItems()
{
    for(auto& item : RootItem::instance()->selectedItems<SkeletonMotionItem>()){
        convertSkeletonMotionItemToMarkerMotionItem(item);
    }
}


void MocapPlugin::convertSkeletonMotionItemToMarkerMotionItem(SkeletonMotionItem* skeletonMotionItem)
{
    MocapMappingPtr mocapMapping;
    auto mocapMappingItem = skeletonMotionItem->findOwnerItem<MocapMappingItem>();
    auto mout = MessageOut::master();
    
    if(mocapMappingItem){
        mocapMapping = mocapMappingItem->mocapMapping();
        mout->put(
            format(_("Converting {0} to the marker motion with {1} ..."),
                   skeletonMotionItem->name(), mocapMappingItem->name()));
    } else {
        mout->put(format(_("Converting {0} to the marker motion ..."), skeletonMotionItem->name()));
    }
                
    SkeletonToMarkerMotionConverter converter;
    MarkerMotionItemPtr converted = new MarkerMotionItem;
    
    if(!converter.convert(*skeletonMotionItem->motion(), mocapMapping, *converted->motion())){
        mout->putln(_("failed."));

    } else {
        converted->setTemporary(true);
        converted->setName(skeletonMotionItem->name() + "-markers");
        skeletonMotionItem->parentItem()->insertChild(skeletonMotionItem->nextItem(), converted);
        converted->notifyUpdate();
        mout->putln(_("OK."));
    }
}


void MocapPlugin::onContextMenuItemToConvertToBodyMotionActivated(Item* motionItem)
{
    if(RootItem::instance()->selectedItems<BodyItem>().empty()){
        showWarningDialog(
            format(_("Select the body item(s) to which {0} will be converted."), motionItem->displayName()));
        return;
    }

    MocapConversionToBodyMotionDialog::getOrCreateInstance()->show();
}


CNOID_IMPLEMENT_PLUGIN_ENTRY(MocapPlugin);
