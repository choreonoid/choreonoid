/**
   @author Shin'ichiro Nakaoka
*/

#include "FolderItem.h"
#include "ItemManager.h"
#include "gettext.h"

using namespace cnoid;

FolderItem::FolderItem()
{
    
}


FolderItem::FolderItem(const FolderItem& org)
    : Item(org)
{

}


FolderItem::~FolderItem()
{

}


Item* FolderItem::doDuplicate() const
{
    return new FolderItem(*this);
}


bool FolderItem::store(Archive& archive)
{
    return true;
}


bool FolderItem::restore(const Archive& archive)
{
    return true;
}


void FolderItem::initializeClass(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        ItemManager& im = ext->itemManager();
        im.registerClass<FolderItem>(N_("FolderItem"));
        im.addCreationPanel<FolderItem>();
        initialized = true;
    }
}
