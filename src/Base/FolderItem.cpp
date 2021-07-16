/**
   @author Shin'ichiro Nakaoka
*/

#include "FolderItem.h"
#include "ItemManager.h"
#include "PutPropertyFunction.h"
#include "Archive.h"
#include "gettext.h"

using namespace cnoid;


void FolderItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager()
        .registerClass<FolderItem>(N_("FolderItem"))
        .addCreationPanel<FolderItem>();
}


FolderItem::FolderItem()
{
    
}


FolderItem::FolderItem(const FolderItem& org)
    : Item(org),
      category_(org.category_)
{

}


Item* FolderItem::doDuplicate() const
{
    return new FolderItem(*this);
}


bool FolderItem::store(Archive& archive)
{
    if(!category_.empty()){
        archive.write("category", category_, DOUBLE_QUOTED);
    }
    return true;
}


bool FolderItem::restore(const Archive& archive)
{
    archive.read("category", category_);
    return true;
}
