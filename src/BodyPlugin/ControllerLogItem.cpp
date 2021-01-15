#include "ControllerLogItem.h"
#include <cnoid/ItemManager>
#include "gettext.h"

using namespace cnoid;

void ControllerLogItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<ControllerLogItem, ReferencedObjectSeqItem>(N_("ControllerLogItem"));
}


ControllerLogItem::ControllerLogItem()
{

}


ControllerLogItem::ControllerLogItem(const ControllerLogItem& org)
    : ReferencedObjectSeqItem(org)
{

}


Item* ControllerLogItem::doDuplicate() const
{
    return new ControllerLogItem(*this);
}

    

