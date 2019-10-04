#include "ManipulatorCoordinateFrameItem.h"
#include <cnoid/CoordinateFrameSetItem>
#include <cnoid/ItemManager>
#include "gettext.h"

using namespace std;
using namespace cnoid;

void ManipulatorCoordinateFrameItem::initializeClass(ExtensionManager* ext)
{
    auto& im = ext->itemManager();
    im.registerClass<ManipulatorCoordinateFrameItem>(N_("ManipulatorCoordinateFrameItem"));
    im.addCreationPanel<ManipulatorCoordinateFrameItem>();
}


ManipulatorCoordinateFrameItem::ManipulatorCoordinateFrameItem()
{
    localFrameSetItem()->setName("Tool");
}


ManipulatorCoordinateFrameItem::ManipulatorCoordinateFrameItem(const ManipulatorCoordinateFrameItem& org)
    : CoordinateFrameSetPairItem(org)
{
    localFrameSetItem()->setName("Tool");
}


Item* ManipulatorCoordinateFrameItem::doDuplicate() const
{
    return new ManipulatorCoordinateFrameItem(*this);
}
