#include "CoordinateFrameItem.h"
#include "CoordinateFrameListItem.h"
#include "ItemManager.h"
#include "PutPropertyFunction.h"
#include "Archive.h"
#include <cnoid/CoordinateFrame>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class CoordinateFrameItem::Impl
{
public:
    GeneralId frameId;

    Impl();
    Impl(const Impl& org);
};

}


void CoordinateFrameItem::initializeClass(ExtensionManager* ext)
{
    auto& im = ext->itemManager();
    im.registerClass<CoordinateFrameItem>(N_("CoordinateFrameItem"));
}


CoordinateFrameItem::CoordinateFrameItem()
{
    impl = new Impl;
}


CoordinateFrameItem::Impl::Impl()
{

}


CoordinateFrameItem::CoordinateFrameItem(const CoordinateFrameItem& org)
    : Item(org)
{
    impl = new Impl(*org.impl);
}


CoordinateFrameItem::Impl::Impl(const Impl& org)
    : frameId(org.frameId)
{

}


CoordinateFrameItem::~CoordinateFrameItem()
{
    delete impl;
}


Item* CoordinateFrameItem::doDuplicate() const
{
    return new CoordinateFrameItem(*this);
}


void CoordinateFrameItem::setFrameId(const GeneralId& id)
{
    impl->frameId = id;
}


const GeneralId& CoordinateFrameItem::frameId() const
{
    return impl->frameId;
}


CoordinateFrameList* CoordinateFrameItem::frameList()
{
    if(auto listItem = dynamic_cast<CoordinateFrameListItem*>(parentItem())){
        return listItem->frameList();
    }
    return nullptr;
}
