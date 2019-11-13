#include "CoordinateFrameListItem.h"
#include <cnoid/CoordinateFrameSet>
#include <cnoid/CoordinateFrameList>
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/ConnectionSet>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class CoordinateFrameListItem::Impl
{
public:
    CoordinateFrameListPtr frameList;

    Impl();
    Impl(const Impl& org);
};

}


void CoordinateFrameListItem::initializeClass(ExtensionManager* ext)
{
    auto& im = ext->itemManager();
    im.registerClass<CoordinateFrameListItem>(N_("CoordinateFrameListItem"));
}


CoordinateFrameListItem::CoordinateFrameListItem()
{
    impl = new Impl;
}


CoordinateFrameListItem::Impl::Impl()
{
    frameList = new CoordinateFrameList;
}


CoordinateFrameListItem::CoordinateFrameListItem(const CoordinateFrameListItem& org)
    : Item(org)
{
    impl = new Impl(*org.impl);
}


CoordinateFrameListItem::Impl::Impl(const Impl& org)
{
    frameList = new CoordinateFrameList(*org.frameList);
}


CoordinateFrameListItem::~CoordinateFrameListItem()
{
    delete impl;
}


Item* CoordinateFrameListItem::doDuplicate() const
{
    return new CoordinateFrameListItem(*this);
}


CoordinateFrameList* CoordinateFrameListItem::frameList()
{
    return impl->frameList;
}


const CoordinateFrameList* CoordinateFrameListItem::frameList() const
{
    return impl->frameList;
}


void CoordinateFrameListItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Num coordinate frames"), impl->frameList->numFrames());
}


bool CoordinateFrameListItem::store(Archive& archive)
{
    return impl->frameList->write(archive);
}


bool CoordinateFrameListItem::restore(const Archive& archive)
{
    impl->frameList->resetIdCounter();
    impl->frameList->read(archive);
    return true;
}
