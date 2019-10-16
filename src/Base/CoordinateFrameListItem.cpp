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
    CoordinateFrameListPtr frames;

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
    frames = new CoordinateFrameList;
}


CoordinateFrameListItem::CoordinateFrameListItem(const CoordinateFrameListItem& org)
    : Item(org)
{
    impl = new Impl(*org.impl);
}


CoordinateFrameListItem::Impl::Impl(const Impl& org)
{
    frames = new CoordinateFrameList(*org.frames);
}


CoordinateFrameListItem::~CoordinateFrameListItem()
{
    delete impl;
}


Item* CoordinateFrameListItem::doDuplicate() const
{
    return new CoordinateFrameListItem(*this);
}


CoordinateFrameList* CoordinateFrameListItem::frames()
{
    return impl->frames;
}


const CoordinateFrameList* CoordinateFrameListItem::frames() const
{
    return impl->frames;
}


void CoordinateFrameListItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Num coordinate frames"), impl->frames->numFrames());
}


bool CoordinateFrameListItem::store(Archive& archive)
{
    return impl->frames->write(archive);
}


bool CoordinateFrameListItem::restore(const Archive& archive)
{
    impl->frames->resetIdCounter();
    impl->frames->read(archive);
    return true;
}
