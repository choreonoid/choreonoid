#include "CoordinateFrameListItem.h"
#include "ItemManager.h"
#include "LocatableItem.h"
#include "PutPropertyFunction.h"
#include "Archive.h"
#include <cnoid/CoordinateFrameList>
#include <cnoid/ConnectionSet>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

Signal<void(CoordinateFrameListItem* frameListItem)> sigInstanceAddedOrUpdated_;

}

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


SignalProxy<void(CoordinateFrameListItem* frameListItem)>
CoordinateFrameListItem::sigInstanceAddedOrUpdated()
{
    return ::sigInstanceAddedOrUpdated_;
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


void CoordinateFrameListItem::onPositionChanged()
{
    ::sigInstanceAddedOrUpdated_(this);
}


CoordinateFrameList* CoordinateFrameListItem::frameList()
{
    return impl->frameList;
}


const CoordinateFrameList* CoordinateFrameListItem::frameList() const
{
    return impl->frameList;
}


LocatableItem* CoordinateFrameListItem::getParentLocatableItem()
{
    if(auto locatableItem = findOwnerItem<LocatableItem>()){
        return locatableItem;
    }
    return nullptr;
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
