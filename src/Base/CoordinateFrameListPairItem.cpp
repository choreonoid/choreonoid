#include "CoordinateFrameListPairItem.h"
#include "CoordinateFrameListItem.h"
#include <cnoid/CoordinateFrameSet>
#include <cnoid/CoordinateFrameList>
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

string baseFrameListLabel;
string localFrameListLabel;

}

namespace cnoid {

class CoordinateFrameListPairItem::Impl
{
public:
    CoordinateFrameListPairItem* self;
    CoordinateFrameSetPairPtr frameSetPair;
    CoordinateFrameListItemPtr baseFrameListItem;
    CoordinateFrameListItemPtr localFrameListItem;

    Impl(CoordinateFrameListPairItem* self);
    Impl(CoordinateFrameListPairItem* self, const Impl& org);
};

}


void CoordinateFrameListPairItem::initializeClass(ExtensionManager* ext)
{
    auto& im = ext->itemManager();
    im.registerClass<CoordinateFrameListPairItem>(N_("CoordinateFrameListPairItem"));
    im.addCreationPanel<CoordinateFrameListPairItem>();
    baseFrameListLabel = "Base";
    localFrameListLabel = "Local";
}


void CoordinateFrameListPairItem::setFrameListLabels(const char* baseFrameLabel, const char* localFrameLabel)
{
    ::baseFrameListLabel = baseFrameLabel;
    ::localFrameListLabel = localFrameLabel;
}


CoordinateFrameListPairItem::CoordinateFrameListPairItem()
{
    impl = new Impl(this);
}


CoordinateFrameListPairItem::Impl::Impl(CoordinateFrameListPairItem* self)
    : self(self)
{
    baseFrameListItem = new CoordinateFrameListItem;
    baseFrameListItem->setName(baseFrameListLabel);
    self->addSubItem(baseFrameListItem);

    localFrameListItem = new CoordinateFrameListItem;
    localFrameListItem->setName(localFrameListLabel);
    self->addSubItem(localFrameListItem);

    frameSetPair = new CoordinateFrameSetPair(
        baseFrameListItem->frames(), localFrameListItem->frames());
};


CoordinateFrameListPairItem::CoordinateFrameListPairItem(const CoordinateFrameListPairItem& org)
    : Item(org)
{
    impl = new Impl(this, *org.impl);
}


CoordinateFrameListPairItem::Impl::Impl(CoordinateFrameListPairItem* self, const Impl& org)
    : self(self)
{
    baseFrameListItem = new CoordinateFrameListItem(*org.baseFrameListItem);
    self->addSubItem(baseFrameListItem);

    localFrameListItem = new CoordinateFrameListItem(*org.localFrameListItem);
    self->addSubItem(localFrameListItem);
    
    frameSetPair = new CoordinateFrameSetPair(
        baseFrameListItem->frames(), localFrameListItem->frames());
}


CoordinateFrameListPairItem::~CoordinateFrameListPairItem()
{
    delete impl;
}


Item* CoordinateFrameListPairItem::doDuplicate() const
{
    return new CoordinateFrameListPairItem(*this);
}


CoordinateFrameSetPair* CoordinateFrameListPairItem::frameSetPair()
{
    return impl->frameSetPair;
}


const CoordinateFrameSetPair* CoordinateFrameListPairItem::frameSetPair() const
{
    return impl->frameSetPair;
}


CoordinateFrameListItem* CoordinateFrameListPairItem::baseFrameListItem()
{
    return impl->baseFrameListItem;
}


const CoordinateFrameListItem* CoordinateFrameListPairItem::baseFrameListItem() const
{
    return impl->baseFrameListItem;
}


CoordinateFrameListItem* CoordinateFrameListPairItem::localFrameListItem()
{
    return impl->localFrameListItem;
}


const CoordinateFrameListItem* CoordinateFrameListPairItem::localFrameListItem() const
{
    return impl->localFrameListItem;
}


CoordinateFrameList* CoordinateFrameListPairItem::baseFrames()
{
    return impl->baseFrameListItem->frames();
}


const CoordinateFrameList* CoordinateFrameListPairItem::baseFrames() const
{
    return impl->baseFrameListItem->frames();
}


CoordinateFrameList* CoordinateFrameListPairItem::localFrames()
{
    return impl->localFrameListItem->frames();
}


const CoordinateFrameList* CoordinateFrameListPairItem::localFrames() const
{
    return impl->localFrameListItem->frames();
}


bool CoordinateFrameListPairItem::store(Archive& archive)
{
    if(!impl->baseFrameListItem->store(*archive.openSubArchive("baseFrames"))){
        return false;
    }
    if(!impl->localFrameListItem->store(*archive.openSubArchive("localFrames"))){
        return false;
    }
    return true;
}


bool CoordinateFrameListPairItem::restore(const Archive& archive)
{
    if(!impl->baseFrameListItem->restore(*archive.findSubArchive("baseFrames"))){
        return false;
    }
    if(!impl->localFrameListItem->restore(*archive.findSubArchive("localFrames"))){
        return false;
    }
    return true;
}
