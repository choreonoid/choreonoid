#include "CoordinateFrameSetPairItem.h"
#include "CoordinateFrameSetItem.h"
#include <cnoid/CoordinateFrameSet>
#include <cnoid/CoordinateFrameContainer>
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class CoordinateFrameSetPairItem::Impl
{
public:
    CoordinateFrameSetPairItem* self;
    CoordinateFrameSetPairPtr frameSetPair;
    CoordinateFrameSetItemPtr baseFrameSetItem;
    CoordinateFrameSetItemPtr localFrameSetItem;

    Impl(CoordinateFrameSetPairItem* self);
    Impl(CoordinateFrameSetPairItem* self, const Impl& org);
};

}


void CoordinateFrameSetPairItem::initializeClass(ExtensionManager* ext)
{
    auto& im = ext->itemManager();
    im.registerClass<CoordinateFrameSetPairItem>(N_("CoordinateFrameSetPairItem"));
    //im.addCreationPanel<CoordinateFrameSetPairItem>();
}


CoordinateFrameSetPairItem::CoordinateFrameSetPairItem()
{
    impl = new Impl(this);
}


CoordinateFrameSetPairItem::Impl::Impl(CoordinateFrameSetPairItem* self)
    : self(self)
{
    baseFrameSetItem = new CoordinateFrameSetItem;
    baseFrameSetItem->setName("Base");
    self->addSubItem(baseFrameSetItem);

    localFrameSetItem = new CoordinateFrameSetItem;
    localFrameSetItem->setName("Local");
    self->addSubItem(localFrameSetItem);

    frameSetPair = new CoordinateFrameSetPair(
        baseFrameSetItem->frames(), localFrameSetItem->frames());
};


CoordinateFrameSetPairItem::CoordinateFrameSetPairItem(const CoordinateFrameSetPairItem& org)
    : Item(org)
{
    impl = new Impl(this, *org.impl);
}


CoordinateFrameSetPairItem::Impl::Impl(CoordinateFrameSetPairItem* self, const Impl& org)
    : self(self)
{
    baseFrameSetItem = new CoordinateFrameSetItem(*org.baseFrameSetItem);
    self->addSubItem(baseFrameSetItem);

    localFrameSetItem = new CoordinateFrameSetItem(*org.localFrameSetItem);
    self->addSubItem(localFrameSetItem);
    
    frameSetPair = new CoordinateFrameSetPair(
        baseFrameSetItem->frames(), localFrameSetItem->frames());
}


CoordinateFrameSetPairItem::~CoordinateFrameSetPairItem()
{
    delete impl;
}


Item* CoordinateFrameSetPairItem::doDuplicate() const
{
    return new CoordinateFrameSetPairItem(*this);
}


CoordinateFrameSetPair* CoordinateFrameSetPairItem::frameSetPair()
{
    return impl->frameSetPair;
}


const CoordinateFrameSetPair* CoordinateFrameSetPairItem::frameSetPair() const
{
    return impl->frameSetPair;
}


CoordinateFrameSetItem* CoordinateFrameSetPairItem::baseFrameSetItem()
{
    return impl->baseFrameSetItem;
}


const CoordinateFrameSetItem* CoordinateFrameSetPairItem::baseFrameSetItem() const
{
    return impl->baseFrameSetItem;
}


CoordinateFrameSetItem* CoordinateFrameSetPairItem::localFrameSetItem()
{
    return impl->localFrameSetItem;
}


const CoordinateFrameSetItem* CoordinateFrameSetPairItem::localFrameSetItem() const
{
    return impl->localFrameSetItem;
}


CoordinateFrameContainer* CoordinateFrameSetPairItem::baseFrames()
{
    return impl->baseFrameSetItem->frames();
}


const CoordinateFrameContainer* CoordinateFrameSetPairItem::baseFrames() const
{
    return impl->baseFrameSetItem->frames();
}


CoordinateFrameContainer* CoordinateFrameSetPairItem::localFrames()
{
    return impl->localFrameSetItem->frames();
}


const CoordinateFrameContainer* CoordinateFrameSetPairItem::localFrames() const
{
    return impl->localFrameSetItem->frames();
}


bool CoordinateFrameSetPairItem::store(Archive& archive)
{
    return true;
}


bool CoordinateFrameSetPairItem::restore(const Archive& archive)
{
    return true;
}
