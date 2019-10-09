#include "CoordinateFrameSetItem.h"
#include <cnoid/CoordinateFrameSet>
#include <cnoid/CoordinateFrameContainer>
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class CoordinateFrameSetItem::Impl
{
public:
    CoordinateFrameSetItem* self;
    CoordinateFrameContainerPtr frames;

    Impl(CoordinateFrameSetItem* self);
    Impl(CoordinateFrameSetItem* self, const Impl& org);
};

}


void CoordinateFrameSetItem::initializeClass(ExtensionManager* ext)
{
    auto& im = ext->itemManager();
    im.registerClass<CoordinateFrameSetItem>(N_("CoordinateFrameSetItem"));
}


CoordinateFrameSetItem::CoordinateFrameSetItem()
{
    impl = new Impl(this);
}


CoordinateFrameSetItem::Impl::Impl(CoordinateFrameSetItem* self)
    : self(self)
{
    frames = new CoordinateFrameContainer;
}


CoordinateFrameSetItem::CoordinateFrameSetItem(const CoordinateFrameSetItem& org)
    : Item(org)
{
    impl = new Impl(this, *org.impl);
}


CoordinateFrameSetItem::Impl::Impl(CoordinateFrameSetItem* self, const Impl& org)
    : self(self)
{
    frames = new CoordinateFrameContainer(*org.frames);
}


CoordinateFrameSetItem::~CoordinateFrameSetItem()
{
    delete impl;
}


Item* CoordinateFrameSetItem::doDuplicate() const
{
    return new CoordinateFrameSetItem(*this);
}


CoordinateFrameContainer* CoordinateFrameSetItem::frames()
{
    return impl->frames;
}


const CoordinateFrameContainer* CoordinateFrameSetItem::frames() const
{
    return impl->frames;
}


SgNode* CoordinateFrameSetItem::getScene()
{
    return nullptr;
}


void CoordinateFrameSetItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Num frames"), impl->frames->numFrames());
}


bool CoordinateFrameSetItem::store(Archive& archive)
{
    return impl->frames->write(archive);
}


bool CoordinateFrameSetItem::restore(const Archive& archive)
{
    impl->frames->resetIdCounter();
    impl->frames->read(archive);
    return true;
}
