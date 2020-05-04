#include "CoordinateFrameListItem.h"
#include "CoordinateFrameItem.h"
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
    CoordinateFrameListItem* self;
    CoordinateFrameListPtr frameList;
    int itemizationMode;
    ScopedConnectionSet frameListConnections;

    Impl(CoordinateFrameListItem* self);
    Impl(CoordinateFrameListItem* self, const Impl& org);
    void setItemizationMode(int mode);
    Item* findFrameItemAt(int index);
    void onFrameAdded(int index);
    void onFrameRemoved(int index);
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
    impl = new Impl(this);
}


CoordinateFrameListItem::Impl::Impl(CoordinateFrameListItem* self)
    : self(self)
{
    frameList = new CoordinateFrameList;
    itemizationMode = NoItemization;
}


CoordinateFrameListItem::CoordinateFrameListItem(const CoordinateFrameListItem& org)
    : Item(org)
{
    impl = new Impl(this, *org.impl);
}


CoordinateFrameListItem::Impl::Impl(CoordinateFrameListItem* self, const Impl& org)
    : self(self)
{
    frameList = new CoordinateFrameList(*org.frameList);
    itemizationMode = org.itemizationMode;
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


int CoordinateFrameListItem::itemizationMode() const
{
    return impl->itemizationMode;
}


void CoordinateFrameListItem::setItemizationMode(int mode)
{
    impl->setItemizationMode(mode);
}


void CoordinateFrameListItem::Impl::setItemizationMode(int mode)
{
    if(mode != itemizationMode){
        frameListConnections.disconnect();
        if(mode == SubItemization){
            frameListConnections.add(
                frameList->sigFrameAdded().connect(
                    [&](int index){ onFrameAdded(index); }));
            frameListConnections.add(
                frameList->sigFrameRemoved().connect(
                    [&](int index, CoordinateFrame*){ onFrameRemoved(index); }));
        }
        itemizationMode = mode;
    }
}


Item* CoordinateFrameListItem::Impl::findFrameItemAt(int index)
{
    CoordinateFrameItem* frameItem = nullptr;
    int childIndex = 0;
    Item* childItem = self->childItem();
    while(childItem){
        frameItem = dynamic_cast<CoordinateFrameItem*>(childItem);
        if(!frameItem){
            continue;
        }
        if(childIndex == index){
            break;
        }
        ++childIndex;
        childItem = childItem->nextItem();
    }
    return frameItem;
}


void CoordinateFrameListItem::Impl::onFrameAdded(int index)
{
    auto frame = frameList->frameAt(index);
    auto newFrameItem = new CoordinateFrameItem;
    newFrameItem->setFrameId(frame->id());
    newFrameItem->setName(frame->id().label());
    newFrameItem->setSubItemAttributes();
    self->insertChild(findFrameItemAt(index), newFrameItem);
}


void CoordinateFrameListItem::Impl::onFrameRemoved(int index)
{
    if(auto frameItem = findFrameItemAt(index)){
        frameItem->detachFromParentItem();
    }
}    


bool CoordinateFrameListItem::onChildItemAboutToBeAdded(Item* childItem, bool isManualOperation)
{
    return true;
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
    if(impl->itemizationMode == SubItemization){
        archive.write("itemization", "sub");
    } else if(impl->itemizationMode == IndependentItemization){
        archive.write("itemization", "independent");
    }
    bool stored = true;
    if(impl->itemizationMode != IndependentItemization){
        stored = impl->frameList->write(archive);
    }
    return stored;
}


bool CoordinateFrameListItem::restore(const Archive& archive)
{
    string mode;
    if(archive.read("itemization", mode)){
        if(mode == "sub"){
            setItemizationMode(SubItemization);
        } else if(mode == "independent"){
            setItemizationMode(IndependentItemization);
        }
    }
    if(impl->itemizationMode != IndependentItemization){
        impl->frameList->resetIdCounter();
        impl->frameList->read(archive);
    }
    return true;
}
