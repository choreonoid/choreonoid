#include "CoordinateFrameListItem.h"
#include "CoordinateFrameItem.h"
#include "ItemManager.h"
#include "LocatableItem.h"
#include "PutPropertyFunction.h"
#include "Archive.h"
#include <cnoid/CoordinateFrameList>
#include <cnoid/ConnectionSet>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

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
    std::function<void(CoordinateFrameItem* item)> newFrameItemCallback;

    Impl(CoordinateFrameListItem* self);
    Impl(CoordinateFrameListItem* self, const Impl& org);
    void setItemizationMode(int mode);
    void updateFrameItems();
    CoordinateFrameItem* createFrameItem(CoordinateFrame* frame);
    void updateFrameAttribute(CoordinateFrameItem* item, CoordinateFrame* frame);
    Item* findFrameItemAt(int index);
    void onFrameAdded(int index);
    void onFrameRemoved(int index);
    void onFrameAttributeChanged(int index);
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


void CoordinateFrameListItem::setNewFrameItemCallback(std::function<void(CoordinateFrameItem* item)> callback)
{
    impl->newFrameItemCallback = callback;
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
            frameListConnections.add(
                frameList->sigFrameAttributeChanged().connect(
                    [&](int index){ onFrameAttributeChanged(index); }));
        }
        itemizationMode = mode;
    }
}


void CoordinateFrameListItem::updateFrameItems()
{
    impl->updateFrameItems();
}


void CoordinateFrameListItem::Impl::updateFrameItems()
{
    if(itemizationMode == NoItemization){
        return;
    }

    // clear existing frame items
    for(auto& item : self->childItems<CoordinateFrameItem>()){
        item->detachFromParentItem();
    }

    const int numFrames = frameList->numFrames();
    for(int i=0; i < numFrames; ++i){
        self->addChildItem(createFrameItem(frameList->frameAt(i)));
    }
}


CoordinateFrameItem* CoordinateFrameListItem::Impl::createFrameItem(CoordinateFrame* frame)
{
    auto item = new CoordinateFrameItem;
    updateFrameAttribute(item, frame);
    if(itemizationMode == SubItemization){
        item->setSubItemAttributes();
    } else if(itemizationMode == IndependentItemization){
        item->setAttribute(Item::Attached);
    }
    if(newFrameItemCallback){
        newFrameItemCallback(item);
    }
    return item;
}
    

void CoordinateFrameListItem::Impl::updateFrameAttribute
(CoordinateFrameItem* item, CoordinateFrame* frame)
{
    item->setFrameId(frame->id());
    item->setName(format("{0}: {1}", frame->id().label(), frame->note()));
}


Item* CoordinateFrameListItem::Impl::findFrameItemAt(int index)
{
    int childIndex = 0;
    Item* childItem = self->childItem();
    while(childItem){
        if(dynamic_cast<CoordinateFrameItem*>(childItem)){
            if(childIndex == index){
                break;
            }
            ++childIndex;
        }
        childItem = childItem->nextItem();
    }
    return childItem;
}


void CoordinateFrameListItem::Impl::onFrameAdded(int index)
{
    auto frame = frameList->frameAt(index);
    auto item = createFrameItem(frame);
    self->insertChild(findFrameItemAt(index), item);
}


void CoordinateFrameListItem::Impl::onFrameRemoved(int index)
{
    if(auto frameItem = findFrameItemAt(index)){
        frameItem->detachFromParentItem();
    }
}


void CoordinateFrameListItem::Impl::onFrameAttributeChanged(int index)
{
    if(auto item = dynamic_cast<CoordinateFrameItem*>(findFrameItemAt(index))){
        updateFrameAttribute(item, frameList->frameAt(index));
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


void CoordinateFrameListItem::useAsBaseFrames()
{
    if(!impl->frameList->isForBaseFrames()){
        impl->frameList->setFrameType(CoordinateFrameList::Base);
        if(isConnectedToRoot()){
            ::sigInstanceAddedOrUpdated_(this);
        }
    }
}


void CoordinateFrameListItem::useAsOffsetFrames()
{
    if(!impl->frameList->isForOffsetFrames()){
        impl->frameList->setFrameType(CoordinateFrameList::Offset);
        if(isConnectedToRoot()){
            ::sigInstanceAddedOrUpdated_(this);
        }
    }
}


bool CoordinateFrameListItem::isForBaseFrames() const
{
    return impl->frameList->isForBaseFrames();
}


bool CoordinateFrameListItem::isForOffsetFrames() const
{
    return impl->frameList->isForOffsetFrames();
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
    impl->frameList->writeHeader(archive);
    if(impl->itemizationMode != IndependentItemization){
        impl->frameList->writeFrames(archive);
    }
    return true;
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
    impl->frameList->resetIdCounter();
    return impl->frameList->read(archive);
}
