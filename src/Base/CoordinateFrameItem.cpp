#include "CoordinateFrameItem.h"
#include "CoordinateFrameListItem.h"
#include "ItemManager.h"
#include "PutPropertyFunction.h"
#include "Archive.h"
#include <cnoid/CoordinateFrame>
#include <cnoid/CoordinateFrameList>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

class CoordinateFrameItem::Impl
{
public:
    GeneralId frameId;
    CoordinateFrameListItem* frameListItem;
    CoordinateFrameList* frameList;
    SignalProxy<void()> sigLocationChanged;
    
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
    frameListItem = nullptr;
    frameList = nullptr;
}


CoordinateFrameItem::CoordinateFrameItem(const CoordinateFrameItem& org)
    : Item(org)
{
    impl = new Impl(*org.impl);
}


CoordinateFrameItem::Impl::Impl(const Impl& org)
    : frameId(org.frameId)
{
    frameListItem = nullptr;
    frameList = nullptr;
}


CoordinateFrameItem::~CoordinateFrameItem()
{
    delete impl;
}


Item* CoordinateFrameItem::doDuplicate() const
{
    return new CoordinateFrameItem(*this);
}


void CoordinateFrameItem::onPositionChanged()
{
    impl->frameListItem = dynamic_cast<CoordinateFrameListItem*>(parentItem());
    if(impl->frameListItem){
        impl->frameList = impl->frameListItem->frameList();
    } else {
        impl->frameList = nullptr;
    }
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
    return impl->frameList;
}


const CoordinateFrameList* CoordinateFrameItem::frameList() const
{
    return impl->frameList;
}


CoordinateFrame* CoordinateFrameItem::frame()
{
    if(impl->frameList){
        return impl->frameList->findFrame(impl->frameId);
    }
    return nullptr;
}


const CoordinateFrame* CoordinateFrameItem::frame() const
{
    return const_cast<CoordinateFrameItem*>(this)->frame();
}


bool CoordinateFrameItem::isBaseFrame() const
{
    if(impl->frameList){
        return impl->frameList->isForBaseFrames();
    }
    return false;
}


bool CoordinateFrameItem::isOffsetFrame() const
{
    if(impl->frameList){
        return impl->frameList->isForOffsetFrames();
    }
    return false;
}


int CoordinateFrameItem::getLocationType() const
{
    if(impl->frameList){
        if(impl->frameList->isForBaseFrames()){
            return ParentRelativeLocation;
        } else if(impl->frameList->isForOffsetFrames()){
            return OffsetLocation;
        }
    }
    return InvalidLocation;
}


LocatableItem* CoordinateFrameItem::getParentLocatableItem()
{
    if(impl->frameListItem){
        return impl->frameListItem->getParentLocatableItem();
    }
    return nullptr;
}


std::string CoordinateFrameItem::getLocationName() const
{
    auto frame_ = frame();
    if(!impl->frameListItem || !frame_){
        return name();
    } else {
        auto listName = impl->frameListItem->name();
        auto id = frame_->id().label();
        auto note = frame_->note();
        if(auto parent = impl->frameListItem->getParentLocatableItem()){
            auto parentName = parent->getLocationName();
            if(note.empty()){
                return format("{0} {1} {2}", parentName, listName, id);
            } else {
                return format("{0} {1} {2} ( {3} )", parentName, listName, id, note);
            }
        } else {
            if(note.empty()){
                return format("{0} {1}", listName, id);
            } else {
                return format("{0} {1} ( {2} )", listName, id, note);
            }
        }
    }
}


Position CoordinateFrameItem::getLocation() const
{
    if(auto frame_ = frame()){
        return frame_->position();
    }
    return Position::Identity();
}


void CoordinateFrameItem::setLocation(const Position& T)
{
    if(auto frame_ = frame()){
        frame_->setPosition(T);
    }
}


SignalProxy<void()> CoordinateFrameItem::sigLocationChanged()
{
    return impl->sigLocationChanged;
}


void CoordinateFrameItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("ID"), impl->frameId.label());
}


bool CoordinateFrameItem::store(Archive& archive)
{
    if(auto list = frameList()){
        if(auto frame = list->findFrame(impl->frameId)){
            return frame->write(archive);
        }
    }
    return false;
}
    

bool CoordinateFrameItem::restore(const Archive& archive)
{
    auto frameListItem = dynamic_cast<CoordinateFrameListItem*>(archive.currentParentItem());
    if(frameListItem){
        CoordinateFramePtr frame = new CoordinateFrame;
        if(frame->read(archive)){
            impl->frameId = frame->id();
            if(frameListItem->frameList()->append(frame)){
                return true;
            }
        }
    }
    return false;
}

                
                
