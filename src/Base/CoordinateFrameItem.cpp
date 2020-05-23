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
    CoordinateFrameItem* self;
    CoordinateFramePtr frame;
    CoordinateFrameListItem* frameListItem;
    CoordinateFrameList* frameList;
    ScopedConnection frameConnection;
    Signal<void()> sigLocationChanged;
    bool isChangingCheckStatePassively;
    
    Impl(CoordinateFrameItem* self, CoordinateFrame* frame);
    void onFrameUpdated(int flags);
    bool resetFrameId(const GeneralId& id);
    void onCheckToggled(bool on);
    std::string getLocationName() const;
    void putFrameAttributes(PutPropertyFunction& putProperty);
};

}


void CoordinateFrameItem::initializeClass(ExtensionManager* ext)
{
    auto& im = ext->itemManager();
    im.registerClass<CoordinateFrameItem>(N_("CoordinateFrameItem"));
}


CoordinateFrameItem::CoordinateFrameItem()
{
    impl = new Impl(this, new CoordinateFrame);
}


CoordinateFrameItem::CoordinateFrameItem(CoordinateFrame* frame)
{
    impl = new Impl(this, frame);
}


CoordinateFrameItem::CoordinateFrameItem(const CoordinateFrameItem& org)
    : Item(org)
{
    impl = new Impl(this, new CoordinateFrame(*org.impl->frame));
}


CoordinateFrameItem::Impl::Impl(CoordinateFrameItem* self, CoordinateFrame* frame)
    : self(self),
      frame(frame)
{
    self->setName(frame->id().label());
    
    frameListItem = nullptr;
    frameList = nullptr;

    frameConnection = frame->sigUpdated().connect(
        [&](int flags){ onFrameUpdated(flags); });

    self->sigCheckToggled().connect(
        [this](bool on){ onCheckToggled(on); });

    self->sigNameChanged().connect(
        [self](const std::string& /* oldName */){
            self->notifyLocationAttributeChange();
        });

    isChangingCheckStatePassively = false;
}


CoordinateFrameItem::~CoordinateFrameItem()
{
    delete impl;
}


Item* CoordinateFrameItem::doDuplicate() const
{
    return new CoordinateFrameItem(*this);
}


std::string CoordinateFrameItem::displayName() const
{
    if(impl->frameListItem){
        return impl->frameListItem->getFrameItemDisplayName(this);
    }
    return name();
}


void CoordinateFrameItem::Impl::onFrameUpdated(int flags)
{
    bool nameChanged = false;
    if(flags & CoordinateFrame::IdUpdate){
        nameChanged = self->setName(frame->id().label());
    }
    if(!nameChanged && (flags & CoordinateFrame::NoteUpdate)){
        self->notifyNameChange();
    }
    if(flags & CoordinateFrame::PositionUpdate){
        sigLocationChanged();
    }
    self->notifyUpdate();
}


void CoordinateFrameItem::onAttachedToParent()
{
    if(impl->frameListItem){
        impl->frameListItem->onFrameItemRemoved(this);
    }
    impl->frameListItem = nullptr;
    impl->frameList = nullptr;

    auto newListItem = dynamic_cast<CoordinateFrameListItem*>(parentItem());
    if(newListItem){
        if(newListItem->onFrameItemAdded(this)){
            impl->frameListItem = newListItem;
            impl->frameList = newListItem->frameList();
        }
    }
}    


void CoordinateFrameItem::onDetachedFromParent()
{
    if(impl->frameListItem){
        impl->frameListItem->onFrameItemRemoved(this);
    }
    impl->frameListItem = nullptr;
    impl->frameList = nullptr;
}    


CoordinateFrameListItem* CoordinateFrameItem::frameListItem()
{
    return impl->frameListItem;
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
    return impl->frame;
}


const CoordinateFrame* CoordinateFrameItem::frame() const
{
    return impl->frame;
}


bool CoordinateFrameItem::resetFrameId(const GeneralId& id)
{
    return impl->resetFrameId(id);
}


bool CoordinateFrameItem::Impl::resetFrameId(const GeneralId& id)
{
    if(frame->resetId(id)){
        frame->notifyUpdate(CoordinateFrame::IdUpdate);
        return true;
    }
    return false;
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


void CoordinateFrameItem::Impl::onCheckToggled(bool on)
{
    if(!isChangingCheckStatePassively && frameListItem){
        frameListItem->setFrameMarkerVisible(frame, on);
    }
}


void CoordinateFrameItem::setVisibilityCheck(bool on)
{
    impl->isChangingCheckStatePassively = true;
    setChecked(on);
    impl->isChangingCheckStatePassively = false;
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
    return impl->getLocationName();
}


std::string CoordinateFrameItem::Impl::getLocationName() const
{
    if(!frameListItem){
        return self->displayName();
        
    } else {
        auto listName = frameListItem->displayName();
        auto id = frame->id().label();
        auto note = frame->note();
        if(auto parent = frameListItem->getParentLocatableItem()){
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
    return impl->frame->position();
}


bool CoordinateFrameItem::isLocationEditable() const
{
    if(impl->frameList && impl->frameList->isDefaultFrameId(impl->frame->id())){
        return false;
    }
    return LocatableItem::isLocationEditable();
}


void CoordinateFrameItem::setLocation(const Position& T)
{
    impl->frame->setPosition(T);
    impl->frame->notifyUpdate(CoordinateFrame::PositionUpdate);
}


SignalProxy<void()> CoordinateFrameItem::sigLocationChanged()
{
    return impl->sigLocationChanged;
}


void CoordinateFrameItem::doPutProperties(PutPropertyFunction& putProperty)
{
    impl->putFrameAttributes(putProperty);
}


void CoordinateFrameItem::putFrameAttributes(PutPropertyFunction& putProperty)
{
    impl->putFrameAttributes(putProperty);
}


void CoordinateFrameItem::Impl::putFrameAttributes(PutPropertyFunction& putProperty)
{
    const auto& id = frame->id();
    bool isDefaultFrame = false;
    Selection mode = { _("Local"), _("Global") };
    bool isModeEditable = false;
    if(frameList){
        isDefaultFrame = frameList->isDefaultFrameId(id);
        mode.select(frame->isLocal() ? 0 : 1);
        if(frameList->isForBaseFrames() && !isDefaultFrame){
            isModeEditable = true;
        }
    }
    if(isDefaultFrame){ // Read only
        putProperty(_("ID"), id.label());
        if(frame){
            putProperty(_("Mode"), mode);
            putProperty(_("Note"), frame->note());
        }
    } else {
        if(id.isInt()){
            putProperty(_("ID"), id.toInt(),
                        [&](int value){ return resetFrameId(value); });
        } else if(id.isString()){
            putProperty(_("ID"), id.toString(),
                        [&](const string& value){ return resetFrameId(value); });
        }
        if(frame){
            if(!isModeEditable){
                putProperty(_("Mode"), mode);
            } else {
                putProperty(
                    _("Mode"), mode,
                    [this](int index){
                        int mode = index == 0 ? CoordinateFrame::Local : CoordinateFrame::Global;
                        frameListItem->switchFrameMode(frame, mode);
                        frame->notifyUpdate(CoordinateFrame::ModeUpdate | CoordinateFrame::PositionUpdate);
                        return true;
                    });
            }
            putProperty(_("Note"), frame->note(),
                        [this](const string& text){
                            frame->setNote(text);
                            frame->notifyUpdate(CoordinateFrame::NoteUpdate);
                            return true;
                        });
        }
    }
}


bool CoordinateFrameItem::store(Archive& archive)
{
    return impl->frame->write(archive);
}
    

bool CoordinateFrameItem::restore(const Archive& archive)
{
    if(impl->frame->read(archive)){
        setName(impl->frame->id().label());
        return true;
    }
    return false;
}
