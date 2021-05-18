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

namespace {

class FrameLocation : public LocationProxy
{
public:
    CoordinateFrameItem::Impl* impl;

    FrameLocation(CoordinateFrameItem::Impl* impl);
    void updateLocationType();
    virtual Item* getCorrespondingItem() override;
    virtual LocationProxyPtr getParentLocationProxy() const override;
    virtual std::string getName() const override;
    virtual Isometry3 getLocation() const override;
    virtual bool isEditable() const override;
    virtual void setEditable(bool on) override;
    virtual bool setLocation(const Isometry3& T) override;
    virtual SignalProxy<void()> sigLocationChanged() override;
};

}

namespace cnoid {

class CoordinateFrameItem::Impl
{
public:
    CoordinateFrameItem* self;
    CoordinateFramePtr frame;
    CoordinateFrameListItem* frameListItem;
    CoordinateFrameList* frameList;
    ScopedConnection frameConnection;
    ref_ptr<FrameLocation> frameLocation;
    Signal<void()> sigLocationChanged;
    bool isLocationEditable;
    bool isChangingCheckStatePassively;
    
    Impl(CoordinateFrameItem* self, CoordinateFrame* frame);
    void onFrameUpdated(int flags);
    bool resetFrameId(const GeneralId& id);
    void onCheckToggled(bool on);
    std::string getLocationName() const;
    void putFrameAttributes(PutPropertyFunction& putProperty);
    bool isDefaultFrame() const;
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
    impl->isLocationEditable = org.impl->isLocationEditable;
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

    isLocationEditable = true;
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
        const auto& oldName = self->name();
        const auto label = frame->id().label();
        if(label != oldName){
            self->setName(label);
            nameChanged = true;
        }
    }
    if(!nameChanged && (flags & CoordinateFrame::NoteUpdate)){
        if(frameLocation){
            frameLocation->notifyAttributeChange();
        }
    }
    if(flags & CoordinateFrame::PositionUpdate){
        sigLocationChanged();
    }
    self->notifyUpdate();
}


void CoordinateFrameItem::onAddedToParent()
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


void CoordinateFrameItem::onRemovedFromParent(Item* /* parentItem */, bool isParentBeingDeleted)
{
    if(!isParentBeingDeleted && impl->frameListItem){
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
    if(impl->frame->write(archive)){
        archive.write("locked", !isLocationEditable());
        return true;
    }
    return false;
}
    

bool CoordinateFrameItem::restore(const Archive& archive)
{
    if(impl->frame->read(archive)){
        setName(impl->frame->id().label());
        setLocationEditable(!archive.get("locked", false));
        return true;
    }
    return false;
}


LocationProxyPtr CoordinateFrameItem::getLocationProxy()
{
    if(!impl->frameLocation){
        impl->frameLocation = new FrameLocation(impl);
    }
    impl->frameLocation->updateLocationType();
    return impl->frameLocation;
}


bool CoordinateFrameItem::Impl::isDefaultFrame() const
{
    return frameList && frameList->isDefaultFrameId(frame->id());
}


bool CoordinateFrameItem::isLocationEditable() const
{
    if(!impl->isLocationEditable || impl->isDefaultFrame()){
        return false;
    }
    return true;
}
    

void CoordinateFrameItem::setLocationEditable(bool on)
{
    if(on != impl->isLocationEditable){
        impl->isLocationEditable = on;
        if(impl->frameLocation){
            impl->frameLocation->notifyAttributeChange();
        }
        notifyUpdate();
    }
}


namespace {

FrameLocation::FrameLocation(CoordinateFrameItem::Impl* impl)
    : LocationProxy(InvalidLocation),
      impl(impl)
{
    impl->self->sigNameChanged().connect(
        [&](const std::string& /* oldName */){ notifyAttributeChange(); });
}


void FrameLocation::updateLocationType()
{
    if(impl->frameList){
        if(impl->frameList->isForBaseFrames()){
            setLocationType(ParentRelativeLocation);
        } else if(impl->frameList->isForOffsetFrames()){
            setLocationType(OffsetLocation);
        }
    } else {
        setLocationType(InvalidLocation);
    }
}


Item* FrameLocation::getCorrespondingItem()
{
    return impl->self;
}


LocationProxyPtr FrameLocation::getParentLocationProxy() const
{
    if(impl->frameListItem){
        return impl->frameListItem->getFrameParentLocationProxy();
    }
    return nullptr;
}

    
std::string FrameLocation::getName() const
{
    if(!impl->frameListItem){
        return impl->self->displayName();
        
    } else {
        auto listName = impl->frameListItem->displayName();
        auto id = impl->frame->id().label();
        auto note = impl->frame->note();
        if(auto parent = const_cast<FrameLocation*>(this)->getParentLocationProxy()){
            auto parentName = parent->getName();
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


Isometry3 FrameLocation::getLocation() const
{
    return impl->frame->position();
}


bool FrameLocation::isEditable() const
{
    return impl->self->isLocationEditable();
}


void FrameLocation::setEditable(bool on)
{
    impl->self->setLocationEditable(on);
}


bool FrameLocation::setLocation(const Isometry3& T)
{
    impl->frame->setPosition(T);
    impl->frame->notifyUpdate(CoordinateFrame::PositionUpdate);
    return true;
}


SignalProxy<void()> FrameLocation::sigLocationChanged()
{
    return impl->sigLocationChanged;
}

}
