#include "CoordinateFrameListItem.h"
#include "CoordinateFrameItem.h"
#include "CoordinateFrameMarker.h"
#include "ItemManager.h"
#include "PutPropertyFunction.h"
#include "Archive.h"
#include "MessageView.h"
#include <cnoid/CoordinateFrameList>
#include <cnoid/ConnectionSet>
#include <fmt/format.h>
#include <unordered_map>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

Signal<void(CoordinateFrameListItem* frameListItem)> sigInstanceAddedOrUpdated_;

class FrameMarker : public CoordinateFrameMarker
{
public:
    CoordinateFrameListItem::Impl* impl;
    bool isGlobal;
    bool isOn;
    int transientHolderCounter;
    
    FrameMarker(CoordinateFrameListItem::Impl* impl, CoordinateFrame* frame);
    virtual void onFrameUpdated(int flags) override;
};

typedef ref_ptr<FrameMarker> FrameMarkerPtr;

}

namespace cnoid {

class CoordinateFrameListItem::Impl
{
public:
    CoordinateFrameListItem* self;
    CoordinateFrameListPtr frameList;
    int itemizationMode;
    ScopedConnectionSet frameListConnections;
    std::function<std::string(const CoordinateFrameItem* item)> frameItemDisplayNameFunction;
    bool isUpdatingFrameItems;

    SgGroupPtr frameMarkerGroup;
    SgPosTransformPtr relativeFrameMarkerGroup;
    unordered_map<CoordinateFramePtr, FrameMarkerPtr> visibleFrameMarkerMap;
    SgUpdate sgUpdate;
    ScopedConnection parentLocationConnection;
    Signal<void(int index, bool on)> sigFrameMarkerVisibilityChanged;

    class TransientFrameMarkerHolder : public Referenced
    {
    public:
        weak_ref_ptr<CoordinateFrameListItem> weakFrameListItem;
        CoordinateFramePtr frame;
        TransientFrameMarkerHolder(CoordinateFrameListItem* frameListItem, CoordinateFrame* frame)
            : weakFrameListItem(frameListItem), frame(frame) { }
        ~TransientFrameMarkerHolder(){
            if(auto item = weakFrameListItem.lock()){
                item->impl->setFrameMarkerVisible(frame, false, true);
            }
        }
    };

    Impl(CoordinateFrameListItem* self, CoordinateFrameList* frameList, int itemizationMode);
    void setItemizationMode(int mode);
    void updateFrameItems();
    void arrangeFrameItems();
    CoordinateFrameItem* createFrameItem(CoordinateFrame* frame);
    CoordinateFrameItem* findFrameItemAt(int index, Item*& out_insertionPosition);
    CoordinateFrameItem* findFrameItemAt(int index);
    void onFrameAdded(int index);
    void onFrameRemoved(int index);
    bool onFrameItemAdded(CoordinateFrameItem* frameItem);
    void setFrameMarkerVisible(CoordinateFrame* frame, bool on, bool isTransient);
    void updateParentFrameForFrameMarkers(const Position& T);
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
    impl = new Impl(this, new CoordinateFrameList, NoItemization);
}


CoordinateFrameListItem::CoordinateFrameListItem(CoordinateFrameList* frameList)
{
    impl = new Impl(this, frameList, NoItemization);
}


CoordinateFrameListItem::CoordinateFrameListItem(const CoordinateFrameListItem& org)
    : Item(org)
{
    auto frameList = new CoordinateFrameList(*org.impl->frameList);
    impl = new Impl(this, frameList, org.impl->itemizationMode);
}


CoordinateFrameListItem::Impl::Impl
(CoordinateFrameListItem* self, CoordinateFrameList* frameList, int itemizationMode)
    : self(self),
      frameList(frameList),
      itemizationMode(itemizationMode)
{
    frameList->setFirstElementAsDefaultFrame();
    isUpdatingFrameItems = false;    

    frameMarkerGroup = new SgGroup;
    relativeFrameMarkerGroup = new SgPosTransform;
    frameMarkerGroup->addChild(relativeFrameMarkerGroup);
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


bool CoordinateFrameListItem::isNoItemizationMode() const
{
    return impl->itemizationMode == NoItemization;
}


void CoordinateFrameListItem::setItemizationMode(int mode)
{
    impl->setItemizationMode(mode);
}


void CoordinateFrameListItem::Impl::setItemizationMode(int mode)
{
    if(mode != itemizationMode){
        itemizationMode = mode;
        frameListConnections.disconnect();
        if(mode != NoItemization){
            frameListConnections.add(
                frameList->sigFrameAdded().connect(
                    [&](int index){ onFrameAdded(index); }));
            frameListConnections.add(
                frameList->sigFrameRemoved().connect(
                    [&](int index, CoordinateFrame*){ onFrameRemoved(index); }));
        }
        updateFrameItems();
    }
}


void CoordinateFrameListItem::customizeFrameItemDisplayName
(std::function<std::string(const CoordinateFrameItem* frame)> func)
{
    impl->frameItemDisplayNameFunction = func;
    for(auto& frameItem : descendantItems<CoordinateFrameItem>()){
        frameItem->notifyNameChange();
    }
}


std::string CoordinateFrameListItem::getFrameItemDisplayName(const CoordinateFrameItem* item) const
{
    if(impl->frameItemDisplayNameFunction){
        return impl->frameItemDisplayNameFunction(item);
    }
    return item->name();
}


void CoordinateFrameListItem::updateFrameItems()
{
    impl->updateFrameItems();
}


void CoordinateFrameListItem::Impl::updateFrameItems()
{
    isUpdatingFrameItems = true;
    
    // clear existing frame items
    for(auto& item : self->childItems<CoordinateFrameItem>()){
        item->removeFromParentItem();
    }

    if(itemizationMode != NoItemization){
        const int numFrames = frameList->numFrames();
        for(int i=0; i < numFrames; ++i){
            self->addChildItem(createFrameItem(frameList->frameAt(i)));
        }
    }

    isUpdatingFrameItems = false;
}


/**
   Currently this function only adds the default frame item if it does not exist.
   This function should be improved so that any other lacking frame items can be added,
   and extra frame items can be removed, and the order of the items can be corrected.
*/
void CoordinateFrameListItem::Impl::arrangeFrameItems()
{
    if(frameList->hasFirstElementAsDefaultFrame()){
        auto firstFrameItem = findFrameItemAt(0);
        if(!firstFrameItem || !frameList->isDefaultFrameId(firstFrameItem->frame()->id())){
            self->insertChild(self->childItem(), createFrameItem(frameList->frameAt(0)));
        }
    }
}


CoordinateFrameItem* CoordinateFrameListItem::Impl::createFrameItem(CoordinateFrame* frame)
{
    CoordinateFrameItem* item = new CoordinateFrameItem(frame);
    if(itemizationMode == SubItemization || frameList->isDefaultFrameId(frame->id())){
        item->setSubItemAttributes();
    } else if(itemizationMode == IndependentItemization){
        item->setAttribute(Item::Attached);
    }
    return item;
}
    

CoordinateFrameItem* CoordinateFrameListItem::Impl::findFrameItemAt
(int index, Item*& out_insertionPosition)
{
    CoordinateFrameItem* frameItem = nullptr;
    int childIndex = 0;
    Item* childItem = self->childItem();
    while(childItem){
        frameItem = dynamic_cast<CoordinateFrameItem*>(childItem);
        if(frameItem){
            if(childIndex == index){
                break;
            }
            ++childIndex;
        }
        childItem = childItem->nextItem();
    }
    out_insertionPosition = childItem;
    return frameItem;
}


CoordinateFrameItem* CoordinateFrameListItem::Impl::findFrameItemAt(int index)
{
    Item* position;
    return findFrameItemAt(index, position);
}


CoordinateFrameItem* CoordinateFrameListItem::findFrameItemAt(int index)
{
    return impl->findFrameItemAt(index);
}
    
    
void CoordinateFrameListItem::Impl::onFrameAdded(int index)
{
    auto frame = frameList->frameAt(index);
    auto item = createFrameItem(frame);
    Item* position;
    findFrameItemAt(index, position);
    isUpdatingFrameItems = true;
    self->insertChild(position, item);
    isUpdatingFrameItems = false;
}


void CoordinateFrameListItem::Impl::onFrameRemoved(int index)
{
    if(auto frameItem = findFrameItemAt(index)){
        isUpdatingFrameItems = true;
        frameItem->removeFromParentItem();
        isUpdatingFrameItems = false;
    }
}


bool CoordinateFrameListItem::onChildItemAboutToBeAdded(Item* childItem, bool isManualOperation)
{
    if(impl->isUpdatingFrameItems){
        return true;
    }
    if(isNoItemizationMode()){
        return false;
    }
    if(auto frameItem = dynamic_cast<CoordinateFrameItem*>(childItem)){
        // check the existing frame with the same id
        auto frame = frameItem->frame();
        if(!impl->frameList->findFrame(frame->id())){
            return true;
        } else if(isManualOperation){
            showWarningDialog(
                format(_("\"{0}\" cannot be added to \"{1}\" because "
                         "the item of ID {2} already exists in it. "),
                       getFrameItemDisplayName(frameItem), displayName(), frame->id().label()));
        }
    }
    return false;
}


bool CoordinateFrameListItem::onFrameItemAdded(CoordinateFrameItem* frameItem)
{
    return impl->onFrameItemAdded(frameItem);
}


bool CoordinateFrameListItem::Impl::onFrameItemAdded(CoordinateFrameItem* frameItem)
{
    if(isUpdatingFrameItems || self->isNoItemizationMode()){
        return true;
    }
    
    bool result = false;
    
    frameListConnections.block();

    auto frame = frameItem->frame();
    if(!frameItem->nextItem()){
        result = frameList->append(frame);
    } else {
        int index = 0;
        Item* item = self->childItem();
        while(item){
            if(item == frameItem){
                break;
            }
            item = item->nextItem();
            ++index;
        }
        if(item){
            result = frameList->insert(index, frame);
        }
    }

    frameListConnections.unblock();

    return result;
}


void CoordinateFrameListItem::onFrameItemRemoved(CoordinateFrameItem* frameItem)
{
    if(!impl->isUpdatingFrameItems){
        if(!isNoItemizationMode()){
            impl->frameListConnections.block();
            impl->frameList->remove(frameItem->frame());
            impl->frameListConnections.unblock();
        }
    }
}


CoordinateFrameItem* CoordinateFrameListItem::findFrameItem(const GeneralId& id)
{
    for(auto item = childItem(); item; item = item->nextItem()){
        if(auto frameItem = dynamic_cast<CoordinateFrameItem*>(item)){
            if(frameItem->frame()->id() == id){
                return frameItem;
            }
        }
    }
    return nullptr;
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


LocationProxyPtr CoordinateFrameListItem::getFrameParentLocationProxy()
{
    if(isForBaseFrames()){
        if(auto locatableItem = findOwnerItem<LocatableItem>()){
            return locatableItem->getLocationProxy();
        }
    }
    return nullptr;
}


bool CoordinateFrameListItem::getRelativeFramePosition(const CoordinateFrame* frame, Position& out_T) const
{
    if(frame->isGlobal()){
        if(auto parentLocation = const_cast<CoordinateFrameListItem*>(this)->getFrameParentLocationProxy()){
            auto T_base = parentLocation->getLocation();
            out_T = T_base.inverse(Eigen::Isometry) * frame->T();
            return true;
        }
        return false;
    }
    out_T = frame->T();
    return true;
}


bool CoordinateFrameListItem::getGlobalFramePosition(const CoordinateFrame* frame, Position& out_T) const
{
    if(!frame->isGlobal()){
        if(auto parentLocation = const_cast<CoordinateFrameListItem*>(this)->getFrameParentLocationProxy()){
            auto T_base = parentLocation->getLocation();
            out_T = T_base * frame->T();
            return true;
        }
        return false;
    }
    out_T = frame->T();
    return true;
}
        

bool CoordinateFrameListItem::switchFrameMode(CoordinateFrame* frame, int mode)
{
    if(frame->mode() == mode){
        return true;
    }
    auto parentLocation = getFrameParentLocationProxy();
    if(!parentLocation){
        return false;
    }
    auto T_base = parentLocation->getLocation();
    if(mode == CoordinateFrame::Global){
        frame->setPosition(T_base * frame->T());
    } else { // Local
        frame->setPosition(T_base.inverse(Eigen::Isometry) * frame->T());
    }
    frame->setMode(mode);
    return true;
}


SgNode* CoordinateFrameListItem::getScene()
{
    return impl->frameMarkerGroup;
}


void CoordinateFrameListItem::setFrameMarkerVisible(const CoordinateFrame* frame, bool on)
{
    impl->setFrameMarkerVisible(const_cast<CoordinateFrame*>(frame), on, false);
}


void CoordinateFrameListItem::Impl::setFrameMarkerVisible(CoordinateFrame* frame, bool on, bool isTransient)
{
    bool changed = false;
    bool relativeMarkerChanged = false;
    FrameMarker* marker = nullptr;
    auto p = visibleFrameMarkerMap.find(frame);
    if(p != visibleFrameMarkerMap.end()){
        marker = p->second;
        if(!isTransient && !marker->isOn){
            changed = true;
        }
    } else if(on){
        marker = new FrameMarker(this, frame);
        marker->isOn = false;
        marker->transientHolderCounter = 0;
        visibleFrameMarkerMap[frame] = marker;
        if(marker->isGlobal){
            frameMarkerGroup->addChild(marker, true);
        } else {
            relativeFrameMarkerGroup->addChild(marker, true);
            relativeMarkerChanged = true;
        }
        changed = true;
    }
    if(on){
        if(!isTransient){
            marker->isOn = true;
        } else {
            marker->transientHolderCounter += 1;
        }
    } else if(marker){
        if(!isTransient){
            if(marker->isOn){
                marker->isOn = false;
                changed = true;
            }
        } else {
            marker->transientHolderCounter -= 1;
        }
        if(!marker->isOn && marker->transientHolderCounter <= 0){
            if(marker->isGlobal){
                frameMarkerGroup->removeChild(marker, true);
            } else {
                relativeFrameMarkerGroup->removeChild(marker, true);
                relativeMarkerChanged = true;
            }
            visibleFrameMarkerMap.erase(p);
            changed = true;
        }
    }

    if(changed){
        int numRelativeMarkers = relativeFrameMarkerGroup->numChildren();
        if(relativeMarkerChanged){
            if(parentLocationConnection.connected()){
                if(relativeFrameMarkerGroup->empty()){
                    parentLocationConnection.disconnect();
                }
            } else {
                if(!relativeFrameMarkerGroup->empty()){
                    if(auto location = self->getFrameParentLocationProxy()){
                        parentLocationConnection =
                            location->sigLocationChanged().connect(
                                [this, location](){
                                    updateParentFrameForFrameMarkers(location->getLocation());
                                });
                        updateParentFrameForFrameMarkers(location->getLocation());
                    }
                }
            }
        }

        if(on){
            self->setChecked(true);
        } else {
            if(numRelativeMarkers == 0 && frameMarkerGroup->numChildren() <= 1){
                self->setChecked(false);
            }
        }
        if(!isTransient){
            int frameIndex = -1;
            if(!sigFrameMarkerVisibilityChanged.empty()){
                frameIndex = frameList->indexOf(frame);
                sigFrameMarkerVisibilityChanged(frameIndex, on);
            }
            if(itemizationMode != NoItemization){
                if(frameIndex < 0){
                    frameIndex = frameList->indexOf(frame);
                }
                if(auto frameItem = findFrameItemAt(frameIndex)){
                    frameItem->setVisibilityCheck(on);
                }
            }
        }
    }
}


ReferencedPtr CoordinateFrameListItem::transientFrameMarkerHolder(const CoordinateFrame* frame)
{
    auto frame_ = const_cast<CoordinateFrame*>(frame);
    impl->setFrameMarkerVisible(frame_, true, true);
    return new Impl::TransientFrameMarkerHolder(this, frame_);
}


bool CoordinateFrameListItem::isFrameMarkerVisible(const CoordinateFrame* frame) const
{
    auto p = impl->visibleFrameMarkerMap.find(const_cast<CoordinateFrame*>(frame));
    if(p != impl->visibleFrameMarkerMap.end()){
        auto& marker = p->second;
        return marker->isOn;
    }
    return false;
}


SignalProxy<void(int index, bool on)> CoordinateFrameListItem::sigFrameMarkerVisibilityChanged()
{
    return impl->sigFrameMarkerVisibilityChanged;
}


void CoordinateFrameListItem::Impl::updateParentFrameForFrameMarkers(const Position& T)
{
    relativeFrameMarkerGroup->setPosition(T);
    relativeFrameMarkerGroup->notifyUpdate(sgUpdate);
}


FrameMarker::FrameMarker(CoordinateFrameListItem::Impl* impl, CoordinateFrame* frame)
    : CoordinateFrameMarker(frame),
      impl(impl)
{
    isGlobal = frame->isGlobal();
}


void FrameMarker::onFrameUpdated(int flags)
{
    if(flags & CoordinateFrame::ModeUpdate){
        bool isCurrentGlobal = frame()->isGlobal();
        if(isCurrentGlobal != isGlobal){
            FrameMarkerPtr holder = this;
            if(isCurrentGlobal){
                impl->relativeFrameMarkerGroup->removeChild(this, true);
                impl->frameMarkerGroup->addChild(this, true);
            } else {
                impl->frameMarkerGroup->removeChild(this, true);
                impl->relativeFrameMarkerGroup->addChild(this, true);
            }
            isGlobal = isCurrentGlobal;
        }
    }

    CoordinateFrameMarker::onFrameUpdated(flags);
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
    bool result = impl->frameList->read(archive);
    if(result){
        archive.addProcessOnSubTreeRestored(
            [&](){ impl->arrangeFrameItems(); });
    }
    return result;
}
