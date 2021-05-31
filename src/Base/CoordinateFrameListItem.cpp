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

class ListAssociationSignalInfo : public Referenced
{
public:
    Signal<void(CoordinateFrameListItem* frameListItem, bool on)> signal;
    ScopedConnection connection;
};
typedef ref_ptr<ListAssociationSignalInfo> ListAssociationSignalInfoPtr;

unordered_map<ItemPtr, ListAssociationSignalInfoPtr> listAssociationSignalInfoMap;

class FrameMarker : public CoordinateFrameMarker
{
public:
    CoordinateFrameListItem::Impl* impl;
    weak_ref_ptr<CoordinateFrameItem> weakFrameItem;
    ScopedConnection frameItemConnection;
    bool isGlobal;
    bool isOn;
    int transientHolderCounter;
    
    FrameMarker(CoordinateFrameListItem::Impl* impl, CoordinateFrame* frame);
    void updateFrameItem(CoordinateFrameItem* frameItem, bool on);
    void updateFrameLock();
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

    struct AssociatedItemInfo
    {
        ListAssociationSignalInfoPtr signalInfo;
        bool isAssociated;
    };
    unordered_map<weak_ref_ptr<Item>, AssociatedItemInfo> associatedItemInfoMap;
    
    bool isUpdatingFrameItems;

    SgGroupPtr frameMarkerGroup;
    SgPosTransformPtr relativeFrameMarkerGroup;
    unordered_map<CoordinateFramePtr, FrameMarkerPtr> visibleFrameMarkerMap;
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
    void notifyRegisteredItemsOfListAssociation(bool doCheckDisassociation);
    void checkListDisassociation(bool doDisassociateAll);
    void setItemizationMode(int mode);
    void updateFrameItems();
    CoordinateFrameItem* createFrameItem(CoordinateFrame* frame);
    CoordinateFrameItem* findFrameItemAt(int index, Item*& out_insertionPosition);
    CoordinateFrameItem* findFrameItemAt(int index);
    void onFrameAdded(int index);
    void onFrameRemoved(int index);
    bool onFrameItemAdded(CoordinateFrameItem* frameItem);
    void setFrameMarkerVisible(CoordinateFrame* frame, bool on, bool isTransient);
    void updateParentFrameForFrameMarkers(const Isometry3& T);
    void storeLockedFrameIndices(Archive& archive);
    void completeFrameItemRestoration(const Archive& archive);
};

}


void CoordinateFrameListItem::initializeClass(ExtensionManager* ext)
{
    auto& im = ext->itemManager();
    im.registerClass<CoordinateFrameListItem>(N_("CoordinateFrameListItem"));
    im.addCreationPanel<CoordinateFrameListItem>();
}


SignalProxy<void(CoordinateFrameListItem* frameListItem, bool on)>
CoordinateFrameListItem::sigListAssociationWith(Item* item)
{
    auto& info = listAssociationSignalInfoMap[item];
    if(!info){
        info = new ListAssociationSignalInfo;
        info->connection =
            item->sigDisconnectedFromRoot().connect(
                [item](){ listAssociationSignalInfoMap.erase(item); });
    }
    return info->signal;
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


void CoordinateFrameListItem::onTreePositionChanged()
{
    impl->notifyRegisteredItemsOfListAssociation(true);
}


void CoordinateFrameListItem::onDisconnectedFromRoot()
{
    impl->checkListDisassociation(true);
}


void CoordinateFrameListItem::Impl::notifyRegisteredItemsOfListAssociation(bool doCheckDisassociation)
{
    if(doCheckDisassociation){
        // Clear flags to check if each item is still associated
        for(auto& kv : associatedItemInfoMap){
            kv.second.isAssociated = false;
        }
    }
    if(Item* item = self->parentItem()){
        do {
            auto p = listAssociationSignalInfoMap.find(item);
            if(p != listAssociationSignalInfoMap.end()){
                weak_ref_ptr<Item> witem = item;
                auto& itemInfo = associatedItemInfoMap[witem];
                if(!itemInfo.signalInfo){
                    auto& signalInfo = p->second;
                    signalInfo->signal(self, true); // notify item of association
                    itemInfo.signalInfo = signalInfo;
                }
                itemInfo.isAssociated = true;
            }
            item = item->parentItem();
        } while(item);
    }
    if(doCheckDisassociation){
        checkListDisassociation(false);
    }
}


void CoordinateFrameListItem::Impl::checkListDisassociation(bool doDisassociateAll)
{
    auto iter = associatedItemInfoMap.begin();
    while(iter != associatedItemInfoMap.end()){
        bool doRemove = false;
        auto item = iter->first.lock();
        if(!item){
            doRemove = true;
        } else {
            auto& itemInfo = iter->second;
            if(doDisassociateAll || !itemInfo.isAssociated){
                itemInfo.signalInfo->signal(self, false); // notify item of disassociation
                doRemove = true;
            }
        }
        if(doRemove){
            iter = associatedItemInfoMap.erase(iter);
        } else {
            ++iter;
        }
    }
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
        impl->notifyRegisteredItemsOfListAssociation(false);
    }
}


void CoordinateFrameListItem::useAsOffsetFrames()
{
    if(!impl->frameList->isForOffsetFrames()){
        impl->frameList->setFrameType(CoordinateFrameList::Offset);
        impl->notifyRegisteredItemsOfListAssociation(false);
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


bool CoordinateFrameListItem::getRelativeFramePosition(const CoordinateFrame* frame, Isometry3& out_T) const
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


bool CoordinateFrameListItem::getGlobalFramePosition(const CoordinateFrame* frame, Isometry3& out_T) const
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
    SgTmpUpdate update;
    
    auto p = visibleFrameMarkerMap.find(frame);
    if(p != visibleFrameMarkerMap.end()){
        marker = p->second;
        if(!isTransient && (on != marker->isOn)){
            changed = true;
        }
    } else if(on){
        marker = new FrameMarker(this, frame);
        visibleFrameMarkerMap[frame] = marker;
        if(marker->isGlobal){
            frameMarkerGroup->addChild(marker, update);
        } else {
            relativeFrameMarkerGroup->addChild(marker, update);
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
            marker->isOn = false;
        } else {
            marker->transientHolderCounter -= 1;
        }
        if(!marker->isOn && marker->transientHolderCounter <= 0){
            if(marker->isGlobal){
                frameMarkerGroup->removeChild(marker, update);
            } else {
                relativeFrameMarkerGroup->removeChild(marker, update);
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

        int frameIndex = frameList->indexOf(frame);
        CoordinateFrameItem* frameItem = nullptr;
        if(itemizationMode != NoItemization){
            frameItem = findFrameItemAt(frameIndex);
        }
        
        if(!isTransient){
            if(!sigFrameMarkerVisibilityChanged.empty()){
                sigFrameMarkerVisibilityChanged(frameIndex, on);
            }
            if(itemizationMode != NoItemization){
                if(frameItem){
                    frameItem->setVisibilityCheck(on);
                }
            }
        }
        if(frameItem){
            marker->updateFrameItem(frameItem, on);
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


void CoordinateFrameListItem::Impl::updateParentFrameForFrameMarkers(const Isometry3& T)
{
    relativeFrameMarkerGroup->setPosition(T);
    relativeFrameMarkerGroup->notifyUpdate();
}


namespace {

FrameMarker::FrameMarker(CoordinateFrameListItem::Impl* impl, CoordinateFrame* frame)
    : CoordinateFrameMarker(frame),
      impl(impl)
{
    isGlobal = frame->isGlobal();
    isOn = false;
    transientHolderCounter = 0;
}


void FrameMarker::updateFrameItem(CoordinateFrameItem* frameItem, bool on)
{
    weakFrameItem.reset();
    frameItemConnection.disconnect();
    if(frameItem && on){
        frameItemConnection =
            frameItem->sigUpdated().connect(
                [&](){ updateFrameLock(); });
        weakFrameItem = frameItem;
    }
    updateFrameLock();
}


void FrameMarker::updateFrameLock()
{
    bool updated = false;
    if(weakFrameItem){
        if(auto item = weakFrameItem.lock()){
            setDragEnabled(item->isLocationEditable());
            updated = true;
        }
    }
    if(!updated){
        setDragEnabled(true);
    }
}
    

void FrameMarker::onFrameUpdated(int flags)
{
    if(flags & CoordinateFrame::ModeUpdate){
        bool isCurrentGlobal = frame()->isGlobal();
        if(isCurrentGlobal != isGlobal){
            FrameMarkerPtr holder = this;
            SgTmpUpdate update;
            if(isCurrentGlobal){
                impl->relativeFrameMarkerGroup->removeChild(this, update);
                impl->frameMarkerGroup->addChild(this, update);
            } else {
                impl->frameMarkerGroup->removeChild(this, update);
                impl->relativeFrameMarkerGroup->addChild(this, update);
            }
            isGlobal = isCurrentGlobal;
        }
    }

    CoordinateFrameMarker::onFrameUpdated(flags);
}

}


void CoordinateFrameListItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Num coordinate frames"), impl->frameList->numFrames());
}


bool CoordinateFrameListItem::store(Archive& archive)
{
    impl->frameList->writeHeader(archive);
    
    if(impl->itemizationMode == IndependentItemization){
        archive.write("itemization", "independent");
    } else {
        if(impl->itemizationMode == SubItemization){
            archive.write("itemization", "sub");
            impl->storeLockedFrameIndices(archive);
        }
        impl->frameList->writeFrames(archive);
    }
    
    return true;
}


/**
   \note The index 0 is usually the default frame, but the information on the default
   frame is not stored in the archive. This means the index of the first frame stored
   in the archive is 1, which is a bit confusing.
*/
void CoordinateFrameListItem::Impl::storeLockedFrameIndices(Archive& archive)
{
    ListingPtr indices = new Listing;
    int n = frameList->numFrames();
    int index = frameList->hasFirstElementAsDefaultFrame() ? 1 : 0;
    while(index < n){
        if(auto frameItem = self->findFrameItemAt(index)){
            if(!frameItem->isLocationEditable()){
                indices->append(index);
            }
        }
        ++index;
    }
    if(!indices->empty()){
        indices->setFlowStyle();
        archive.insert("locked_frame_indices", indices);
    }
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
        if(impl->itemizationMode != NoItemization){
            archive.addProcessOnSubTreeRestored(
                [&](){ impl->completeFrameItemRestoration(archive); });
        }
    }
    return result;
}


void CoordinateFrameListItem::Impl::completeFrameItemRestoration(const Archive& archive)
{
    if(frameList->hasFirstElementAsDefaultFrame()){
        auto firstFrameItem = findFrameItemAt(0);
        if(!firstFrameItem || !frameList->isDefaultFrameId(firstFrameItem->frame()->id())){
            self->insertChild(self->childItem(), createFrameItem(frameList->frameAt(0)));
        }
    }

    if(itemizationMode == SubItemization){
        auto& locked = *archive.findListing("locked_frame_indices");
        if(locked.isValid()){
            for(int i=0; i < locked.size(); ++i){
                int index = locked[i].toInt();
                if(auto frameItem = self->findFrameItemAt(index)){
                    frameItem->setLocationEditable(false);
                }
            }
        }
    }
}
