#include "PositionTagGroupItem.h"
#include "ItemManager.h"
#include "SceneWidgetEditable.h"
#include "LazyCaller.h"
#include "Archive.h"
#include <cnoid/PositionTagGroup>
#include <cnoid/SceneDrawables>
#include <cnoid/ConnectionSet>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

class ScenePositionTagGroup : public SgPosTransform, public SceneWidgetEditable
{
public:
    PositionTagGroupItem::Impl* itemImpl;
    SgPosTransformPtr offsetTransform;
    SgUpdate update;
    ScopedConnectionSet tagGroupConnections;
    
    ScenePositionTagGroup(PositionTagGroupItem::Impl* itemImpl);
    void finalize();
    void addTagNode(int index, bool doNotify);
    static SgNode* getOrCreateTagMarker();
    void removeTagNode(int index);
    void updateTagNodePosition(int index);
    void setOffsetPosition(const Position& T);
    void setParentPosition(const Position& T);
};

typedef ref_ptr<ScenePositionTagGroup> ScenePositionTagGroupPtr;


class PositionTagGroupLocation : public LocationProxy
{
public:
    PositionTagGroupItem::Impl* itemImpl;

    PositionTagGroupLocation(PositionTagGroupItem::Impl* itemImpl);
    virtual int getType() const override;
    virtual Item* getCorrespondingItem() override;
    virtual Position getLocation() const override;
    virtual void setLocation(const Position& T) override;
    virtual SignalProxy<void()> sigLocationChanged() override;
};

typedef ref_ptr<PositionTagGroupLocation> PositionTagGroupLocationPtr;

}

namespace cnoid {

class PositionTagGroupItem::Impl
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    PositionTagGroupItem* self;
    PositionTagGroupPtr tags;
    ScopedConnectionSet tagGroupConnections;
    LazyCaller notifyUpdateLater;
    PositionTagGroupLocationPtr location;
    LocationProxyPtr parentLocation;
    ScopedConnection parentLocationConnection;
    Position parentPosition;
    ScenePositionTagGroupPtr scene;
    Signal<void()> sigLocationChanged;
    
    Impl(PositionTagGroupItem* self);
    void setParentLocation(LocatableItem* parentLocatableItem);
    void convertLocalCoordinates(
        LocationProxy* currentParentLocation, LocationProxy* newParentLocation);
    void onParentLocationChanged();
};

}


void PositionTagGroupItem::initializeClass(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        auto& im = ext->itemManager();
        im.registerClass<PositionTagGroupItem>(N_("PositionTagGroupItem"));
        im.addCreationPanel<PositionTagGroupItem>();
        initialized = true;
    }
}


PositionTagGroupItem::PositionTagGroupItem()
{
    impl = new Impl(this);
}


PositionTagGroupItem::PositionTagGroupItem(const PositionTagGroupItem& org)
    : Item(org)
{
    impl = new Impl(this);
}


PositionTagGroupItem::Impl::Impl(PositionTagGroupItem* self)
    : self(self),
      notifyUpdateLater([=](){ self->notifyUpdate(); })
{
    tags = new PositionTagGroup;

    tagGroupConnections.add(
        tags->sigTagAdded().connect(
            [&](int){ notifyUpdateLater(); }));
    tagGroupConnections.add(
        tags->sigTagRemoved().connect(
            [&](int, PositionTag*){ notifyUpdateLater(); }));
    tagGroupConnections.add(
        tags->sigTagUpdated().connect(
            [&](int){ notifyUpdateLater(); }));
    tagGroupConnections.add(
        tags->sigOffsetPositionChanged().connect(
            [&](const Position&){ notifyUpdateLater(); }));

    location = new PositionTagGroupLocation(this);
    parentPosition.setIdentity();
}


PositionTagGroupItem::~PositionTagGroupItem()
{
    delete impl;
}


Item* PositionTagGroupItem::doDuplicate() const
{
    return new PositionTagGroupItem(*this);
}


const PositionTagGroup* PositionTagGroupItem::tags() const
{
    return impl->tags;
}


PositionTagGroup* PositionTagGroupItem::tags()
{
    return impl->tags;
}


SgNode* PositionTagGroupItem::getScene()
{
    if(!impl->scene){
        impl->scene = new ScenePositionTagGroup(impl);
    }
    return impl->scene;
}
    

LocationProxyPtr PositionTagGroupItem::getLocationProxy()
{
    return impl->location;
}


void PositionTagGroupItem::onPositionChanged()
{
    impl->setParentLocation(findOwnerItem<LocatableItem>());
}


void PositionTagGroupItem::Impl::setParentLocation(LocatableItem* parentLocatableItem)
{
    LocationProxyPtr newLocation;
    if(parentLocatableItem){
        newLocation = parentLocatableItem->getLocationProxy();
    }
    if(newLocation != parentLocation){
        parentLocationConnection.disconnect();
        
        convertLocalCoordinates(parentLocation, newLocation);
        
        parentLocation = newLocation;

        if(!parentLocation){
            parentPosition.setIdentity();
            tags->setOffsetPosition(Position::Identity(), true);
        } else {
            parentLocationConnection =
                parentLocation->sigLocationChanged().connect(
                    [&](){ onParentLocationChanged(); });
            onParentLocationChanged();
        }
    }
}


void PositionTagGroupItem::Impl::convertLocalCoordinates
(LocationProxy* currentParentLocation, LocationProxy* newParentLocation)
{
    Position T0;
    if(currentParentLocation){
        T0 = currentParentLocation->getLocation();
    } else {
        T0.setIdentity();
    }
    Position T1;
    if(newParentLocation){
        T1 = newParentLocation->getLocation();
    } else {
        T1.setIdentity();
    }

    Position Tc = T1.inverse(Eigen::Isometry) * T0;
    int n = tags->numTags();
    for(int i=0; i < n; ++i){
        auto tag = tags->tagAt(i);
        if(tag->hasAttitude()){
            tag->setPosition(Tc * tag->position());
        } else {
            tag->setTranslation(Tc * tag->translation());
        }
        tags->notifyTagUpdate(i);
    }
}


void PositionTagGroupItem::Impl::onParentLocationChanged()
{
    parentPosition = parentLocation->getLocation();
    if(scene){
        scene->setParentPosition(parentPosition);
    }
    sigLocationChanged();
}


const Position& PositionTagGroupItem::parentPosition() const
{
    return impl->parentPosition;
}


Position PositionTagGroupItem::globalOffsetPosition() const
{
    return impl->parentPosition * impl->tags->offsetPosition();
}


bool PositionTagGroupItem::store(Archive& archive)
{
    impl->tags->write(&archive);
    return true;
}


bool PositionTagGroupItem::restore(const Archive& archive)
{
    return impl->tags->read(&archive);
}


ScenePositionTagGroup::ScenePositionTagGroup(PositionTagGroupItem::Impl* itemImpl)
    : itemImpl(itemImpl)
{
    offsetTransform = new SgPosTransform;
    addChild(offsetTransform);
    
    auto tags = itemImpl->tags;
    int n = tags->numTags();
    for(int i=0; i < n; ++i){
        addTagNode(i, false);
    }

    tagGroupConnections.add(
        tags->sigTagAdded().connect(
            [&](int index){ addTagNode(index, true); }));
    tagGroupConnections.add(
        tags->sigTagRemoved().connect(
            [&](int index, PositionTag*){ removeTagNode(index); }));
    tagGroupConnections.add(
        tags->sigTagUpdated().connect(
            [&](int index){ updateTagNodePosition(index); }));
    tagGroupConnections.add(
        tags->sigOffsetPositionChanged().connect(
            [&](const Position& T){ setOffsetPosition(T); }));
}


void ScenePositionTagGroup::finalize()
{
    tagGroupConnections.disconnect();
    itemImpl = nullptr;
}


void ScenePositionTagGroup::addTagNode(int index, bool doNotify)
{
    auto tag = itemImpl->tags->tagAt(index);
    auto node = new SgPosTransform;
    node->addChild(getOrCreateTagMarker());
    node->setPosition(tag->position());
    offsetTransform->insertChild(index, node);
    if(doNotify){
        update.resetAction(SgUpdate::ADDED);
        offsetTransform->notifyUpdate(update);
    }
}
    

SgNode* ScenePositionTagGroup::getOrCreateTagMarker()
{
    static SgNodePtr marker;

    if(!marker){
        auto lines = new SgLineSet;

        auto& vertices = *lines->getOrCreateVertices(4);
        constexpr float r = 1.0f;
        vertices[0] << 0.0f,     0.0f,     0.0f;     // Origin
        vertices[1] << 0.0f,     0.0f,     r * 3.0f; // Z direction
        vertices[2] << r * 1.0f, 0.0f,     0.0f;     // X direction
        vertices[3] << 0.0f,     r * 1.0f, 0.0f;     // Y direction
        
        auto& colors = *lines->getOrCreateColors(3);
        colors[0] << 1.0f, 0.0f, 0.0f; // Red
        colors[1] << 0.0f, 0.8f, 0.0f; // Green
        colors[2] << 0.0f, 0.0f, 1.0f; // Blue
        
        lines->setNumLines(5);
        lines->setLineWidth(2.0f);
        lines->resizeColorIndicesForNumLines(5);
        // Origin -> Z, Blue
        lines->setLine(0, 0, 1);    
        lines->setLineColor(0, 2);
        // Origin -> X, Red
        lines->setLine(1, 0, 2);
        lines->setLineColor(1, 0);
        // Origin -> Y, Green
        lines->setLine(2, 0, 3);
        lines->setLineColor(2, 1);
        // Z -> X, Red
        lines->setLine(3, 1, 2);
        lines->setLineColor(3, 0);
        // Z -> Y, Green
        lines->setLine(4, 1, 3);
        lines->setLineColor(4, 1);

        auto fixedPixelSizeGroup = new SgFixedPixelSizeGroup;
        fixedPixelSizeGroup->setPixelSizeRatio(7.0f);
        fixedPixelSizeGroup->addChild(lines);
        
        marker = fixedPixelSizeGroup;
    }

    return marker;
}


void ScenePositionTagGroup::removeTagNode(int index)
{
    offsetTransform->removeChildAt(index);
    update.resetAction(SgUpdate::REMOVED);
    offsetTransform->notifyUpdate(update);
}


void ScenePositionTagGroup::updateTagNodePosition(int index)
{
    auto tag = itemImpl->tags->tagAt(index);
    auto node = static_cast<SgPosTransform*>(offsetTransform->child(index));
    node->setTranslation(tag->translation());
    update.resetAction(SgUpdate::MODIFIED);
    node->notifyUpdate(update);
}


void ScenePositionTagGroup::setOffsetPosition(const Position& T)
{
    offsetTransform->setPosition(T);
    update.resetAction(SgUpdate::MODIFIED);
    offsetTransform->notifyUpdate(update);
}


void ScenePositionTagGroup::setParentPosition(const Position& T)
{
    setPosition(T);
    update.resetAction(SgUpdate::MODIFIED);
    notifyUpdate(update);
}


PositionTagGroupLocation::PositionTagGroupLocation(PositionTagGroupItem::Impl* itemImpl)
    : itemImpl(itemImpl)
{

}


int PositionTagGroupLocation::getType() const
{
    return ParentRelativeLocation;
}


Item* PositionTagGroupLocation::getCorrespondingItem()
{
    return itemImpl->self;
}


Position PositionTagGroupLocation::getLocation() const
{
    return Position::Identity();
}


void PositionTagGroupLocation::setLocation(const Position& T)
{

}


SignalProxy<void()> PositionTagGroupLocation::sigLocationChanged()
{
    return itemImpl->sigLocationChanged;
}
