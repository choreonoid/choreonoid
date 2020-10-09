#include "PositionTagListItem.h"
#include "ItemManager.h"
#include "SceneWidgetEditable.h"
#include "LazyCaller.h"
#include "Archive.h"
#include <cnoid/PositionTagList>
#include <cnoid/SceneDrawables>
#include <cnoid/ConnectionSet>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

class ScenePositionTagList : public SgPosTransform, public SceneWidgetEditable
{
public:
    PositionTagListItem::Impl* itemImpl;
    LazyCaller notifySceneUpdateLater;
    SgUpdate update;
    ScopedConnectionSet tagListConnections;
    
    ScenePositionTagList(PositionTagListItem::Impl* itemImpl);
    void finalize();
    void addTagNode(int index, bool doNotify);
    static SgNode* getOrCreateTagMarker();
    void removeTagNode(int index);
    void updateTagNodePosition(int index);
};

typedef ref_ptr<ScenePositionTagList> ScenePositionTagListPtr;


class PositionTagListLocation : public LocationProxy
{
public:
    PositionTagListItem::Impl* itemImpl;

    PositionTagListLocation(PositionTagListItem::Impl* itemImpl);
    virtual int getType() const override;
    virtual Item* getCorrespondingItem() override;
    virtual Position getLocation() const override;
    virtual void setLocation(const Position& T) override;
    virtual SignalProxy<void()> sigLocationChanged() override;
};

typedef ref_ptr<PositionTagListLocation> PositionTagListLocationPtr;

}

namespace cnoid {

class PositionTagListItem::Impl
{
public:
    PositionTagListItem* self;
    PositionTagListPtr tags;
    ScopedConnectionSet tagListConnections;
    LazyCaller notifyUpdateLater;
    ScenePositionTagListPtr scene;
    PositionTagListLocationPtr location;
    SignalProxy<void()> sigLocationChanged;
    
    Impl(PositionTagListItem* self);
};

}


void PositionTagListItem::initializeClass(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        auto& im = ext->itemManager();
        im.registerClass<PositionTagListItem>(N_("PositionTagListItem"));
        im.addCreationPanel<PositionTagListItem>();
        initialized = true;
    }
}


PositionTagListItem::PositionTagListItem()
{
    impl = new Impl(this);
}


PositionTagListItem::PositionTagListItem(const PositionTagListItem& org)
    : Item(org)
{
    impl = new Impl(this);
}


PositionTagListItem::Impl::Impl(PositionTagListItem* self)
    : self(self),
      notifyUpdateLater([=](){ self->notifyUpdate(); })
{
    tags = new PositionTagList;

    tagListConnections.add(
        tags->sigTagAdded().connect(
            [&](int index){ notifyUpdateLater(); }));
    tagListConnections.add(
        tags->sigTagRemoved().connect(
            [&](int index, PositionTag*){ notifyUpdateLater(); }));
    tagListConnections.add(
        tags->sigTagUpdated().connect(
            [&](int index){ notifyUpdateLater(); }));
}


PositionTagListItem::~PositionTagListItem()
{
    delete impl;
}


Item* PositionTagListItem::doDuplicate() const
{
    return new PositionTagListItem(*this);
}


const PositionTagList* PositionTagListItem::tags() const
{
    return impl->tags;
}


PositionTagList* PositionTagListItem::tags()
{
    return impl->tags;
}


SgNode* PositionTagListItem::getScene()
{
    if(!impl->scene){
        impl->scene = new ScenePositionTagList(impl);
    }
    return impl->scene;
}
    

LocationProxyPtr PositionTagListItem::getLocationProxy()
{
    if(!impl->location){
        impl->location = new PositionTagListLocation(impl);
    }
    return impl->location;
    
}


bool PositionTagListItem::store(Archive& archive)
{
    impl->tags->write(&archive);
    return true;
}


bool PositionTagListItem::restore(const Archive& archive)
{
    return impl->tags->read(&archive);
}


ScenePositionTagList::ScenePositionTagList(PositionTagListItem::Impl* itemImpl)
    : itemImpl(itemImpl)
{
    auto tags = itemImpl->tags;

    int n = tags->numTags();
    for(int i=0; i < n; ++i){
        addTagNode(i, false);
    }

    tagListConnections.add(
        tags->sigTagAdded().connect(
            [&](int index){ addTagNode(index, true); }));
    tagListConnections.add(
        tags->sigTagRemoved().connect(
            [&](int index, PositionTag*){ removeTagNode(index); }));
    tagListConnections.add(
        tags->sigTagUpdated().connect(
            [&](int index){ updateTagNodePosition(index); }));

    notifySceneUpdateLater.setFunction(
        [this](){
            notifyUpdate(update);
            update.resetAction();
        });

    update.resetAction();
}


void ScenePositionTagList::finalize()
{
    tagListConnections.disconnect();
    itemImpl = nullptr;
}


void ScenePositionTagList::addTagNode(int index, bool doNotify)
{
    auto tag = itemImpl->tags->tagAt(index);
    auto node = new SgPosTransform;
    node->addChild(getOrCreateTagMarker());
    node->setPosition(tag->position());
    insertChild(index, node);
    if(doNotify){
        update.setAction(SgUpdate::ADDED);
        notifySceneUpdateLater();
    }
}
    

SgNode* ScenePositionTagList::getOrCreateTagMarker()
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

        auto autoScale = new SgAutoScale;
        autoScale->setPixelSizeRatio(7.0f);
        autoScale->addChild(lines);
        
        marker = autoScale;
    }

    return marker;
}


void ScenePositionTagList::removeTagNode(int index)
{
    removeChildAt(index);
    update.setAction(SgUpdate::REMOVED);
    notifySceneUpdateLater();
}


void ScenePositionTagList::updateTagNodePosition(int index)
{
    if(notifySceneUpdateLater.isPending()){
        notifySceneUpdateLater.flush();
    }
    auto tag = itemImpl->tags->tagAt(index);
    auto node = static_cast<SgPosTransform*>(child(index));
    node->setTranslation(tag->translation());
    update.setAction(SgUpdate::MODIFIED);
    node->notifyUpdate(update);
    update.resetAction();
}


PositionTagListLocation::PositionTagListLocation(PositionTagListItem::Impl* itemImpl)
    : itemImpl(itemImpl)
{

}


int PositionTagListLocation::getType() const
{
    return ParentRelativeLocation;
}


Item* PositionTagListLocation::getCorrespondingItem()
{
    return itemImpl->self;
}


Position PositionTagListLocation::getLocation() const
{
    return Position::Identity();
}


void PositionTagListLocation::setLocation(const Position& T)
{

}


SignalProxy<void()> PositionTagListLocation::sigLocationChanged()
{
    return itemImpl->sigLocationChanged;
}
