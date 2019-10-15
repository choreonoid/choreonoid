#include "CoordinateFrameSetItem.h"
#include "PositionDragger.h"
#include "PositionEditManager.h"
#include <cnoid/CoordinateFrameSet>
#include <cnoid/CoordinateFrameContainer>
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/ConnectionSet>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class CoordinateFrameSetItem::Impl
{
public:
    CoordinateFrameSetItem* self;
    CoordinateFrameContainerPtr frames;
    
    PositionDraggerPtr positionDragger;
    SgUpdate update;
    AbstractPositionEditTarget* positionEditTarget;
    ScopedConnection managerConnection;
    ScopedConnectionSet targetConnections;

    Impl(CoordinateFrameSetItem* self);
    Impl(CoordinateFrameSetItem* self, const Impl& org);
    void setupPositionDragger();
    bool onPositionEditRequest(AbstractPositionEditTarget* target);
    void setPositionEditTarget(AbstractPositionEditTarget* target);
    void onEditTargetPositionChanged(const Position& T);
    void onDraggerPositionChanged();    
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
    setupPositionDragger();
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
    setupPositionDragger();
}


void CoordinateFrameSetItem::Impl::setupPositionDragger()
{
    positionDragger = new PositionDragger;
    positionDragger->setRadius(0.05);
    positionDragger->setDisplayMode(PositionDragger::DisplayInEditMode);
    positionDragger->setContainerMode(true);

    positionDragger->sigPositionDragged().connect(
        [&](){ onDraggerPositionChanged(); });

    positionEditTarget = nullptr;

    managerConnection =
        PositionEditManager::instance()->sigPositionEditRequest().connect(
            [&](AbstractPositionEditTarget* target){
                return onPositionEditRequest(target); });
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
    return impl->positionDragger;
}


bool CoordinateFrameSetItem::Impl::onPositionEditRequest(AbstractPositionEditTarget* target)
{
    if(auto frame = dynamic_cast<CoordinateFrame*>(target->getPositionObject())){
        if(frame->ownerFrameSet() == frames){
            setPositionEditTarget(target);
            return true;
        }
    }
    return false;
}


void CoordinateFrameSetItem::Impl::setPositionEditTarget(AbstractPositionEditTarget* target)
{
    targetConnections.disconnect();

    positionEditTarget = target;

    if(target){
        targetConnections.add(
            target->sigGlobalPositionChanged().connect(
                [&](const Position& T){ onEditTargetPositionChanged(T); }));

        targetConnections.add(
            target->sigPositionEditTargetExpired().connect(
                [&](){ setPositionEditTarget(nullptr); }));

        onEditTargetPositionChanged(target->getGlobalPosition());
    }
}


void CoordinateFrameSetItem::Impl::onEditTargetPositionChanged(const Position& T)
{
    positionDragger->setPosition(T);
    positionDragger->notifyUpdate(update);
}


void CoordinateFrameSetItem::Impl::onDraggerPositionChanged()
{
    if(positionEditTarget){
        targetConnections.block();
        Position T = positionDragger->T();
        positionEditTarget->setPosition(T, T, nullptr, nullptr);
        targetConnections.unblock();
    }
}


void CoordinateFrameSetItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Num coordinate frames"), impl->frames->numFrames());
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
