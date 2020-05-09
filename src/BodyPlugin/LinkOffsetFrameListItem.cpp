#include "LinkOffsetFrameListItem.h"
#include "BodyItem.h"
#include <cnoid/ItemManager>
#include <cnoid/LocatableItem>
#include <cnoid/Link>
#include <cnoid/LinkKinematicsKit>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

class LinkLocation : public LocatableItem
{
public:
    weak_ref_ptr<BodyItem> refBodyItem;
    weak_ref_ptr<Link> refLink;

    void setTarget(BodyItem* parentBodyItem, Link* parentLink);
    void resetTarget();
    virtual int getLocationType() const override;
    virtual std::string getLocationName() const override;
    virtual SignalProxy<void()> sigLocationChanged() override;
    virtual Position getLocation() const override;
    virtual bool isLocationEditable() const override;
    virtual LocatableItem* getParentLocatableItem() override;
};

}

namespace cnoid {

class LinkOffsetFrameListItem::Impl
{
public:
    unique_ptr<LinkLocation> linkLocation;
};

}


void LinkOffsetFrameListItem::initializeClass(ExtensionManager* ext)
{
    auto& im = ext->itemManager();
    im.registerClass<LinkOffsetFrameListItem, CoordinateFrameListItem>(N_("LinkOffsetFrameListItem"));
}


LinkOffsetFrameListItem::LinkOffsetFrameListItem()
{
    useAsOffsetFrames();
    impl = new Impl;
}


LinkOffsetFrameListItem::LinkOffsetFrameListItem(CoordinateFrameList* frameList)
    : CoordinateFrameListItem(frameList)
{
    useAsOffsetFrames();
    impl = new Impl;
}


LinkOffsetFrameListItem::LinkOffsetFrameListItem(const LinkOffsetFrameListItem& org)
    : CoordinateFrameListItem(org)
{
    impl = new Impl;
}


LinkOffsetFrameListItem::~LinkOffsetFrameListItem()
{
    delete impl;
}


Item* LinkOffsetFrameListItem::doDuplicate() const
{
    return new LinkOffsetFrameListItem(*this);
}


LocatableItem* LinkOffsetFrameListItem::getParentLocatableItem()
{
    if(auto bodyItem = findOwnerItem<BodyItem>()){
        // Currently a body that has a unique end link is supported.
        if(auto kinematicsKit = bodyItem->findPresetLinkKinematicsKit()){
            if(!impl->linkLocation){
                impl->linkLocation.reset(new LinkLocation);
            }
            impl->linkLocation->setTarget(bodyItem, kinematicsKit->link());
            return impl->linkLocation.get();
        }
    }
    if(impl->linkLocation){
        impl->linkLocation->resetTarget();
    }
    return nullptr;
}


void LinkLocation::setTarget(BodyItem* bodyItem, Link* link)
{
    refBodyItem = bodyItem;
    refLink = link;
}


void LinkLocation::resetTarget()
{
    refBodyItem.reset();
    refLink.reset();
}


int LinkLocation::getLocationType() const
{
    return GlobalLocation;
}


std::string LinkLocation::getLocationName() const
{
    if(auto link = refLink.lock()){
        return link->body()->name() + " - " + link->name();
    }
    return string();
}


SignalProxy<void()> LinkLocation::sigLocationChanged()
{
    if(auto bodyItem = refBodyItem.lock()){
        return bodyItem->sigKinematicStateChanged();
    } else {
        static Signal<void()> dummySignal;
        return dummySignal;
    }
}


Position LinkLocation::getLocation() const
{
    if(auto link = refLink.lock()){
        return link->Ta();
    }
    return Position::Identity();
}


bool LinkLocation::isLocationEditable() const
{
    return false;
}


LocatableItem* LinkLocation::getParentLocatableItem()
{
    return nullptr;
}
