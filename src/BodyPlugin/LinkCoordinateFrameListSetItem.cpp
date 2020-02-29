#include "LinkCoordinateFrameListSetItem.h"
#include "BodyItem.h"
#include <cnoid/CoordinateFrameListItem>
#include <cnoid/LocatableItem>
#include <cnoid/ItemManager>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

string worldFrameListLabel;
string bodyFrameListLabel;
string linkFrameListLabel;

bool enabledFlags[] = { 1, 1, 1 };

Signal<void(LinkCoordinateFrameListSetItem::FrameType type, bool on)> sigEnabledFrameListsChanged;
Signal<void(LinkCoordinateFrameListSetItem* frameListSetItem)> sigInstanceAddedOrUpdated_;

class LinkCoordinateFrameListItem;

class LinkCoordLocatableItem : public LocatableItem
{
public:
    LinkCoordinateFrameListItem* frameListItem;
    weak_ref_ptr<BodyItem> weakBodyItem;
    LinkPtr parentLink;
    
    LinkCoordLocatableItem(LinkCoordinateFrameListItem* frameListItem);
    void setTarget(BodyItem* bodyItem, Link* link);
    virtual Item* getCorrespondingItem() override;
    virtual std::string getLocationName() const override;
    virtual Position getLocation() const  override;
    virtual bool isLocationEditable() const override;
    virtual void setLocation(const Position& T) override;
    virtual SignalProxy<void()> sigLocationChanged()  override;
    virtual LocatableItem* getParentLocatableItem() override;
};

class LinkCoordinateFrameListItem : public CoordinateFrameListItem
{
public:
    LinkCoordLocatableItem linkCoordLocatableItem;
    
    LinkCoordinateFrameListItem();
    LinkCoordinateFrameListItem(const LinkCoordinateFrameListItem& org);
    virtual Item* doDuplicate() const override;
    virtual LocatableItem* getParentLocatableItem() override;
};

}

void LinkCoordinateFrameListSetItem::initializeClass(ExtensionManager* ext)
{
    auto& im = ext->itemManager();
    im.registerClass<LinkCoordinateFrameListSetItem, MultiCoordinateFrameListItem>(
        N_("LinkCoordinateFrameListSetItem"));
    im.addCreationPanel<LinkCoordinateFrameListSetItem>();

    im.registerClass<LinkCoordinateFrameListItem, CoordinateFrameListItem>(
        "LinkCoordinateFrameListItem");

    worldFrameListLabel = "World";
    bodyFrameListLabel = "Body";
    linkFrameListLabel = "Link";
}


SignalProxy<void(LinkCoordinateFrameListSetItem* frameListSetItem)>
LinkCoordinateFrameListSetItem::sigInstanceAddedOrUpdated()
{
    return ::sigInstanceAddedOrUpdated_;
}


void LinkCoordinateFrameListSetItem::setFrameListLabels
(const char* worldFrameLabel, const char* bodyFrameLabel, const char* linkFrameLabel)
{
    ::worldFrameListLabel = worldFrameLabel;
    ::bodyFrameListLabel = bodyFrameLabel;
    ::linkFrameListLabel = linkFrameLabel;
}


void LinkCoordinateFrameListSetItem::setFrameListEnabledForAllItems(FrameType type, bool on)
{
    enabledFlags[type] = on;
    sigEnabledFrameListsChanged(type, on);
}


LinkCoordinateFrameListSetItem::LinkCoordinateFrameListSetItem()
    : MultiCoordinateFrameListItem()
{
    setNumFrameLists(3);
    
    auto worldFrameListItem = new CoordinateFrameListItem;
    worldFrameListItem->setName(::worldFrameListLabel);
    setFrameListItem(WorldFrame, worldFrameListItem);

    auto bodyFrameListItem = new CoordinateFrameListItem;
    bodyFrameListItem->setName(::bodyFrameListLabel);
    setFrameListItem(BodyFrame, bodyFrameListItem);

    auto linkFrameListItem = new LinkCoordinateFrameListItem;
    linkFrameListItem->setName(::linkFrameListLabel);
    setFrameListItem(LinkFrame, linkFrameListItem);
    
    replaceFrameListContainer(new LinkCoordinateFrameSet);
    initializeFrameListEnabling();
}


LinkCoordinateFrameListSetItem::LinkCoordinateFrameListSetItem(const LinkCoordinateFrameListSetItem& org)
    : MultiCoordinateFrameListItem(org)
{
    replaceFrameListContainer(new LinkCoordinateFrameSet);
    initializeFrameListEnabling();
}


void LinkCoordinateFrameListSetItem::initializeFrameListEnabling()
{
    for(size_t i=0; i < 3; ++i){
        if(!enabledFlags[i]){
            setFrameListEnabled(i, false);
        }
    }
    
    connection = sigEnabledFrameListsChanged.connect(
        [&](int type, bool on){
            setFrameListEnabled(type, on); });
}


Item* LinkCoordinateFrameListSetItem::doDuplicate() const
{
    return new LinkCoordinateFrameListSetItem(*this);
}


void LinkCoordinateFrameListSetItem::onPositionChanged()
{
    ::sigInstanceAddedOrUpdated_(this);
}


LinkCoordinateFrameSet* LinkCoordinateFrameListSetItem::frameSets()
{
    return static_cast<LinkCoordinateFrameSet*>(MultiCoordinateFrameListItem::frameSets());
}


const LinkCoordinateFrameSet* LinkCoordinateFrameListSetItem::frameSets() const
{
    return static_cast<const LinkCoordinateFrameSet*>(MultiCoordinateFrameListItem::frameSets());
}


CoordinateFrameListItem* LinkCoordinateFrameListSetItem::worldFrameListItem(int index)
{
    return frameListItem(WorldFrame);
}


const CoordinateFrameListItem* LinkCoordinateFrameListSetItem::worldFrameListItem(int index) const
{
    return frameListItem(WorldFrame);
}    


CoordinateFrameListItem* LinkCoordinateFrameListSetItem::bodyFrameListItem(int index)
{
    return frameListItem(BodyFrame);
}
    

const CoordinateFrameListItem* LinkCoordinateFrameListSetItem::bodyFrameListItem(int index) const
{
    return frameListItem(BodyFrame);
}    


CoordinateFrameListItem* LinkCoordinateFrameListSetItem::linkFrameListItem(int index)
{
    return frameListItem(LinkFrame);
}
    

const CoordinateFrameListItem* LinkCoordinateFrameListSetItem::linkFrameListItem(int index) const
{
    return frameListItem(LinkFrame);
}


LinkCoordinateFrameListItem::LinkCoordinateFrameListItem()
    : CoordinateFrameListItem(),
      linkCoordLocatableItem(this)
{

}


LinkCoordinateFrameListItem::LinkCoordinateFrameListItem(const LinkCoordinateFrameListItem& org)
    : CoordinateFrameListItem(org),
      linkCoordLocatableItem(this)
{

}


Item* LinkCoordinateFrameListItem::doDuplicate() const
{
    return new LinkCoordinateFrameListItem(*this);
}


LocatableItem* LinkCoordinateFrameListItem::getParentLocatableItem()
{
    if(auto bodyItem = findOwnerItem<BodyItem>()){
        auto body = bodyItem->body();
        if(auto link = body->findUniqueEndLink()){
            linkCoordLocatableItem.setTarget(bodyItem, link);
            return &linkCoordLocatableItem;
        }
    }
    return nullptr;
}


LinkCoordLocatableItem::LinkCoordLocatableItem(LinkCoordinateFrameListItem* frameListItem)
    : frameListItem(frameListItem)
{

}


void LinkCoordLocatableItem::setTarget(BodyItem* bodyItem, Link* link)
{
    weakBodyItem = bodyItem;
    parentLink = link;
}


Item* LinkCoordLocatableItem::getCorrespondingItem()
{
    return weakBodyItem.lock();
}


std::string LinkCoordLocatableItem::getLocationName() const
{
    return parentLink->body()->name() + " - " +  parentLink->name();
}


Position LinkCoordLocatableItem::getLocation() const
{
    return parentLink->position();
}


bool LinkCoordLocatableItem::isLocationEditable() const
{
    return false;
}


void LinkCoordLocatableItem::setLocation(const Position&)
{

}


LocatableItem* LinkCoordLocatableItem::getParentLocatableItem()
{
    return nullptr;
}


SignalProxy<void()> LinkCoordLocatableItem::sigLocationChanged()
{
    return weakBodyItem.lock()->sigKinematicStateChanged();
}



