#include "LinkCoordFrameListSuiteItem.h"
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

Signal<void(LinkCoordFrameListSuiteItem::FrameType type, bool on)> sigEnabledFrameListsChanged;
Signal<void(LinkCoordFrameListSuiteItem* frameListSetItem)> sigInstanceAddedOrUpdated_;

class LinkCoordFrameListItem;

class LinkCoordLocatableItem : public LocatableItem
{
public:
    LinkCoordFrameListItem* frameListItem;
    weak_ref_ptr<BodyItem> weakBodyItem;
    LinkPtr parentLink;
    
    LinkCoordLocatableItem(LinkCoordFrameListItem* frameListItem);
    void setTarget(BodyItem* bodyItem, Link* link);
    virtual Item* getCorrespondingItem() override;
    virtual std::string getLocationName() const override;
    virtual Position getLocation() const  override;
    virtual bool isLocationEditable() const override;
    virtual void setLocation(const Position& T) override;
    virtual SignalProxy<void()> sigLocationChanged()  override;
    virtual LocatableItem* getParentLocatableItem() override;
};

class LinkCoordFrameListItem : public CoordinateFrameListItem
{
public:
    LinkCoordLocatableItem linkCoordLocatableItem;
    
    LinkCoordFrameListItem();
    LinkCoordFrameListItem(const LinkCoordFrameListItem& org);
    virtual Item* doDuplicate() const override;
    virtual LocatableItem* getParentLocatableItem() override;
};

}

void LinkCoordFrameListSuiteItem::initializeClass(ExtensionManager* ext)
{
    auto& im = ext->itemManager();
    im.registerClass<LinkCoordFrameListSuiteItem, CoordinateFrameListSuiteItem>(
        N_("LinkCoordFrameListSuiteItem"));
    im.addCreationPanel<LinkCoordFrameListSuiteItem>();

    im.registerClass<LinkCoordFrameListItem, CoordinateFrameListItem>(
        "LinkCoordFrameListItem");

    worldFrameListLabel = "World";
    bodyFrameListLabel = "Body";
    linkFrameListLabel = "Link";
}


SignalProxy<void(LinkCoordFrameListSuiteItem* frameListSetItem)>
LinkCoordFrameListSuiteItem::sigInstanceAddedOrUpdated()
{
    return ::sigInstanceAddedOrUpdated_;
}


void LinkCoordFrameListSuiteItem::setFrameListLabels
(const char* worldFrameLabel, const char* bodyFrameLabel, const char* linkFrameLabel)
{
    ::worldFrameListLabel = worldFrameLabel;
    ::bodyFrameListLabel = bodyFrameLabel;
    ::linkFrameListLabel = linkFrameLabel;
}


void LinkCoordFrameListSuiteItem::setFrameListEnabledForAllItems(FrameType type, bool on)
{
    enabledFlags[type] = on;
    sigEnabledFrameListsChanged(type, on);
}


LinkCoordFrameListSuiteItem::LinkCoordFrameListSuiteItem()
    : CoordinateFrameListSuiteItem()
{
    setNumFrameLists(3);
    
    auto worldFrameListItem = new CoordinateFrameListItem;
    worldFrameListItem->setName(::worldFrameListLabel);
    setFrameListItem(WorldFrame, worldFrameListItem);

    auto bodyFrameListItem = new CoordinateFrameListItem;
    bodyFrameListItem->setName(::bodyFrameListLabel);
    setFrameListItem(BodyFrame, bodyFrameListItem);

    auto linkFrameListItem = new LinkCoordFrameListItem;
    linkFrameListItem->setName(::linkFrameListLabel);
    setFrameListItem(LinkFrame, linkFrameListItem);
    
    replaceFrameListContainer(new LinkCoordFrameSetSuite);
    initializeFrameListEnabling();
}


LinkCoordFrameListSuiteItem::LinkCoordFrameListSuiteItem(const LinkCoordFrameListSuiteItem& org)
    : CoordinateFrameListSuiteItem(org)
{
    replaceFrameListContainer(new LinkCoordFrameSetSuite);
    initializeFrameListEnabling();
}


void LinkCoordFrameListSuiteItem::initializeFrameListEnabling()
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


Item* LinkCoordFrameListSuiteItem::doDuplicate() const
{
    return new LinkCoordFrameListSuiteItem(*this);
}


void LinkCoordFrameListSuiteItem::onPositionChanged()
{
    ::sigInstanceAddedOrUpdated_(this);
}


LinkCoordFrameSetSuite* LinkCoordFrameListSuiteItem::frameSetSuite()
{
    return static_cast<LinkCoordFrameSetSuite*>(CoordinateFrameListSuiteItem::frameSetSuite());
}


const LinkCoordFrameSetSuite* LinkCoordFrameListSuiteItem::frameSetSuite() const
{
    return static_cast<const LinkCoordFrameSetSuite*>(CoordinateFrameListSuiteItem::frameSetSuite());
}


CoordinateFrameListItem* LinkCoordFrameListSuiteItem::worldFrameListItem(int index)
{
    return frameListItem(WorldFrame);
}


const CoordinateFrameListItem* LinkCoordFrameListSuiteItem::worldFrameListItem(int index) const
{
    return frameListItem(WorldFrame);
}    


CoordinateFrameListItem* LinkCoordFrameListSuiteItem::bodyFrameListItem(int index)
{
    return frameListItem(BodyFrame);
}
    

const CoordinateFrameListItem* LinkCoordFrameListSuiteItem::bodyFrameListItem(int index) const
{
    return frameListItem(BodyFrame);
}    


CoordinateFrameListItem* LinkCoordFrameListSuiteItem::linkFrameListItem(int index)
{
    return frameListItem(LinkFrame);
}
    

const CoordinateFrameListItem* LinkCoordFrameListSuiteItem::linkFrameListItem(int index) const
{
    return frameListItem(LinkFrame);
}


LinkCoordFrameListItem::LinkCoordFrameListItem()
    : CoordinateFrameListItem(),
      linkCoordLocatableItem(this)
{

}


LinkCoordFrameListItem::LinkCoordFrameListItem(const LinkCoordFrameListItem& org)
    : CoordinateFrameListItem(org),
      linkCoordLocatableItem(this)
{

}


Item* LinkCoordFrameListItem::doDuplicate() const
{
    return new LinkCoordFrameListItem(*this);
}


LocatableItem* LinkCoordFrameListItem::getParentLocatableItem()
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


LinkCoordLocatableItem::LinkCoordLocatableItem(LinkCoordFrameListItem* frameListItem)
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



