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
string endFrameListLabel;

bool enabledFlags[] = { 1, 1, 1 };

Signal<void(LinkCoordinateFrameListSetItem::FrameType type, bool on)> sigEnabledFrameListsChanged;
Signal<void(LinkCoordinateFrameListSetItem* frameListSetItem)> sigInstanceAddedOrUpdated_;

class EndCoordinateFrameListItem;

class EndLocatableItem : public LocatableItem
{
public:
    EndCoordinateFrameListItem* frameListItem;
    weak_ref_ptr<BodyItem> weakBodyItem;
    LinkPtr parentLink;
    
    EndLocatableItem(EndCoordinateFrameListItem* frameListItem);
    void setTarget(BodyItem* bodyItem, Link* link);
    virtual Item* getCorrespondingItem() override;
    virtual std::string getLocationName() const override;
    virtual Position getLocation() const  override;
    virtual bool isLocationEditable() const override;
    virtual void setLocation(const Position& T) override;
    virtual SignalProxy<void()> sigLocationChanged()  override;
    virtual LocatableItem* getParentLocatableItem() override;
};

class EndCoordinateFrameListItem : public CoordinateFrameListItem
{
public:
    EndLocatableItem endLocatableItem;
    
    EndCoordinateFrameListItem();
    EndCoordinateFrameListItem(const EndCoordinateFrameListItem& org);
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

    im.registerClass<EndCoordinateFrameListItem, CoordinateFrameListItem>(
        "EndCoordinateFrameListItem");

    worldFrameListLabel = "World";
    bodyFrameListLabel = "Body";
    endFrameListLabel = "End";
}


SignalProxy<void(LinkCoordinateFrameListSetItem* frameListSetItem)>
LinkCoordinateFrameListSetItem::sigInstanceAddedOrUpdated()
{
    return ::sigInstanceAddedOrUpdated_;
}


void LinkCoordinateFrameListSetItem::setFrameListLabels
(const char* worldFrameLabel, const char* bodyFrameLabel, const char* endFrameLabel)
{
    ::worldFrameListLabel = worldFrameLabel;
    ::bodyFrameListLabel = bodyFrameLabel;
    ::endFrameListLabel = endFrameLabel;
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

    auto endFrameListItem = new EndCoordinateFrameListItem;
    endFrameListItem->setName(::endFrameListLabel);
    setFrameListItem(EndFrame, endFrameListItem);
    
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


CoordinateFrameListItem* LinkCoordinateFrameListSetItem::endFrameListItem(int index)
{
    return frameListItem(EndFrame);
}
    

const CoordinateFrameListItem* LinkCoordinateFrameListSetItem::endFrameListItem(int index) const
{
    return frameListItem(EndFrame);
}


EndCoordinateFrameListItem::EndCoordinateFrameListItem()
    : CoordinateFrameListItem(),
      endLocatableItem(this)
{

}


EndCoordinateFrameListItem::EndCoordinateFrameListItem(const EndCoordinateFrameListItem& org)
    : CoordinateFrameListItem(org),
      endLocatableItem(this)
{

}


Item* EndCoordinateFrameListItem::doDuplicate() const
{
    return new EndCoordinateFrameListItem(*this);
}


LocatableItem* EndCoordinateFrameListItem::getParentLocatableItem()
{
    if(auto bodyItem = findOwnerItem<BodyItem>()){
        auto body = bodyItem->body();
        if(auto link = body->findUniqueEndLink()){
            endLocatableItem.setTarget(bodyItem, link);
            return &endLocatableItem;
        }
    }
    return nullptr;
}


EndLocatableItem::EndLocatableItem(EndCoordinateFrameListItem* frameListItem)
    : frameListItem(frameListItem)
{

}


void EndLocatableItem::setTarget(BodyItem* bodyItem, Link* link)
{
    weakBodyItem = bodyItem;
    parentLink = link;
}


Item* EndLocatableItem::getCorrespondingItem()
{
    return weakBodyItem.lock();
}


std::string EndLocatableItem::getLocationName() const
{
    return parentLink->body()->name() + " - " +  parentLink->name();
}


Position EndLocatableItem::getLocation() const
{
    return parentLink->position();
}


bool EndLocatableItem::isLocationEditable() const
{
    return false;
}


void EndLocatableItem::setLocation(const Position&)
{

}


LocatableItem* EndLocatableItem::getParentLocatableItem()
{
    return nullptr;
}


SignalProxy<void()> EndLocatableItem::sigLocationChanged()
{
    return weakBodyItem.lock()->sigKinematicStateChanged();
}



