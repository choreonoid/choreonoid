#include "LinkCoordinateFrameListSetItem.h"
#include <cnoid/ItemManager>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

string worldFrameListLabel;
string bodyFrameListLabel;
string endFrameListLabel;

bool enabledFlags[] = { 1, 1, 1 };

Signal<void(int index, bool on)> sigEnabledFrameListsChanged;

}

void LinkCoordinateFrameListSetItem::initializeClass(ExtensionManager* ext)
{
    auto& im = ext->itemManager();
    im.registerClass<LinkCoordinateFrameListSetItem>(N_("LinkCoordinateFrameListSetItem"));
    im.addCreationPanel<LinkCoordinateFrameListSetItem>();

    worldFrameListLabel = "World";
    bodyFrameListLabel = "Body";
    endFrameListLabel = "End";
}


void LinkCoordinateFrameListSetItem::setFrameListLabels
(const char* worldFrameLabel, const char* bodyFrameLabel, const char* endFrameLabel)
{
    ::worldFrameListLabel = worldFrameLabel;
    ::bodyFrameListLabel = bodyFrameLabel;
    ::endFrameListLabel = endFrameLabel;
}


void LinkCoordinateFrameListSetItem::setFrameListEnabledForAllItems(int index, bool on)
{
    enabledFlags[index] = on;
    sigEnabledFrameListsChanged(index, on);
}


LinkCoordinateFrameListSetItem::LinkCoordinateFrameListSetItem()
    : MultiCoordinateFrameListItem{ worldFrameListLabel, bodyFrameListLabel, endFrameListLabel }
{
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
        [&](int index, bool on){
            setFrameListEnabled(index, on); });
}


Item* LinkCoordinateFrameListSetItem::doDuplicate() const
{
    return new LinkCoordinateFrameListSetItem(*this);
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


