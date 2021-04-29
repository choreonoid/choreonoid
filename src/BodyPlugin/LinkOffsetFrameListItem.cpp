#include "LinkOffsetFrameListItem.h"
#include "BodyItem.h"
#include <cnoid/ItemManager>
#include <cnoid/Link>
#include <cnoid/LinkKinematicsKit>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class LinkOffsetFrameListItem::Impl
{
public:
    LocationProxyPtr linkLocation;
};

}


void LinkOffsetFrameListItem::initializeClass(ExtensionManager* ext)
{
    auto& im = ext->itemManager();
    im.registerClass<LinkOffsetFrameListItem, CoordinateFrameListItem>(N_("LinkOffsetFrameListItem"));
    im.addCreationPanel<LinkOffsetFrameListItem>();
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


LocationProxyPtr LinkOffsetFrameListItem::getFrameParentLocationProxy()
{
    if(!impl->linkLocation){
        if(auto bodyItem = findOwnerItem<BodyItem>()){
            // Currently a body that has a unique end link is supported.
            if(auto kinematicsKit = bodyItem->findPresetLinkKinematicsKit()){
                impl->linkLocation = bodyItem->createLinkLocationProxy(kinematicsKit->link());
            }
        }
    }
    return impl->linkLocation;
}
