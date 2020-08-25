#include "WaypointListItem.h"
#include "SceneWidgetEditable.h"
//#include "ItemManager.h"
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

class SceneWaypointList : public SgPosTransform, public SceneWidgetEditable
{
public:
    WaypointListItem::Impl* impl;
    
    SceneWaypointList(WaypointListItem::Impl* impl);

};

typedef ref_ptr<SceneWaypointList> SceneWaypointListPtr;


class WaypointListLocation : public LocationProxy
{
public:
    WaypointListItem::Impl* impl;

    WaypointListLocation(WaypointListItem::Impl* impl);
    virtual int getType() const override;
    virtual Item* getCorrespondingItem() override;
    virtual Position getLocation() const override;
    virtual void setLocation(const Position& T) override;
    virtual SignalProxy<void()> sigLocationChanged() override;
};

typedef ref_ptr<WaypointListLocation> WaypointListLocationPtr;

}

namespace cnoid {

class WaypointListItem::Impl
{
public:
    WaypointListItem* self;
    SceneWaypointListPtr scene;
    WaypointListLocationPtr location;
    SignalProxy<void()> sigLocationChanged;
    
    Impl(WaypointListItem* self);
};

}


/*
void WaypointListItem:initializeClass(ExtensionManager* ext)
{
    auto& im = ext->itemManager();
    im.registerClass<WaypointListItem>(N_("WaypointListItem"));
}
*/


WaypointListItem::WaypointListItem()
{
    impl = new Impl(this);
}


WaypointListItem::WaypointListItem(const WaypointListItem& org)
    : Item(org)
{
    impl = new Impl(this);
}


WaypointListItem::Impl::Impl(WaypointListItem* self)
    : self(self)
{

}


WaypointListItem::~WaypointListItem()
{
    delete impl;
}


Item* WaypointListItem::doDuplicate() const
{
    return new WaypointListItem(*this);
}


SgNode* WaypointListItem::getScene()
{
    if(!impl->scene){
        impl->scene = new SceneWaypointList(impl);
    }
    return impl->scene;
}
    

LocationProxyPtr WaypointListItem::getLocationProxy()
{
    if(!impl->location){
        impl->location = new WaypointListLocation(impl);
    }
    return impl->location;
    
}


bool WaypointListItem::store(Archive& archive)
{
    return true;
}


bool WaypointListItem::restore(const Archive& archive)
{
    return true;
}


SceneWaypointList::SceneWaypointList(WaypointListItem::Impl* itemImpl)
    : impl(impl)
{

}


WaypointListLocation::WaypointListLocation(WaypointListItem::Impl* impl)
    : impl(impl)
{

}


int WaypointListLocation::getType() const
{
    return ParentRelativeLocation;
}


Item* WaypointListLocation::getCorrespondingItem()
{
    return impl->self;
}


Position WaypointListLocation::getLocation() const
{
    return Position::Identity();
}


void WaypointListLocation::setLocation(const Position& T)
{

}


SignalProxy<void()> WaypointListLocation::sigLocationChanged()
{
    return impl->sigLocationChanged;
}
