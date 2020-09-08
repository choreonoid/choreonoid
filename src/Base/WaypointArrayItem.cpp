#include "WaypointArrayItem.h"
#include "SceneWidgetEditable.h"
//#include "ItemManager.h"
#include <cnoid/WaypointArray>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

class SceneWaypointArray : public SgPosTransform, public SceneWidgetEditable
{
public:
    WaypointArrayItem::Impl* impl;
    
    SceneWaypointArray(WaypointArrayItem::Impl* impl);

};

typedef ref_ptr<SceneWaypointArray> SceneWaypointArrayPtr;


class WaypointArrayLocation : public LocationProxy
{
public:
    WaypointArrayItem::Impl* impl;

    WaypointArrayLocation(WaypointArrayItem::Impl* impl);
    virtual int getType() const override;
    virtual Item* getCorrespondingItem() override;
    virtual Position getLocation() const override;
    virtual void setLocation(const Position& T) override;
    virtual SignalProxy<void()> sigLocationChanged() override;
};

typedef ref_ptr<WaypointArrayLocation> WaypointArrayLocationPtr;

}

namespace cnoid {

class WaypointArrayItem::Impl
{
public:
    WaypointArrayItem* self;
    WaypointArrayPtr waypoints;
    SceneWaypointArrayPtr scene;
    WaypointArrayLocationPtr location;
    SignalProxy<void()> sigLocationChanged;
    
    Impl(WaypointArrayItem* self);
};

}


/*
void WaypointArrayItem:initializeClass(ExtensionManager* ext)
{
    auto& im = ext->itemManager();
    im.registerClass<WaypointArrayItem>(N_("WaypointArrayItem"));
}
*/


WaypointArrayItem::WaypointArrayItem()
{
    impl = new Impl(this);
}


WaypointArrayItem::WaypointArrayItem(const WaypointArrayItem& org)
    : Item(org)
{
    impl = new Impl(this);
}


WaypointArrayItem::Impl::Impl(WaypointArrayItem* self)
    : self(self)
{
    waypoints = new WaypointArray;
}


WaypointArrayItem::~WaypointArrayItem()
{
    delete impl;
}


Item* WaypointArrayItem::doDuplicate() const
{
    return new WaypointArrayItem(*this);
}


const WaypointArray* WaypointArrayItem::waypoints() const
{
    return impl->waypoints;
}


WaypointArray* WaypointArrayItem::waypoints()
{
    return impl->waypoints;
}


SgNode* WaypointArrayItem::getScene()
{
    if(!impl->scene){
        impl->scene = new SceneWaypointArray(impl);
    }
    return impl->scene;
}
    

LocationProxyPtr WaypointArrayItem::getLocationProxy()
{
    if(!impl->location){
        impl->location = new WaypointArrayLocation(impl);
    }
    return impl->location;
    
}


bool WaypointArrayItem::store(Archive& archive)
{
    return true;
}


bool WaypointArrayItem::restore(const Archive& archive)
{
    return true;
}


SceneWaypointArray::SceneWaypointArray(WaypointArrayItem::Impl* itemImpl)
    : impl(impl)
{

}


WaypointArrayLocation::WaypointArrayLocation(WaypointArrayItem::Impl* impl)
    : impl(impl)
{

}


int WaypointArrayLocation::getType() const
{
    return ParentRelativeLocation;
}


Item* WaypointArrayLocation::getCorrespondingItem()
{
    return impl->self;
}


Position WaypointArrayLocation::getLocation() const
{
    return Position::Identity();
}


void WaypointArrayLocation::setLocation(const Position& T)
{

}


SignalProxy<void()> WaypointArrayLocation::sigLocationChanged()
{
    return impl->sigLocationChanged;
}
