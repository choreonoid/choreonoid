#include "Item.h"
#include "LocatableItem.h"

using namespace std;
using namespace cnoid;

namespace {

Signal<bool(LocationProxyPtr location), LogicalSum> sigEditRequest;

}


LocationProxy::LocationProxy(LocationType type)
    : locationType_(type)
{
    isEditable_ = true;
}


LocationProxy::~LocationProxy()
{
    sigExpired_();
}


std::string LocationProxy::getName() const
{
    auto self = const_cast<LocationProxy*>(this);
    if(auto item = self->getCorrespondingItem()){
        if(!itemNameConnection_.connected()){
            self->itemNameConnection_ = item->sigNameChanged().connect(
                [self](const std::string&){ self->notifyAttributeChange(); });
        }
        return item->displayName();
    }
    return std::string();
}


Position LocationProxy::getGlobalLocation() const
{
    switch(locationType_){
    case GlobalLocation:
        return getLocation();
    case ParentRelativeLocation:
    case OffsetLocation:
        if(auto parent = getParentLocationProxy()){
            return parent->getGlobalLocation() * getLocation();
        } else {
            return getLocation();
        }
    default:
        return Position::Identity();
    }
}


bool LocationProxy::isEditable() const
{
    return isEditable_;
}


void LocationProxy::setEditable(bool on)
{
    if(on != isEditable_){
        isEditable_ = on;
        sigAttributeChanged_();
    }
}


bool LocationProxy::setLocation(const Position& /* T */)
{
    return false;
}


Item* LocationProxy::getCorrespondingItem()
{
    return nullptr;
}


LocationProxyPtr LocationProxy::getParentLocationProxy() const
{
    if(auto item = const_cast<LocationProxy*>(this)->getCorrespondingItem()){
        if(auto parentLocatableItem = item->findOwnerItem<LocatableItem>()){
            return parentLocatableItem->getLocationProxy();
        }
    }
    return nullptr;
}


void LocationProxy::expire()
{
    sigExpired_();
}


SignalProxy<void()> LocationProxy::sigAttributeChanged()
{
    return sigAttributeChanged_;
}


void LocationProxy::notifyAttributeChange()
{
    return sigAttributeChanged_();
}


SignalProxy<void()> LocationProxy::sigExpired()
{
    return sigExpired_;
}


bool LocationProxy::requestEdit()
{
    return ::sigEditRequest(this);
}


SignalProxy<bool(LocationProxyPtr item), LogicalSum> LocationProxy::sigEditRequest()
{
    return ::sigEditRequest;
}
