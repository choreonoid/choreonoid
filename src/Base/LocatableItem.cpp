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


bool LocationProxy::setLocation(const Isometry3& /* T */)
{
    return false;
}


void LocationProxy::finishLocationEditing()
{

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


Isometry3 LocationProxy::getGlobalLocation() const
{
    return getGlobalLocationOf(getLocation());
}


Isometry3 LocationProxy::getGlobalLocationOf(const Isometry3 T) const
{
    switch(locationType_){
    case GlobalLocation:
        return T;
    case ParentRelativeLocation:
    case OffsetLocation:
        if(auto parent = getParentLocationProxy()){
            return parent->getGlobalLocation() * T;
        } else {
            return T;
        }
    default:
        return Isometry3::Identity();
    }
}


bool LocationProxy::setGlobalLocation(const Isometry3& T)
{
    switch(locationType_){
    case GlobalLocation:
        return setLocation(T);
    case ParentRelativeLocation:
    case OffsetLocation:
        if(auto parent = getParentLocationProxy()){
            return setLocation(
                parent->getGlobalLocation().inverse(Eigen::Isometry) * T);
        }
    default:
        break;
    }
    return false;
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
