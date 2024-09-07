#include "Item.h"
#include "LocatableItem.h"

using namespace std;
using namespace cnoid;

namespace {

Signal<bool(LocationProxyPtr location), LogicalSum> sigEditRequest;

}


LocationProxy::LocationProxy(Item* locatableItem, LocationType type)
    : locatableItemRef(locatableItem),
      locationType_(type)
{
    isLocked_ = false;
}


LocationProxy::~LocationProxy()
{
    sigExpired_();
}


void LocationProxy::setNameDependencyOnItemName()
{
    if(auto item = locatableItem()){
        if(!itemNameConnection_.connected()){
            itemNameConnection_ = item->sigNameChanged().connect(
                [this](const std::string&){ notifyAttributeChange(); });
        }
    }
}


std::string LocationProxy::getName() const
{
    auto self = const_cast<LocationProxy*>(this);
    if(auto item = self->locatableItem()){
        if(!itemNameConnection_.connected()){
            self->setNameDependencyOnItemName();
        }
        return item->displayName();
    }
    return std::string();
}


std::string LocationProxy::getCategory() const
{
    return std::string();
}


bool LocationProxy::isLocked() const
{
    return isLocked_;
}


void LocationProxy::setLocked(bool on)
{
    if(on != isLocked_){
        isLocked_ = on;
        sigAttributeChanged_();
    }
}


bool LocationProxy::isContinuousUpdateState() const
{
    auto self = const_cast<LocationProxy*>(this);
    if(auto item = self->locatableItem()){
        if(!self->continuousUpdateStateConnection_.connected()){
            self->continuousUpdateStateConnection_ =
                item->sigContinuousUpdateStateChanged().connect(
                    [self](bool){ self->notifyAttributeChange(); });
        }
        return item->isContinuousUpdateState();
    }
    return false;
}


bool LocationProxy::setLocation(const Isometry3& /* T */)
{
    return false;
}


void LocationProxy::finishLocationEditing()
{

}


LocationProxyPtr LocationProxy::getParentLocationProxy()
{
    if(auto item = locatableItem()){
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
        if(auto parent = const_cast<LocationProxy*>(this)->getParentLocationProxy()){
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


LocationProxyPtr LocatableItem::getLocationProxy()
{
    return nullptr;
}


std::vector<LocationProxyPtr> LocatableItem::getLocationProxies()
{
    return std::vector<LocationProxyPtr>();
}


SignalProxy<void()> LocatableItem::getSigLocationProxiesChanged()
{
    return SignalProxy<void()>();
}
