#include "Item.h"
#include "LocatableItem.h"

using namespace std;
using namespace cnoid;

namespace {

Signal<bool(LocationProxyPtr location), LogicalSum> sigEditRequest;

}


LocationProxy::LocationProxy()
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


void LocationProxy::setLocation(const Position& /* T */)
{

}


Item* LocationProxy::getCorrespondingItem()
{
    return nullptr;
}


LocationProxyPtr LocationProxy::getParentLocationProxy()
{
    if(auto item = getCorrespondingItem()){
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
