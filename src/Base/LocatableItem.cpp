#include "Item.h"
#include "LocatableItem.h"
#include <utility>

using namespace std;
using namespace cnoid;

namespace {

Signal<bool(LocatableItem* item), LogicalSum> sigLocationEditRequest;

}


LocatableItem::LocatableItem()
{
    isLocationEditable_ = true;
}


LocatableItem::~LocatableItem()
{
    sigLocationExpired_();
}


LocatableItem* LocatableItem::getParentLocatableItem()
{
    if(auto item = dynamic_cast<Item*>(this)){
        return item->findOwnerItem<LocatableItem>();
    }
    return nullptr;
}


Item* LocatableItem::getCorrespondingItem()
{
    if(auto item = dynamic_cast<Item*>(this)){
        return item;
    }
    return nullptr;
}


std::string LocatableItem::getLocationName() const
{
    return const_cast<LocatableItem*>(this)->getCorrespondingItem()->displayName();
}


bool LocatableItem::isLocationEditable() const
{
    return isLocationEditable_;
}


void LocatableItem::setLocationEditable(bool on)
{
    if(on != isLocationEditable_){
        isLocationEditable_ = on;
        sigLocationAttributeChanged_();
    }
}


void LocatableItem::setLocation(const Position& /* T */)
{

}


void LocatableItem::expireLocation()
{
    sigLocationExpired_();
}


SignalProxy<void()> LocatableItem::sigLocationAttributeChanged()
{
    return sigLocationAttributeChanged_;
}


SignalProxy<void()> LocatableItem::sigLocationExpired()
{
    return sigLocationExpired_;
}


bool LocatableItem::requestLocationEdit()
{
    return ::sigLocationEditRequest(this);
}


SignalProxy<bool(LocatableItem* item), LogicalSum> LocatableItem::sigLocationEditRequest()
{
    return ::sigLocationEditRequest;
}
