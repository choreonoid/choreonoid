#include "Item.h"
#include "LocatableItem.h"
#include <utility>

using namespace std;
using namespace cnoid;


LocatableItem::LocatableItem()
{
    isLocationEditable_ = true;
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
    return const_cast<LocatableItem*>(this)->getCorrespondingItem()->name();
}


bool LocatableItem::isLocationEditable() const
{
    return isLocationEditable_;
}


void LocatableItem::setLocationEditable(bool on)
{
    if(on != isLocationEditable_){
        isLocationEditable_ = on;
        sigLocationEditableChanged_(on);
    }
}


SignalProxy<void(bool on)> LocatableItem::sigLocationEditableChanged()
{
    return sigLocationEditableChanged_;
}


void LocatableItem::setLocation(const Position& /* T */)
{

}


LocatableItem* LocatableItem::getParentLocatableItem()
{
    if(auto item = dynamic_cast<Item*>(this)){
        return item->findOwnerItem<LocatableItem>();
    }
    return nullptr;
}
