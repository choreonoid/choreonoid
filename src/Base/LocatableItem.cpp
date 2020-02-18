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


bool LocatableItem::prefersLocalLocation() const
{
    return false;
}


bool LocatableItem::getLocationEditable() const
{
    return isLocationEditable_;
}


void LocatableItem::setLocationEditable(bool on)
{
    if(on != isLocationEditable_){
        isLocationEditable_ = on;
        sigLocationEditableToggled_(on);
    }
}


SignalProxy<void(bool on)> LocatableItem::sigLocationEditableToggled()
{
    return sigLocationEditableToggled_;
}


LocatableItem* LocatableItem::getParentLocatableItem()
{
    if(auto item = dynamic_cast<Item*>(this)){
        while(true){
            item = item->parentItem();
            if(item){
                if(auto locatable = dynamic_cast<LocatableItem*>(item)){
                    return locatable;
                }
            } else {
                break;
            }
        }
    }
    return nullptr;
}
