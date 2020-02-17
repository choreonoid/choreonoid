#include "Item.h"
#include "LocatableItem.h"
#include <utility>

using namespace std;
using namespace cnoid;

namespace {

typedef pair<const Item*, const LocatableItem*> ConstItemPair;

static ConstItemPair getParentLocatableItem(const LocatableItem* locatable)
{
    if(auto item = dynamic_cast<const Item*>(locatable)){
        while(true){
            item = item->parentItem();
            if(item){
                if(auto locatable = dynamic_cast<const LocatableItem*>(item)){
                    return make_pair(item, locatable);
                }
            } else {
                break;
            }
        }
    }
    return ConstItemPair(nullptr, nullptr);
}

}
 

bool LocatableItem::hasParentLocation() const
{
    return getParentLocatableItem(this).first != nullptr;
}


std::string LocatableItem::getParentLocationName() const
{
    if(auto parent = getParentLocatableItem(this).first){
        return parent->name();
    }
    return std::string();
}


Position LocatableItem::getParentLocation() const
{
    if(auto parent = getParentLocatableItem(this).second){
        return parent->getLocation();
    }
    return Position::Identity();
}
