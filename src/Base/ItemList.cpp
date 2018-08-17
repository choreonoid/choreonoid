/**
   @author Shin'ichiro Nakaoka
*/

#include "ItemList.h"

using namespace cnoid;


static bool extractChildItemsIter(ItemListBase* items, Item* item)
{
    bool extracted = false;
    if(item){
        if(items->push_back_if_type_matches(item)){
            extracted = true;
        }
        if(extractChildItemsIter(items, item->childItem())){
            extracted = true;
        }
        if(extractChildItemsIter(items, item->nextItem())){
            extracted = true;
        }
    }
    return extracted;
}


bool ItemListBase::extractChildItemsSub(Item* item)
{
    return extractChildItemsIter(this, item->childItem());
}


static bool extractParentItemsIter(ItemListBase* items, Item* item)
{
    bool extracted = false;
    Item* parent = item->parentItem();
    if(parent){
        if(extractParentItemsIter(items, parent)){
            extracted = true;
        }
        if(items->push_back_if_type_matches(item)){
            extracted = true;
        }
    }
    return extracted;
}
        
    
bool ItemListBase::extractParentItemsSub(Item* item)
{
    return extractParentItemsIter(this, item->parentItem());
}


bool ItemListBase::extractAssociatedItemsSub(Item* item)
{
    bool extracted = extractParentItemsSub(item);
    if(extractChildItemsSub(item)){
        extracted = true;
    }
    return extracted;
}


bool ItemListBase::extractSubItemsSub(Item* item)
{
    bool extracted = false;
    for(Item* child = item->childItem(); child; child = child->nextItem()){
        if(child->isSubItem()){
            if(push_back_if_type_matches(child)){
                extracted = true;
            }
        }
    }
    return extracted;
}
