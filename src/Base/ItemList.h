/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_ITEM_LIST_H
#define CNOID_BASE_ITEM_LIST_H

#include "Item.h"
#include <cnoid/PolymorphicPointerArray>

namespace cnoid {

template<class ItemType = Item>
class ItemList : public PolymorphicPointerArray<ItemType, ref_ptr<ItemType> >
{
    typedef PolymorphicPointerArray<ItemType, ref_ptr<ItemType> > ArrayBase;

public:
    ItemList() { }
        
    template <class RhsObjectType>
    ItemList(const ItemList<RhsObjectType>& rhs)
        : ArrayBase(rhs) { }

    template <class SubType>
    SubType* get(int index) const { return dynamic_cast<SubType*>(ArrayBase::operator[](index).get()); }

    ItemType* get(int index) const { return ArrayBase::operator[](index).get(); }

    bool extractChildItems(ItemPtr item) {
        ArrayBase::clear();
        extractChildItemsSub(item->childItem());
        return !ArrayBase::empty();
    }
        
    ItemType* toSingle(bool allowFromMultiElements = false) const {
        return (ArrayBase::size() == 1 || (allowFromMultiElements && !ArrayBase::empty())) ?
            ArrayBase::front().get() : 0;
    }

    bool extractSubItems(ItemPtr item){
        ArrayBase::clear();
        for(Item* child = item->childItem(); child; child = child->nextItem()){
            if(child->isSubItem()){
                ItemType* targetItem = dynamic_cast<ItemType*>(child);
                if(targetItem){
                    ArrayBase::push_back(targetItem);
                }
            }
        }
        return !ArrayBase::empty();
    }

    ItemType* find(const std::string& name){
        for(size_t i=0; i < ArrayBase::size(); ++i){
            if((*this)[i]->name() == name){
                return (*this)[i];
            }
        }
        return 0;
    }

private:
    void extractChildItemsSub(Item* item){
        if(item){
            ItemType* targetItem = dynamic_cast<ItemType*>(item);
            if(targetItem){
                ArrayBase::push_back(targetItem);
            }
            extractChildItemsSub(item->childItem());
            extractChildItemsSub(item->nextItem());
        }
    }
};

}

#endif
