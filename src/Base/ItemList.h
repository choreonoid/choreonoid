/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_ITEM_LIST_H
#define CNOID_BASE_ITEM_LIST_H

#include "Item.h"
#include <cnoid/PolymorphicPointerArray>
#include "exportdecl.h"

namespace cnoid {

class Item;

class CNOID_EXPORT ItemListBase
{
public:
    virtual bool push_back_if_type_matches(Item* item) = 0;
    bool extractSubTreeItemsSub(Item* item);
    bool extractParentItemsSub(Item* item);
    bool extractAssociatedItemsSub(Item* item);
    bool extractSubItemsSub(Item* item);
};
    

template<class ItemType>
class ItemList : public PolymorphicPointerArray<ItemType, ref_ptr<ItemType>>, public ItemListBase
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

    ItemType* toSingle(bool allowFromMultiElements = false) const {
        return (ArrayBase::size() == 1 || (allowFromMultiElements && !ArrayBase::empty())) ?
            ArrayBase::front().get() : nullptr;
    }

    //! \deprecated Use Item::descendantItems
    bool extractSubTreeItems(const Item* root){
        ArrayBase::clear();
        return ItemListBase::extractSubTreeItemsSub(const_cast<Item*>(root));
    }

    //! \deprecated Use Item::descendantItems.
    bool extractChildItems(const Item* item){
        return extractSubTreeItems(item);
    }

    bool extractAssociatedItems(const Item* item){
        ArrayBase::clear();
        return ItemListBase::extractAssociatedItemsSub(const_cast<Item*>(item));
    }
        
    bool extractSubItems(const Item* item){
        ArrayBase::clear();
        return ItemListBase::extractSubItemsSub(const_cast<Item*>(item));
    }

    ItemType* find(const std::string& name){
        for(size_t i=0; i < ArrayBase::size(); ++i){
            if((*this)[i]->name() == name){
                return (*this)[i];
            }
        }
        return nullptr;
    }

private:
    virtual bool push_back_if_type_matches(Item* item) override {
        if(ItemType* casted = dynamic_cast<ItemType*>(item)){
            ArrayBase::push_back(casted);
            return true;
        }
        return false;
    }
};

}

#endif
