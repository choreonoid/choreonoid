#ifndef CNOID_BASE_TARGET_ITEM_PICKER_H
#define CNOID_BASE_TARGET_ITEM_PICKER_H

#include "Item.h"
#include "ItemList.h"
#include "exportdecl.h"

namespace cnoid {

class View;

class CNOID_EXPORT TargetItemPickerBase
{
public:
    TargetItemPickerBase(View* view);
    ~TargetItemPickerBase();

    void storeTargetItem(Archive& archive, const std::string& key);
    void restoreTargetItemLater(const Archive& archive, const std::string& key);

protected:
    Item* getTargetItem();
    virtual void extractTargetItemCandidates(ItemList<>& io_items) = 0;
    virtual void onTargetItemChanged(Item* item) = 0;

private:
class Impl;
Impl* impl;

};

template<class ItemType>
class TargetItemPicker : public TargetItemPickerBase
{
public:
    typedef ref_ptr<ItemType> ItemTypePtr;
    
    TargetItemPicker(View* view)
        : TargetItemPickerBase(view)
    { }

    ItemType* currentItem(){ return static_cast<ItemType*>(getTargetItem()); }

    SignalProxy<void(ItemType* targetItem)> sigTargetItemChanged(){
        return sigTargetItemChanged_;
    }

protected:
    virtual void extractTargetItemCandidates(ItemList<>& io_items) override
    {
        auto iter = io_items.begin();
        while(iter != io_items.end()){
            if(!dynamic_cast<ItemType*>(iter->get())){
                iter = io_items.erase(iter);
            } else {
                ++iter;
            }
        }
    }

    virtual void onTargetItemChanged(Item* item) override
    {
        sigTargetItemChanged_(static_cast<ItemType*>(item));
    }

private:
    Signal<void(ItemType* targetItem)> sigTargetItemChanged_;
};

}

#endif
