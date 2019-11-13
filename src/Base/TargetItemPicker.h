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
    TargetItemPickerBase(View* view = nullptr);
    ~TargetItemPickerBase();

    void clearTargetItem();
    void storeTargetItem(Archive& archive, const std::string& key);
    void restoreTargetItem(const Archive& archive, const std::string& key);
    void restoreTargetItemLater(const Archive& archive, const std::string& key);

protected:
    Item* getTargetItem();
    virtual void extractTargetItemCandidates(ItemList<>& io_items, bool selectionChanged) = 0;
    virtual void onTargetItemSpecified(Item* item, bool isChanged) = 0;
    virtual void onDeactivated() = 0;

private:
class Impl;
Impl* impl;

};

template<class ItemType>
class TargetItemPicker : public TargetItemPickerBase
{
public:
    typedef ref_ptr<ItemType> ItemTypePtr;
    
    TargetItemPicker() : TargetItemPickerBase() { }
    TargetItemPicker(View* view) : TargetItemPickerBase(view) { }

    ItemType* currentItem(){ return static_cast<ItemType*>(getTargetItem()); }
    const ItemList<ItemType>& selectedItems(){ return selectedItems_; }

    SignalProxy<void(ItemType* targetItem)> sigTargetItemSpecified(){
        return sigTargetItemSpecified_;
    }
    SignalProxy<void(ItemType* targetItem)> sigTargetItemChanged(){
        return sigTargetItemChanged_;
    }
    SignalProxy<void(const ItemList<ItemType>& selected)> sigSelectedItemsChanged(){
        return sigSelectedItemsChanged_;
    }

protected:
    virtual void extractTargetItemCandidates(ItemList<>& io_items, bool selectionChanged) override
    {
        auto iter = io_items.begin();
        while(iter != io_items.end()){
            if(!dynamic_cast<ItemType*>(iter->get())){
                iter = io_items.erase(iter);
            } else {
                ++iter;
            }
        }
        if(selectionChanged){
            selectedItems_ = io_items;
            sigSelectedItemsChanged_(selectedItems_);
        }
    }

    virtual void onTargetItemSpecified(Item* item, bool isChanged) override
    {
        auto targetItem = static_cast<ItemType*>(item);
        sigTargetItemSpecified_(targetItem);
        if(isChanged){
           sigTargetItemChanged_(targetItem);
        }
    }

    virtual void onDeactivated() override
    {
        selectedItems_.clear();
    }

private:
    Signal<void(ItemType* targetItem)> sigTargetItemSpecified_;
    Signal<void(ItemType* targetItem)> sigTargetItemChanged_;
    Signal<void(const ItemList<ItemType>& selected)> sigSelectedItemsChanged_;
    ItemList<ItemType> selectedItems_;
};

}

#endif
