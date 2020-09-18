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

    void setEnabled(bool on);
    void clearTargetItem();
    void refresh();
    void storeTargetItem(Archive& archive, const std::string& key);
    void restoreTargetItem(const Archive& archive, const std::string& key);
    void restoreTargetItemLater(const Archive& archive, const std::string& key);

protected:
    Item* getTargetItem();
    virtual Item* extractTargetItemCandidates(ItemList<>& io_items, Item* preferred, bool selectionChanged) = 0;
    virtual bool targetPredicate(Item* item) = 0;
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

    void setTargetPredicate(std::function<bool(ItemType* item)> predicate){ targetPredicate_ = predicate; }
    
    template<class Interface>
    void setTargetInterface(){
        targetPredicate_ = [](ItemType* item)->bool { return dynamic_cast<Interface*>(item) != nullptr; };
    }

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
    virtual Item* extractTargetItemCandidates
    (ItemList<>& io_items, Item* preferred, bool selectionChanged) override
    {
        Item* candidate = nullptr;
        
        auto iter = io_items.begin();
        while(iter != io_items.end()){
            auto item = dynamic_cast<ItemType*>(iter->get());
            if(item && targetPredicate_){
                if(!targetPredicate_(item)){
                    item = nullptr;
                }
            }
            if(!item){
                iter = io_items.erase(iter);
            } else {
                if(!candidate){
                    candidate = item;
                } else if(item == preferred){
                    candidate = preferred;
                }
                ++iter;
            }
        }

        if(selectionChanged){
            if(io_items != selectedItems_){
                selectedItems_ = io_items;
                sigSelectedItemsChanged_(selectedItems_);
            }
        }

        return candidate;
    }

    virtual bool targetPredicate(Item* item) override
    {
        if(targetPredicate_){
            return targetPredicate_(static_cast<ItemType*>(item));
        }
        return true;
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
    std::function<bool(ItemType* item)> targetPredicate_;
    Signal<void(ItemType* targetItem)> sigTargetItemSpecified_;
    Signal<void(ItemType* targetItem)> sigTargetItemChanged_;
    Signal<void(const ItemList<ItemType>& selected)> sigSelectedItemsChanged_;
    ItemList<ItemType> selectedItems_;
};

}

#endif
