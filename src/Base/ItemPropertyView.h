#ifndef CNOID_BASE_ITEM_PROPERTY_VIEW_H
#define CNOID_BASE_ITEM_PROPERTY_VIEW_H

#include <cnoid/View>
#include <functional>
#include "exportdecl.h"

namespace cnoid {

class Item;
class PutPropertyFunction;

class CNOID_EXPORT ItemPropertyView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
    static ItemPropertyView* instance();

    ItemPropertyView();
    ~ItemPropertyView();

    template<class ItemType>
    void addPropertyFunction(std::function<void(ItemType* item, PutPropertyFunction& putProperty)> func){
        addPropertyFunction_(
            typeid(ItemType),
            [func](Item* item, PutPropertyFunction& putProperty){
                func(static_cast<ItemType*>(item), putProperty);
            });
    };

    virtual void onAttachedMenuRequest(MenuManager& menuManager);

private:
    void addPropertyFunction_(
        const std::type_info& type,
        std::function<void(Item* item, PutPropertyFunction& putProperty)> func);

    class Impl;
    Impl* impl;
};

}

#endif
