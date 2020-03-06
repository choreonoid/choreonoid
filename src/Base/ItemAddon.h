#ifndef CNOID_BASE_ITEM_ADDON_H
#define CNOID_BASE_ITEM_ADDON_H

#include <cnoid/Referenced>
#include "exportdecl.h"

namespace cnoid {

class Item;
class Archive;

class CNOID_EXPORT ItemAddon : public Referenced
{
public:
    ItemAddon();
    ItemAddon(const ItemAddon& org) = delete;
    virtual bool setOwnerItem(Item* item);
    virtual Item* ownerItem();
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

private:
    Item* ownerItem_;
};

typedef ref_ptr<ItemAddon> ItemAddonPtr;

}

#endif
