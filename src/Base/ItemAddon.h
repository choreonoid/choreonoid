#ifndef CNOID_BASE_ITEM_ADDON_H
#define CNOID_BASE_ITEM_ADDON_H

#include <cnoid/Referenced>
#include "exportdecl.h"

namespace cnoid {

class ExtensionManager;
class Item;
class Archive;

class CNOID_EXPORT ItemAddon : public Referenced
{
public:
    ItemAddon();
    ItemAddon(const ItemAddon& org);
    virtual Item* ownerItem();
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

protected:
    virtual bool setOwnerItem(Item* item);

private:
    Item* ownerItem_;
    friend class Item;
};

typedef ref_ptr<ItemAddon> ItemAddonPtr;

}

#endif
