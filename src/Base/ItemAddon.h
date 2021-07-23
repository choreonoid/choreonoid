#ifndef CNOID_BASE_ITEM_ADDON_H
#define CNOID_BASE_ITEM_ADDON_H

#include <cnoid/Referenced>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class ExtensionManager;
class Item;
class Archive;

class CNOID_EXPORT ItemAddon : public Referenced
{
public:
    ItemAddon();
    ItemAddon(const ItemAddon&) = delete;
    virtual Item* ownerItem();

    /**
       This function assigns the state of the source addon object to this assign object.
       This function is used when the owner item is duplicated or assigned from another item.

       \note This function does not necessarily need to be overriden. In that case, the false
       is returned by the default implementation and the addon object is created and initialized
       on demand even if the item is duplicated or assigned from another item with the
       corresponding addon object.
    */
    virtual bool assign(const ItemAddon* srcAddon);
    
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

    const std::string& name() const;
    
protected:
    virtual bool setOwnerItem(Item* item);

private:
    Item* ownerItem_;
    mutable std::string name_;
    friend class Item;
};

typedef ref_ptr<ItemAddon> ItemAddonPtr;

}

#endif
