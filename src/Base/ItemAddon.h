#ifndef CNOID_BASE_ITEM_ADDON_H
#define CNOID_BASE_ITEM_ADDON_H

#include <cnoid/Referenced>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class ExtensionManager;
class Item;
class Archive;
class CloneMap;

class CNOID_EXPORT ItemAddon : public Referenced
{
public:
    ItemAddon();

    ItemAddon* clone(Item* newItem = nullptr, CloneMap* cloneMap = nullptr) const;
    const std::string& name() const;

    virtual Item* ownerItem();
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

    
protected:
    ItemAddon(const ItemAddon& org);

    /**
       \note This function does not necessarily need to be overriden. In that case, nullptr
       is returned by the default implementation, and the addon object is created and
       initialized on demand even if the item is cloned or assigned from another item with
       the corresponding addon object.
    */
    virtual ItemAddon* doClone(Item* newItem, CloneMap* cloneMap) const;
    
    virtual bool setOwnerItem(Item* item);

private:
    Item* ownerItem_;
    mutable std::string name_;
    friend class Item;
};

typedef ref_ptr<ItemAddon> ItemAddonPtr;

}

#endif
