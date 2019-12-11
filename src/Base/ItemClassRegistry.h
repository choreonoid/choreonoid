#ifndef CNOID_BASE_ITEM_CLASS_REGISTRY_H
#define CNOID_BASE_ITEM_CLASS_REGISTRY_H

#include "Item.h"
#include <cnoid/HierarchicalClassRegistry>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ItemClassRegistry : public HierarchicalClassRegistry<Item>
{
public:
    static ItemClassRegistry& instance();

private:
    ItemClassRegistry();
};

}
        
#endif

