#ifndef CNOID_BASE_ITEM_CLASS_ID_REGISTRY_H
#define CNOID_BASE_ITEM_CLASS_ID_REGISTRY_H

#include "Item.h"
#include <cnoid/HierarchicalClassIdRegistry>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ItemClassIdRegistry : public HierarchicalClassIdRegistry<Item>
{
public:
    static ItemClassIdRegistry* instance();

private:
    ItemClassIdRegistry();
};

}
        
#endif

