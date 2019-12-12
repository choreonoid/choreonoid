#ifndef CNOID_BASE_POLYMORPHIC_ITEM_FUNCTION_SET_H
#define CNOID_BASE_POLYMORPHIC_ITEM_FUNCTION_SET_H

#include "Item.h"
#include <cnoid/PolymorphicFunctionSet>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT PolymorphicItemFunctionSet : public PolymorphicFunctionSet<Item>
{
public:
    PolymorphicItemFunctionSet();
};

typedef PolymorphicItemFunctionSet::Dispatcher ItemFunctionDispatcher;

}

#endif
