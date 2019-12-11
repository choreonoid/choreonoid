#include "PolymorphicItemFunctionSet.h"
#include "ItemClassRegistry.h"

using namespace cnoid;


PolymorphicItemFunctionSet::PolymorphicItemFunctionSet()
    : PolymorphicFunctionSet<Item>(ItemClassRegistry::instance())
{

}
