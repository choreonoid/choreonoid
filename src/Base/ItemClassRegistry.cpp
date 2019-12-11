#include "ItemClassRegistry.h"

using namespace cnoid;


ItemClassRegistry& ItemClassRegistry::instance()
{
    static ItemClassRegistry registry;
    return registry;
}


ItemClassRegistry::ItemClassRegistry()
{

}
