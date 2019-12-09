#include "ItemClassIdRegistry.h"

using namespace std;
using namespace cnoid;


ItemClassIdRegistry* ItemClassIdRegistry::instance()
{
    static ItemClassIdRegistry manager;
    return &manager;
}


ItemClassIdRegistry::ItemClassIdRegistry()
{

}
