#include "ItemAddon.h"
#include "ItemManager.h"

using namespace std;
using namespace cnoid;


ItemAddon::ItemAddon()
{
    ownerItem_ = nullptr;
}


bool ItemAddon::setOwnerItem(Item* item)
{
    ownerItem_ = item;
    return true;
}


bool ItemAddon::assign(const ItemAddon* /* srcAddon */)
{
    return false;
}


Item* ItemAddon::ownerItem()
{
    return ownerItem_;
}


bool ItemAddon::store(Archive& archive)
{
    return false;
}


bool ItemAddon::restore(const Archive& archive)
{
    return false;
}


const std::string& ItemAddon::name() const
{
    if(name_.empty()){
        string moduleName;
        ItemManager::getAddonIdentifier(this, moduleName, name_);
    }
    return name_;
}
