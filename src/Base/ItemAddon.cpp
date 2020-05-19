#include "ItemAddon.h"

using namespace cnoid;


ItemAddon::ItemAddon()
{
    ownerItem_ = nullptr;
}


ItemAddon::ItemAddon(const ItemAddon& org)
{
    ownerItem_ = nullptr;
}


bool ItemAddon::setOwnerItem(Item* item)
{
    ownerItem_ = item;
    return true;
}


Item* ItemAddon::ownerItem()
{
    return ownerItem_;
}


bool ItemAddon::store(Archive& archive)
{
    return true;
}


bool ItemAddon::restore(const Archive& archive)
{
    return true;
}

