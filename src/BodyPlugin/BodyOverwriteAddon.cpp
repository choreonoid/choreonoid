#include "BodyOverwriteAddon.h"
#include "LinkOverwriteItem.h"
#include <cnoid/ItemManager>
#include <cnoid/BodyItem>
#include <cnoid/SceneDrawables>
#include <cnoid/Archive>

using namespace std;
using namespace cnoid;


namespace cnoid {

class BodyOverwriteAddon::Impl
{
public:
    BodyOverwriteAddon* self;
    BodyItem* bodyItem;

    std::map<Link*, LinkOverwriteItemPtr> linkOverwriteItemMap;
    std::map<Device*, DeviceOverwriteItemPtr> deviceOverwriteItemMap;

    Impl(BodyOverwriteAddon* self);
};

}


void BodyOverwriteAddon::initializeClass(ExtensionManager* ext)
{
    auto& im = ext->itemManager();
    im.registerAddon<BodyOverwriteAddon>("BodyOverwrite");
}


BodyOverwriteAddon::BodyOverwriteAddon()
{
    impl = new Impl(this);
}


BodyOverwriteAddon::Impl::Impl(BodyOverwriteAddon* self)
    : self(self)
{
    bodyItem = nullptr;
}


bool BodyOverwriteAddon::setOwnerItem(Item* item)
{
    impl->bodyItem = dynamic_cast<BodyItem*>(item);
    ItemAddon::setOwnerItem(impl->bodyItem);
    return bool(impl->bodyItem);
}


LinkOverwriteItem* BodyOverwriteAddon::findLinkOverwriteItem(Link* link)
{
    auto p = impl->linkOverwriteItemMap.find(link);
    if(p != impl->linkOverwriteItemMap.end()){
        return p->second;
    }
    return nullptr;
}


bool BodyOverwriteAddon::registerLinkOverwriteItem(Link* link, LinkOverwriteItem* item)
{
    auto body = link->body();
    if(!body || (body == impl->bodyItem->body())){
        auto& mappedItem = impl->linkOverwriteItemMap[link];
        if(!mappedItem){
            mappedItem = item;
            return true;
        }
    }
    return false;
}


void BodyOverwriteAddon::unregisterLinkOverwriteItem(LinkOverwriteItem* item)
{
    if(item->bodyItem() == impl->bodyItem){
        auto p = impl->linkOverwriteItemMap.begin();
        while(p != impl->linkOverwriteItemMap.end()){
            if(p->second == item){
                impl->linkOverwriteItemMap.erase(p);
                break;
            }
            ++p;
        }
    }
}


ItemList<DeviceOverwriteItem> BodyOverwriteAddon::extractDevicesAsOverwriteItems
(std::function<bool(Device* device)> predicate)
{
    ItemList<DeviceOverwriteItem> items;
    auto body = impl->bodyItem->body();
    int n = body->numDevices();
    items.reserve(n);
    Item* prevItem = nullptr;
    for(int i=0; i < n; ++i){
        auto device = body->device(i);
        bool isTarget = predicate(device);
        if(isTarget){
            DeviceOverwriteItemPtr item = findDeviceOverwriteItem(device);
            if(!item){
                item = new DeviceOverwriteItem;
                if(device->name().empty()){
                    item->setName(device->typeName());
                } else {
                    item->setName(device->name());
                } 
                if(!item->setDevice(impl->bodyItem, device, false)){
                    item.reset();
                } else {
                    if(prevItem){
                        impl->bodyItem->insertChild(prevItem->nextItem(), item);
                    } else {
                        impl->bodyItem->addChildItem(item);
                    }
                }
            }
            if(item){
                items.push_back(item);
                prevItem = item;
            }
        }
    }
    return items;                
}


DeviceOverwriteItem* BodyOverwriteAddon::findDeviceOverwriteItem(Device* device)
{
    auto p = impl->deviceOverwriteItemMap.find(device);
    if(p != impl->deviceOverwriteItemMap.end()){
        return p->second;
    }
    return nullptr;
}


bool BodyOverwriteAddon::addDeviceOverwriteItem(DeviceOverwriteItem* item)
{
    if(auto device = item->device()){
        impl->deviceOverwriteItemMap[device] = item;
        return true;
    }
    return false;
}


void BodyOverwriteAddon::removeDeviceOverwriteItem(DeviceOverwriteItem* item)
{
    auto p = impl->deviceOverwriteItemMap.find(item->device());
    if(p != impl->deviceOverwriteItemMap.end()){
        if(p->second == item){
            impl->deviceOverwriteItemMap.erase(p);
        }
    }
}


/**
   \note Currently this function only clears LinkOverwriteItems.
   It does not clear DeviceOverwriteItems because outputting overwritten devices
   into a body file is not yet supported.
*/
void BodyOverwriteAddon::clearOverwriteItems()
{
    vector<BodyElementOverwriteItem*> overwriteItems;
    overwriteItems.reserve(impl->linkOverwriteItemMap.size());
    for(auto& kv : impl->linkOverwriteItemMap){
        // note: The function to remove each item must not executed here because
        // it also removes the item from linkOverwriteItemMap and this loop crashes.
        overwriteItems.push_back(kv.second);
    }

    for(auto& item : overwriteItems){
        item->removeFromParentItem();
    }
}
